// SPDX-License-Identifier: Apache-2.0
//! The tessellation of one face.

use std::collections::BTreeSet;

use cadmpeg_ir::ids::{CoedgeId, FaceId};
use cadmpeg_ir::index::StandardIndex;
use cadmpeg_ir::math::{Point3, Vector3};
use cadmpeg_ir::topology::{Face, LoopBoundary, Sense};

use crate::boundary::{trace, EdgeCache, Sampler, Traced, TracedLoop};
use crate::domain::{regions, reversed, Frame, Node, Region, Ring};
use crate::plane::{cross, distance, dot, length, midpoint, scaled, sub, unit, Uv};
use crate::refine::{Limits, Mesh};
use crate::surface::Carrier;
use crate::triangulate::{triangulate, Rings};
use crate::Tessellator;
use cadmpeg_core::decode::WorkBudget;

/// Ratio of a parameter gap between two joined coedges to the size of
/// their parameters above which the gap is a jump across a pole.
const JUNCTION_RATIO: f64 = 1e-7;
/// Evaluation work one face may spend.
const FACE_WORK: usize = 1 << 28;
/// Deepest bisection of one derived pcurve segment.
const PCURVE_DEPTH: u32 = 24;
/// Largest sample count of one derived pcurve.
const MAX_PCURVE_SAMPLES: usize = 1 << 16;
/// Ratio of a triangle's model area to the squared surface size below which
/// it is dropped as degenerate.
const DEGENERATE_AREA_RATIO: f64 = 1e-20;

/// Why a face has no tessellation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal(String);

impl Refusal {
    pub(crate) fn new(reason: impl Into<String>) -> Self {
        Self(reason.into())
    }

    /// The reason.
    #[must_use]
    pub fn reason(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for Refusal {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for Refusal {}

/// One coedge of a face in the face's parameter plane, in traversal order.
#[derive(Debug, Clone, PartialEq)]
pub struct CoedgeTrace {
    /// The traced coedge.
    pub coedge: CoedgeId,
    /// Parameters on the face's surface. A seam traced twice by one loop
    /// lies a whole period apart in its two uses.
    pub uv: Vec<[f64; 2]>,
    /// The model points of the samples: edge-curve points, or the pole of a
    /// degenerate edge.
    pub points: Vec<Point3>,
    /// The parameters come from the coedge's own pcurve.
    pub from_pcurve: bool,
}

/// The triangles of one face.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceMesh {
    /// Vertex positions: edge samples on the boundary, surface points
    /// inside.
    pub positions: Vec<Point3>,
    /// Unit normals, oriented with the face.
    pub normals: Vec<Vector3>,
    /// Counterclockwise about the face's oriented normal.
    pub triangles: Vec<[usize; 3]>,
    /// The coedges of every loop, in loop order.
    pub coedges: Vec<CoedgeTrace>,
    /// The largest distance between a boundary sample and the surface.
    pub deviation: f64,
}

/// A ring of domain nodes from a traced loop, oriented so that the face lies
/// on its left.
fn ring(traced: &TracedLoop<'_>, left: bool) -> Ring {
    let mut nodes = Vec::new();
    let count = traced.coedges.len();
    for (position, coedge) in traced.coedges.iter().enumerate() {
        let next = traced.coedges[(position + 1) % count].uv.first().copied();
        let samples = coedge.uv.len();
        for (at, (uv, point)) in coedge.uv.iter().zip(&coedge.points).enumerate() {
            let last = at + 1 == samples;
            // A junction kept on both branches of a pole joins them along
            // the pole line.
            let jump = last
                && position + 1 < count
                && next.is_some_and(|next| {
                    let scale = 1.0 + uv[0].abs() + uv[1].abs();
                    (next[0] - uv[0]).abs().max((next[1] - uv[1]).abs()) > JUNCTION_RATIO * scale
                });
            if last && !jump {
                continue;
            }
            nodes.push(Node {
                uv: *uv,
                point: Some(*point),
                open: coedge.degenerate || jump,
            });
        }
    }
    if left {
        return Ring {
            nodes,
            closure: traced.closure,
        };
    }
    Ring {
        nodes: reversed(&nodes),
        closure: [-traced.closure[0], -traced.closure[1]],
    }
}

pub(crate) fn tessellate(
    tessellator: &mut Tessellator<'_>,
    face_id: &FaceId,
) -> Result<FaceMesh, Refusal> {
    let index = &tessellator.index;
    let options = tessellator.options;
    let face = index
        .faces(face_id.as_str(), StandardIndex)
        .ok_or_else(|| Refusal::new(format!("the face {face_id} is not in the model")))?;
    let work = WorkBudget::new(FACE_WORK);
    let carrier = Carrier::new(index, &work, &face.surface)?;
    let sense = if face.sense == Sense::Reversed {
        -1.0
    } else {
        1.0
    };
    let orientation = sense * carrier.normal_sign;
    let left = orientation > 0.0;
    let sampler = Sampler {
        index,
        work: &work,
        chord: options.chord,
        angle: options.angle,
    };
    if carrier.fallback_blend() {
        return boundary_fallback(&sampler, &mut tessellator.edges, &face, &carrier);
    }
    let mut traced_loops = Vec::new();
    let mut poles: Vec<(Uv, Option<Point3>)> = Vec::new();
    for loop_id in &face.loops {
        let loop_ = index
            .loops(loop_id.as_str(), StandardIndex)
            .ok_or_else(|| Refusal::new(format!("the loop {loop_id} is not in the model")))?;
        match &loop_.boundary {
            LoopBoundary::Vertex { vertex, .. } => {
                let vertex = index
                    .vertices(vertex.as_str(), StandardIndex)
                    .and_then(|vertex| index.points(vertex.point.as_str(), StandardIndex));
                if let Some(point) = vertex {
                    let point = point.position().get();
                    if let Some(uv) = carrier.invert(point, None)? {
                        poles.push((uv, Some(point)));
                    }
                }
            }
            LoopBoundary::Ring(ring) => {
                let coedges = ring
                    .coedges()
                    .iter()
                    .map(|id| {
                        index.coedges(id.as_str(), StandardIndex).ok_or_else(|| {
                            Refusal::new(format!("the coedge {id} is not in the model"))
                        })
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                traced_loops.push(trace(
                    &sampler,
                    &mut tessellator.edges,
                    &carrier,
                    &coedges,
                    left,
                )?);
            }
        }
    }
    if let Some(apex) = carrier.apex {
        poles.push(([0.0, apex], carrier.point([0.0, apex])?));
    }
    let frame = Frame {
        periods: carrier.periods,
        domain: carrier.domain,
        poles,
    };
    let rings = traced_loops
        .iter()
        .map(|traced| ring(traced, left))
        .collect();
    let regions = regions(rings, &frame)?;
    let mut mesh = Mesh {
        uv: Vec::new(),
        points: Vec::new(),
        normals: Vec::new(),
        scales: Vec::new(),
        triangles: Vec::new(),
        fixed: BTreeSet::new(),
    };
    for region in regions {
        let mut rings_by_index = Vec::new();
        for nodes in std::iter::once(&region.outer).chain(&region.holes) {
            let start = mesh.uv.len();
            for node in nodes {
                let point = match node.point {
                    Some(point) => point,
                    None => carrier
                        .point(node.uv)?
                        .ok_or_else(|| Refusal::new("a domain corner has no surface point"))?,
                };
                mesh.uv.push(node.uv);
                mesh.points.push(point);
                mesh.normals.push(carrier.normal(node.uv)?);
                mesh.scales.push(crate::refine::scales(&carrier, node.uv)?);
            }
            let ring: Vec<usize> = (start..mesh.uv.len()).collect();
            for (at, node) in nodes.iter().enumerate() {
                if !node.open {
                    let from = ring[at];
                    let to = ring[(at + 1) % ring.len()];
                    mesh.fixed.insert((from.min(to), from.max(to)));
                }
            }
            rings_by_index.push(ring);
        }
        let outer = rings_by_index.remove(0);
        let triangles = triangulate(Rings {
            points: &mesh.uv,
            outer,
            holes: rings_by_index,
        })?;
        mesh.triangles.extend(triangles.into_iter().map(Some));
    }
    mesh.flip();
    let limits = Limits {
        chord: options.chord,
        angle: options.angle,
        // A reconstructed blend's contact chart can have parameter kinks that
        // keep subdividing without improving the chord. Bound its share of the
        // mesh while every evaluated point remains on the blend.
        max_vertices: if carrier.reconstructed_blend() {
            options.max_vertices.min(20_000)
        } else {
            options.max_vertices
        },
    };
    if !carrier.planar() {
        mesh.seed(&carrier, &limits)?;
    }
    mesh.refine(&carrier, &limits)?;
    mesh.flip();
    let mut coedges = Vec::new();
    let mut deviation = 0.0_f64;
    for traced in &traced_loops {
        for coedge in &traced.coedges {
            let derived = match options.pcurve_tolerance {
                Some(tolerance) if !coedge.from_pcurve && !coedge.degenerate => {
                    derive(&sampler, &carrier, coedge, tolerance)?
                }
                _ => (coedge.uv.clone(), coedge.points.clone()),
            };
            for (uv, point) in derived.0.iter().zip(&derived.1) {
                if coedge.degenerate {
                    continue;
                }
                if let Some((gap, _)) = carrier.deviation(*point, *uv)? {
                    deviation = deviation.max(gap);
                }
            }
            coedges.push(CoedgeTrace {
                coedge: coedge.coedge.id.clone(),
                uv: derived.0,
                points: derived.1,
                from_pcurve: coedge.from_pcurve,
            });
        }
    }
    Ok(finish(&carrier, mesh, orientation, coedges, deviation))
}

/// Meshes a face whose carrier has no usable parameterization from its
/// boundary alone. The loops are projected onto the plane that best follows
/// the largest loop, triangulated there, and lifted back to the sampled
/// points. This keeps procedural blends visible when no exact section can be
/// reconstructed; it does not add interior surface samples.
fn boundary_fallback<'ir>(
    sampler: &Sampler<'_, 'ir>,
    cache: &mut EdgeCache,
    face: &'ir Face,
    carrier: &Carrier<'_, 'ir>,
) -> Result<FaceMesh, Refusal> {
    let index = sampler.index;
    let mut loops: Vec<Vec<Point3>> = Vec::new();
    let mut traces: Vec<(CoedgeTrace, Vec<Point3>)> = Vec::new();
    for loop_id in &face.loops {
        let loop_ = index
            .loops(loop_id.as_str(), StandardIndex)
            .ok_or_else(|| Refusal::new(format!("the loop {loop_id} is not in the model")))?;
        let LoopBoundary::Ring(ring) = &loop_.boundary else {
            continue;
        };
        let mut points = Vec::new();
        for id in ring.coedges() {
            let coedge = index
                .coedges(id.as_str(), StandardIndex)
                .ok_or_else(|| Refusal::new(format!("the coedge {id} is not in the model")))?;
            let edge = index
                .edges(coedge.edge.as_str(), StandardIndex)
                .ok_or_else(|| {
                    Refusal::new(format!("the edge {} is not in the model", coedge.edge))
                })?;
            let samples = sampler.edge(cache, edge)?;
            let mut lane = samples.points.clone();
            if coedge.sense == Sense::Reversed {
                lane.reverse();
            }
            for point in &lane {
                append_unique(&mut points, *point);
            }
            traces.push((
                CoedgeTrace {
                    coedge: coedge.id.clone(),
                    uv: Vec::new(),
                    points: lane.clone(),
                    from_pcurve: false,
                },
                lane,
            ));
        }
        if points.len() > 1
            && distance(points[0], *points.last().unwrap_or(&points[0]))
                <= sample_tolerance(&points)
        {
            points.pop();
        }
        if points.len() >= 3 {
            loops.push(points);
        }
    }
    if loops.is_empty() {
        return Err(Refusal::new("the face boundary has no closed loop"));
    }
    let normal = loops
        .iter()
        .max_by_key(|points| points.len())
        .and_then(|points| newell(points))
        .unwrap_or(Vector3::new(0.0, 0.0, 1.0));
    let origin = loops
        .iter()
        .flat_map(|points| points.iter())
        .fold(Point3::new(0.0, 0.0, 0.0), |sum, point| {
            Point3::new(sum.x + point.x, sum.y + point.y, sum.z + point.z)
        });
    let count = loops.iter().map(Vec::len).sum::<usize>().max(1);
    let origin = Point3::new(
        origin.x / count as f64,
        origin.y / count as f64,
        origin.z / count as f64,
    );
    let first_axis = loops
        .iter()
        .find_map(|points| {
            points.windows(2).find_map(|pair| {
                let direction = unit(sub(pair[1], pair[0]))?;
                let along = scaled(normal, dot(direction, normal));
                unit(Vector3::new(
                    direction.x - along.x,
                    direction.y - along.y,
                    direction.z - along.z,
                ))
            })
        })
        .unwrap_or_else(|| {
            let fallback = if normal.x.abs() < 0.9 {
                Vector3::new(1.0, 0.0, 0.0)
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            };
            unit(cross(normal, fallback)).unwrap_or(Vector3::new(1.0, 0.0, 0.0))
        });
    let second_axis = cross(normal, first_axis);
    let project = |point: Point3| -> Uv {
        let offset = sub(point, origin);
        [dot(offset, first_axis), dot(offset, second_axis)]
    };
    let rings: Vec<Ring> = loops
        .iter()
        .map(|points| Ring {
            nodes: points
                .iter()
                .map(|point| Node {
                    uv: project(*point),
                    point: Some(*point),
                    open: false,
                })
                .collect(),
            closure: [0.0, 0.0],
        })
        .collect();
    let frame = Frame {
        periods: [None, None],
        domain: None,
        poles: Vec::new(),
    };
    let regions = regions(rings, &frame).unwrap_or_else(|_| {
        loops
            .iter()
            .map(|points| Region {
                outer: points
                    .iter()
                    .map(|point| Node {
                        uv: project(*point),
                        point: Some(*point),
                        open: false,
                    })
                    .collect(),
                holes: Vec::new(),
            })
            .collect()
    });
    let mut mesh = Mesh {
        uv: Vec::new(),
        points: Vec::new(),
        normals: Vec::new(),
        scales: Vec::new(),
        triangles: Vec::new(),
        fixed: BTreeSet::new(),
    };
    for region in regions {
        let mut rings_by_index = Vec::new();
        for nodes in std::iter::once(&region.outer).chain(&region.holes) {
            let start = mesh.uv.len();
            for node in nodes {
                let Some(point) = node.point else {
                    continue;
                };
                mesh.uv.push(node.uv);
                mesh.points.push(point);
                mesh.normals.push(None);
                mesh.scales.push([1.0, 1.0]);
            }
            if mesh.uv.len() > start {
                rings_by_index.push((start..mesh.uv.len()).collect::<Vec<_>>());
            }
        }
        if rings_by_index.is_empty() {
            continue;
        }
        let outer = rings_by_index.remove(0);
        if let Ok(triangles) = triangulate(Rings {
            points: &mesh.uv,
            outer,
            holes: rings_by_index,
        }) {
            mesh.triangles.extend(triangles.into_iter().map(Some));
        }
    }
    let mut coedges = Vec::new();
    for (mut trace, points) in traces {
        trace.uv = points.iter().map(|point| project(*point)).collect();
        coedges.push(trace);
    }
    Ok(finish(carrier, mesh, 1.0, coedges, 0.0))
}

fn append_unique(points: &mut Vec<Point3>, point: Point3) {
    let scale = point.x.abs().max(point.y.abs()).max(point.z.abs()).max(1.0);
    if points
        .last()
        .is_none_or(|last| distance(*last, point) > 1.0e-9 * scale)
    {
        points.push(point);
    }
}

fn sample_tolerance(points: &[Point3]) -> f64 {
    let low = points.iter().fold(
        Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        |low, point| Point3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z)),
    );
    let high = points.iter().fold(
        Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        |high, point| {
            Point3::new(
                high.x.max(point.x),
                high.y.max(point.y),
                high.z.max(point.z),
            )
        },
    );
    let diagonal = distance(low, high);
    diagonal * 1.0e-9
}

fn newell(points: &[Point3]) -> Option<Vector3> {
    let origin = points
        .iter()
        .fold(Point3::new(0.0, 0.0, 0.0), |sum, point| {
            Point3::new(sum.x + point.x, sum.y + point.y, sum.z + point.z)
        });
    let count = points.len().max(1) as f64;
    let origin = Point3::new(origin.x / count, origin.y / count, origin.z / count);
    let mut normal = Vector3::new(0.0, 0.0, 0.0);
    for (at, point) in points.iter().enumerate() {
        let next = points[(at + 1) % points.len()];
        let a = sub(*point, origin);
        let b = sub(next, origin);
        let cross = cross(a, b);
        normal = Vector3::new(normal.x + cross.x, normal.y + cross.y, normal.z + cross.z);
    }
    unit(normal)
}

/// The derived pcurve of a coedge: its trace bisected until each segment
/// mapped onto the surface lies within `tolerance` of the edge curve.
fn derive(
    sampler: &Sampler<'_, '_>,
    carrier: &Carrier<'_, '_>,
    traced: &Traced<'_>,
    tolerance: f64,
) -> Result<(Vec<Uv>, Vec<Point3>), Refusal> {
    let Some(edge) = sampler
        .index
        .edges(traced.coedge.edge.as_str(), StandardIndex)
    else {
        return Ok((traced.uv.clone(), traced.points.clone()));
    };
    let Some(curve) = edge.curve() else {
        return Ok((traced.uv.clone(), traced.points.clone()));
    };
    let derivation = Derivation {
        sampler,
        carrier,
        curve,
        tolerance,
    };
    let mut out = (vec![traced.uv[0]], vec![traced.points[0]]);
    for at in 1..traced.uv.len() {
        let a = (
            traced.params[at - 1],
            traced.uv[at - 1],
            traced.points[at - 1],
        );
        let b = (traced.params[at], traced.uv[at], traced.points[at]);
        derivation.bisect(a, b, 0, &mut out)?;
        if out.0.len() > MAX_PCURVE_SAMPLES {
            return Ok((traced.uv.clone(), traced.points.clone()));
        }
    }
    Ok(out)
}

/// The fixed inputs of one derived pcurve.
struct Derivation<'s, 'a, 'b, 'c, 'd> {
    sampler: &'s Sampler<'a, 'b>,
    carrier: &'s Carrier<'c, 'd>,
    curve: &'s cadmpeg_ir::ids::CurveId,
    tolerance: f64,
}

impl Derivation<'_, '_, '_, '_, '_> {
    fn bisect(
        &self,
        a: (f64, Uv, Point3),
        b: (f64, Uv, Point3),
        depth: u32,
        out: &mut (Vec<Uv>, Vec<Point3>),
    ) -> Result<(), Refusal> {
        let middle = f64::midpoint(a.0, b.0);
        let seed = midpoint(a.1, b.1);
        let Some(on_curve) = self.sampler.curve_point(self.curve, middle)? else {
            keep(b, out);
            return Ok(());
        };
        let Some(on_chord) = self.carrier.point(seed)? else {
            keep(b, out);
            return Ok(());
        };
        let projected = self.carrier.project(on_curve, seed)?.unwrap_or(seed);
        let projected = [
            self.carrier.nearest_branch(0, projected[0], seed[0]),
            self.carrier.nearest_branch(1, projected[1], seed[1]),
        ];
        // A curve that lies off the surface by itself cannot be followed more
        // closely than that distance.
        let off = self
            .carrier
            .point(projected)?
            .map_or(0.0, |on_surface| distance(on_curve, on_surface));
        match distance(on_curve, on_chord) {
            gap if gap > self.tolerance + off && depth < PCURVE_DEPTH => {
                let mid = (middle, projected, on_curve);
                self.bisect(a, mid, depth + 1, out)?;
                self.bisect(mid, b, depth + 1, out)
            }
            _ => {
                keep(b, out);
                Ok(())
            }
        }
    }
}

fn keep(b: (f64, Uv, Point3), out: &mut (Vec<Uv>, Vec<Point3>)) {
    out.0.push(b.1);
    out.1.push(b.2);
}

/// The output mesh: triangles wound about the oriented normal, unit vertex
/// normals, and no degenerate triangles.
fn finish(
    carrier: &Carrier<'_, '_>,
    mesh: Mesh,
    orientation: f64,
    coedges: Vec<CoedgeTrace>,
    deviation: f64,
) -> FaceMesh {
    let smallest = DEGENERATE_AREA_RATIO * carrier.size * carrier.size;
    let mut triangles = Vec::new();
    let mut accumulated: Vec<Vector3> = vec![Vector3::new(0.0, 0.0, 0.0); mesh.points.len()];
    for [a, b, c] in mesh.triangles.into_iter().flatten() {
        let (pa, pb, pc) = (mesh.points[a], mesh.points[b], mesh.points[c]);
        let normal = cross(sub(pb, pa), sub(pc, pa));
        if length(normal) <= smallest {
            continue;
        }
        let triangle = if orientation > 0.0 {
            [a, b, c]
        } else {
            [a, c, b]
        };
        let facing = scaled(normal, orientation);
        for corner in [a, b, c] {
            let sum = accumulated[corner];
            accumulated[corner] =
                Vector3::new(sum.x + facing.x, sum.y + facing.y, sum.z + facing.z);
        }
        triangles.push(triangle);
    }
    let normals = mesh
        .normals
        .iter()
        .zip(&accumulated)
        .map(|(surface, sum)| {
            let fallback = unit(*sum).unwrap_or(Vector3::new(0.0, 0.0, 1.0));
            match surface.and_then(unit) {
                Some(normal) => {
                    let oriented = scaled(normal, orientation);
                    if dot(oriented, fallback) < 0.0 && length(*sum) > 0.0 {
                        scaled(oriented, -1.0)
                    } else {
                        oriented
                    }
                }
                None => fallback,
            }
        })
        .collect();
    FaceMesh {
        positions: mesh.points,
        normals,
        triangles,
        coedges,
        deviation,
    }
}

#[cfg(test)]
mod tests;
