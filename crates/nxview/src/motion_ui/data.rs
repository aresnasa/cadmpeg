// SPDX-License-Identifier: Apache-2.0
//! Motion UI tree data, JSON schema, and scene-derived fallback.

use std::path::Path;

use serde::Deserialize;
use serde_json::Value;

use crate::scene::Scene;

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct MotionUiData {
    pub title: String,
    pub assembly: String,
    pub scenario: Scenario,
    pub groups: Vec<String>,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub couplers: Vec<Coupler>,
    pub solution: Option<Solution>,
    pub channels: Vec<Channel>,
    pub flex_modes: Vec<Value>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Scenario {
    pub name: String,
}

/// One editable parameter projected by `motion-ui-data`.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Property {
    pub path: String,
    pub label_zh: String,
    pub label_en: String,
    pub value: Value,
    pub value_type: String,
    pub unit: Option<String>,
    pub writable: bool,
    pub nxopen_writable: bool,
    pub provider: Option<String>,
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Body {
    pub name: String,
    /// Source part file when the producer could resolve it. `motion-ui-data`
    /// emits `null` because a `.sim` link does not carry a part mapping yet.
    pub source: Option<String>,
    pub properties: Vec<Property>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Joint {
    pub name: String,
    #[serde(rename = "type")]
    pub joint_type: String,
    pub properties: Vec<Property>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Coupler {
    pub name: String,
    #[serde(rename = "type")]
    pub coupler_type: String,
    pub ratio: Option<f64>,
    pub properties: Vec<Property>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Solution {
    pub name: String,
    pub time: Option<f64>,
    pub steps: Option<u32>,
    pub properties: Vec<Property>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(default)]
pub struct Channel {
    pub path: String,
    pub unit: String,
}

impl MotionUiData {
    pub fn parse(text: &str) -> serde_json::Result<Self> {
        serde_json::from_str(text)
    }

    pub fn display_title(&self) -> String {
        if !self.title.is_empty() {
            return self.title.clone();
        }
        if !self.scenario.name.is_empty() {
            return self.scenario.name.clone();
        }
        if !self.assembly.is_empty() {
            return self.assembly.clone();
        }
        String::from("Motion")
    }

    pub fn solution_name(&self) -> Option<&str> {
        self.solution
            .as_ref()
            .map(|solution| solution.name.as_str())
            .filter(|name| !name.is_empty())
    }

    fn ensure_labels(&mut self, fallback_title: &str) {
        if self.title.is_empty() {
            self.title = fallback_title.to_string();
        }
        if self.scenario.name.is_empty() {
            self.scenario.name = self.title.clone();
        }
        if self.assembly.is_empty() {
            self.assembly = fallback_title.to_string();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeKind {
    Scenario,
    Assembly,
    Group,
    Body,
    Joint,
    Coupler,
    Solution,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TreeNode {
    pub label: String,
    pub kind: NodeKind,
    pub depth: usize,
    pub detail: Option<String>,
}

impl TreeNode {
    fn new(label: impl Into<String>, kind: NodeKind, depth: usize, detail: Option<String>) -> Self {
        Self {
            label: label.into(),
            kind,
            depth,
            detail,
        }
    }
}

/// Flat, indented rows in the order the spec lists them.
pub fn tree_nodes(data: &MotionUiData) -> Vec<TreeNode> {
    let mut nodes = Vec::new();
    nodes.push(TreeNode::new(
        scenario_label(data),
        NodeKind::Scenario,
        0,
        None,
    ));
    if !data.assembly.is_empty() {
        nodes.push(TreeNode::new(
            data.assembly.clone(),
            NodeKind::Assembly,
            1,
            Some(String::from("装配")),
        ));
    }
    for group in &data.groups {
        nodes.push(TreeNode::new(
            group.clone(),
            NodeKind::Group,
            1,
            Some(String::from("定制组")),
        ));
    }
    if !data.bodies.is_empty() {
        nodes.push(TreeNode::new(
            "运动体",
            NodeKind::Group,
            1,
            Some(format!("{} 个", data.bodies.len())),
        ));
        for body in &data.bodies {
            nodes.push(TreeNode::new(
                body.name.clone(),
                NodeKind::Body,
                2,
                body.source.clone(),
            ));
        }
    }
    if !data.joints.is_empty() {
        nodes.push(TreeNode::new(
            "运动副",
            NodeKind::Group,
            1,
            Some(format!("{} 个", data.joints.len())),
        ));
        for joint in &data.joints {
            nodes.push(TreeNode::new(
                joint.name.clone(),
                NodeKind::Joint,
                2,
                Some(joint.joint_type.clone()),
            ));
        }
    }
    if !data.couplers.is_empty() {
        nodes.push(TreeNode::new(
            "耦合副",
            NodeKind::Group,
            1,
            Some(format!("{} 个", data.couplers.len())),
        ));
        for coupler in &data.couplers {
            let detail = match (&coupler.coupler_type, coupler.ratio) {
                (kind, Some(ratio)) if !kind.is_empty() => format!("{kind} · {ratio}"),
                (kind, None) if !kind.is_empty() => kind.clone(),
                (_, Some(ratio)) => format!("ratio {ratio}"),
                _ => String::new(),
            };
            nodes.push(TreeNode::new(
                coupler.name.clone(),
                NodeKind::Coupler,
                2,
                Some(detail),
            ));
        }
    }
    if let Some(name) = data.solution_name() {
        nodes.push(TreeNode::new(
            name.to_string(),
            NodeKind::Solution,
            1,
            Some(String::from("解算方案")),
        ));
    }
    nodes
}

/// Properties attached to one navigator row, if the row names an object.
pub fn node_properties<'a>(
    data: &'a MotionUiData,
    node: &TreeNode,
) -> Option<(&'a str, &'a [Property])> {
    match node.kind {
        NodeKind::Body => data
            .bodies
            .iter()
            .find(|body| body.name == node.label)
            .map(|body| (body.name.as_str(), body.properties.as_slice())),
        NodeKind::Joint => data
            .joints
            .iter()
            .find(|joint| joint.name == node.label)
            .map(|joint| (joint.name.as_str(), joint.properties.as_slice())),
        NodeKind::Coupler => data
            .couplers
            .iter()
            .find(|coupler| coupler.name == node.label)
            .map(|coupler| (coupler.name.as_str(), coupler.properties.as_slice())),
        NodeKind::Solution => data
            .solution
            .as_ref()
            .filter(|solution| solution.name == node.label)
            .map(|solution| (solution.name.as_str(), solution.properties.as_slice())),
        NodeKind::Scenario | NodeKind::Assembly | NodeKind::Group => None,
    }
}

fn scenario_label(data: &MotionUiData) -> String {
    if data.scenario.name.is_empty() {
        data.display_title()
    } else {
        data.scenario.name.clone()
    }
}

/// Build a usable tree when the user supplies no `--data` file.
pub fn synthesize(scene: &Scene) -> MotionUiData {
    let stem = scene.source.file_stem().map_or_else(
        || String::from("motion"),
        |stem| stem.to_string_lossy().into(),
    );
    let assembly = scene
        .assembly
        .file_name()
        .map_or_else(|| stem.clone(), |name| name.to_string_lossy().into());
    let mut data = MotionUiData {
        title: stem.clone(),
        assembly: assembly.clone(),
        scenario: Scenario { name: stem },
        groups: vec![String::from("定制组")],
        ..MotionUiData::default()
    };

    let visible: Vec<_> = scene.parts.iter().filter(|part| part.visible).collect();
    let parts: Vec<_> = if visible.is_empty() {
        scene.parts.iter().collect()
    } else {
        visible
    };
    for part in parts {
        data.bodies.push(Body {
            name: part.label.clone(),
            source: Some(part.label.clone()),
            properties: Vec::new(),
        });
    }

    if let Some(model) = &scene.motion_model {
        for driver in &model.drivers {
            data.joints.push(Joint {
                name: driver.name.clone(),
                joint_type: String::from("Driver"),
                properties: Vec::new(),
            });
        }
    }

    let range = scene
        .rmd_path
        .as_deref()
        .and_then(|path| crate::motion_model::read_range(path).ok());
    data.solution = Some(Solution {
        name: String::from("Solution_1"),
        time: range.as_ref().map(|range| range.end_time),
        steps: range.as_ref().map(|range| range.steps),
        properties: Vec::new(),
    });
    data.ensure_labels(&assembly);
    data
}

pub fn flex_mode_label(value: &Value) -> String {
    if let Some(text) = value.as_str() {
        return text.to_string();
    }
    if let Some(object) = value.as_object() {
        let name = object
            .get("name")
            .or_else(|| object.get("mode"))
            .and_then(Value::as_str)
            .unwrap_or("");
        let frequency = object
            .get("frequency")
            .or_else(|| object.get("freq"))
            .and_then(Value::as_f64);
        return match (name.is_empty(), frequency) {
            (false, Some(frequency)) => format!("{name} · {frequency:.2} Hz"),
            (false, None) => name.to_string(),
            (true, Some(frequency)) => format!("{frequency:.2} Hz"),
            (true, None) => String::from("模态"),
        };
    }
    value.to_string()
}

/// Load `--data`, or synthesize one. Kept here so the CLI and tests share it.
pub fn load_or_synthesize(path: Option<&Path>, scene: &Scene) -> anyhow::Result<MotionUiData> {
    let Some(path) = path else {
        return Ok(synthesize(scene));
    };
    let text = std::fs::read_to_string(path)
        .map_err(|error| anyhow::anyhow!("cannot read {}: {error}", path.display()))?;
    let mut data = MotionUiData::parse(&text)
        .map_err(|error| anyhow::anyhow!("cannot parse {}: {error}", path.display()))?;
    let fallback = scene.source.file_stem().map_or_else(
        || String::from("motion"),
        |stem| stem.to_string_lossy().into(),
    );
    data.ensure_labels(&fallback);
    Ok(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"{
      "title": "motion_1",
      "assembly": "齿轮泵装配.prt",
      "scenario": {"name": "motion_1"},
      "groups": ["定制组"],
      "bodies": [{"name": "L001", "source": "2.主动齿轮轴.prt"}],
      "joints": [{"name": "J001", "type": "Fixed"}],
      "couplers": [{"name": "J004", "type": "Gear", "ratio": 1.0}],
      "solution": {"name": "Solution_1", "time": 5.0, "steps": 200},
      "channels": [{"path": "/Bodies/L001/Pos_TX", "unit": "mm"}],
      "flex_modes": []
    }"#;

    #[test]
    fn parses_the_spec_sample() {
        let data = MotionUiData::parse(SAMPLE).expect("sample parses");
        assert_eq!(data.title, "motion_1");
        assert_eq!(data.assembly, "齿轮泵装配.prt");
        assert_eq!(data.scenario.name, "motion_1");
        assert_eq!(data.groups, ["定制组"]);
        assert_eq!(data.bodies.len(), 1);
        assert_eq!(data.bodies[0].source.as_deref(), Some("2.主动齿轮轴.prt"));
        assert_eq!(data.joints[0].joint_type, "Fixed");
        assert_eq!(data.couplers[0].coupler_type, "Gear");
        assert_eq!(data.couplers[0].ratio, Some(1.0));
        let solution = data.solution.expect("solution");
        assert_eq!(solution.time, Some(5.0));
        assert_eq!(solution.steps, Some(200));
        assert_eq!(data.channels[0].unit, "mm");
        assert!(data.flex_modes.is_empty());
    }

    #[test]
    fn missing_fields_take_defaults() {
        let data = MotionUiData::parse("{}").expect("empty object parses");
        assert!(data.bodies.is_empty());
        assert!(data.solution.is_none());
        assert_eq!(data.display_title(), "Motion");
        let data = MotionUiData::parse(r#"{"title":"x"}"#).expect("partial object parses");
        assert_eq!(data.display_title(), "x");
    }

    #[test]
    fn parses_property_rows_and_maps_them_to_tree_nodes() {
        let data = MotionUiData::parse(
            r#"{
              "bodies": [{"name": "L002", "source": null, "properties": [
                {"path": "body.L002.mass", "label_zh": "质量", "label_en": "Mass",
                 "value": 0.5, "value_type": "float", "unit": "kg",
                 "writable": true, "nxopen_writable": true}
              ]}],
              "solution": {"name": "Solution_1", "properties": [
                {"path": "solution.end_time", "label_zh": "结束时间",
                 "label_en": "End time", "value": 5.0, "value_type": "float",
                 "unit": "s", "writable": false, "nxopen_writable": true}
              ]}
            }"#,
        )
        .expect("properties parse");
        assert_eq!(data.bodies[0].properties[0].path, "body.L002.mass");
        assert!(data.bodies[0].properties[0].writable);
        let nodes = tree_nodes(&data);
        let body = nodes
            .iter()
            .find(|node| node.kind == NodeKind::Body)
            .expect("body node");
        let (name, properties) = node_properties(&data, body).expect("body properties");
        assert_eq!(name, "L002");
        assert_eq!(properties.len(), 1);
        let solution = nodes
            .iter()
            .find(|node| node.kind == NodeKind::Solution)
            .expect("solution node");
        assert_eq!(node_properties(&data, solution).unwrap().1.len(), 1);
    }

    #[test]
    fn null_body_source_is_accepted() {
        let data =
            MotionUiData::parse(r#"{"bodies":[{"name":"L001","source":null},{"name":"L002"}]}"#)
                .expect("null and missing source parse");
        assert_eq!(data.bodies[0].source, None);
        assert_eq!(data.bodies[1].source, None);
    }

    #[test]
    fn tree_order_follows_the_motion_navigator() {
        let data = MotionUiData::parse(SAMPLE).expect("sample parses");
        let nodes = tree_nodes(&data);
        let labels: Vec<&str> = nodes.iter().map(|node| node.label.as_str()).collect();
        assert_eq!(
            labels,
            [
                "motion_1",
                "齿轮泵装配.prt",
                "定制组",
                "运动体",
                "L001",
                "运动副",
                "J001",
                "耦合副",
                "J004",
                "Solution_1"
            ]
        );
        assert_eq!(nodes[0].kind, NodeKind::Scenario);
        assert_eq!(nodes[4].depth, 2);
        assert_eq!(nodes[8].detail.as_deref(), Some("Gear · 1"));
        assert_eq!(nodes[9].kind, NodeKind::Solution);
    }

    #[test]
    fn empty_sections_are_omitted() {
        let data =
            MotionUiData::parse(r#"{"scenario":{"name":"m"},"assembly":"a.prt"}"#).expect("parses");
        let nodes = tree_nodes(&data);
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[1].kind, NodeKind::Assembly);
    }

    #[test]
    fn flex_modes_accept_strings_and_objects() {
        let value = serde_json::json!(["bending"]);
        assert_eq!(flex_mode_label(&value[0]), "bending");
        let value = serde_json::json!({"name": "mode 1", "frequency": 12.5});
        assert_eq!(flex_mode_label(&value), "mode 1 · 12.50 Hz");
        let value = serde_json::json!({"frequency": 3.0});
        assert_eq!(flex_mode_label(&value), "3.00 Hz");
    }

    #[test]
    fn title_falls_back_through_scenario_and_assembly() {
        let data = MotionUiData::parse(r#"{"scenario":{"name":"sc"},"assembly":"a.prt"}"#)
            .expect("parses");
        assert_eq!(data.display_title(), "sc");
        let data = MotionUiData::parse(r#"{"assembly":"a.prt"}"#).expect("parses");
        assert_eq!(data.display_title(), "a.prt");
    }
}
