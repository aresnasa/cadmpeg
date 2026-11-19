// SPDX-License-Identifier: Apache-2.0
//! RecurDyn text model (`-solution_1.rmd`) and message (`-solution_1.msg`)
//! editing.
//!
//! NX Motion exports the solved model as a RecurDyn text file. Each driver is
//! a `MOTION` block whose `FUNCTION` line is a polynomial in `TIME`:
//!
//! ```text
//! MOTION / 1000000
//! , NAME = 'J002.RMotion'
//! , JOINT = 47
//! , ROTATION
//! , DISPLACEMENT
//! , FUNCTION = 0.5*(0)*TIME**2+(3.141592653589792)*TIME+(0)\
//! ```
//!
//! The solver end time and step count live in the paired `.msg` log:
//! `sim/dyn,end=5.000000,step=200`.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};

/// One driver function.
#[derive(Debug, Clone, PartialEq)]
pub struct MotionDriver {
    /// Driver name, for example `J002.RMotion`.
    pub name: String,
    /// Joint id carried by the block.
    pub joint: Option<u32>,
    /// Displacement expression as stored.
    pub function: String,
    /// Quadratic coefficient, when the expression has the standard shape.
    pub acceleration: Option<f64>,
    /// Linear coefficient in radians per second (or model units per second).
    pub velocity: Option<f64>,
    /// Constant term.
    pub initial: Option<f64>,
}

/// Parsed RecurDyn model text.
#[derive(Debug, Clone)]
pub struct MotionModel {
    /// Model text.
    pub text: String,
    /// Driver functions in file order.
    pub drivers: Vec<MotionDriver>,
}

/// Solver range from a `.msg` log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationRange {
    /// End time in seconds.
    pub end_time: f64,
    /// Requested step count.
    pub steps: u32,
}

impl MotionModel {
    /// Parse a `.rmd` model text.
    pub fn parse(path: &Path) -> anyhow::Result<Self> {
        let text = fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
        Ok(Self {
            drivers: parse_drivers(&text),
            text,
        })
    }
}

/// Read the solver range from a `.msg` log.
pub fn read_range(path: &Path) -> anyhow::Result<SimulationRange> {
    let text = fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    parse_range(&text)
}

/// Rewrite the solver range of a `.msg` log.
pub fn rewrite_range(text: &str, range: SimulationRange) -> anyhow::Result<String> {
    let marker = "sim/dyn,end=";
    let Some(start) = text.find(marker) else {
        bail!("no sim/dyn,end= line in the .msg log");
    };
    let value_start = start + marker.len();
    let Some(line_end) = text[value_start..].find('\n').map(|end| value_start + end) else {
        bail!("truncated sim/dyn line");
    };
    let line = &text[value_start..line_end];
    let Some((_, step_part)) = line.split_once(",step=") else {
        bail!("sim/dyn line has no step= field");
    };
    if step_part.trim().parse::<f64>().is_err() {
        bail!("sim/dyn step is not numeric");
    }
    let replacement = format!("{:.6},step={}", range.end_time, range.steps);
    let mut output = String::with_capacity(text.len() + replacement.len());
    output.push_str(&text[..value_start]);
    output.push_str(&replacement);
    output.push_str(&text[line_end..]);
    Ok(output)
}

fn parse_range(text: &str) -> anyhow::Result<SimulationRange> {
    let marker = "sim/dyn,end=";
    let Some(start) = text.find(marker) else {
        bail!("no sim/dyn,end= line in the .msg log");
    };
    let value_start = start + marker.len();
    let Some(line_end) = text[value_start..].find('\n').map(|end| value_start + end) else {
        bail!("truncated sim/dyn line");
    };
    let line = &text[value_start..line_end];
    let Some((end_text, step_text)) = line.split_once(",step=") else {
        bail!("sim/dyn line has no step= field");
    };
    Ok(SimulationRange {
        end_time: end_text.trim().parse::<f64>()?,
        steps: step_text.trim().parse::<f64>()? as u32,
    })
}

fn parse_drivers(text: &str) -> Vec<MotionDriver> {
    let mut drivers = Vec::new();
    let mut current: Option<MotionDriver> = None;
    for line in text.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("MOTION /") || trimmed.starts_with("MOTION/") {
            if let Some(driver) = current.take() {
                drivers.push(driver);
            }
            current = Some(MotionDriver {
                name: String::new(),
                joint: None,
                function: String::new(),
                acceleration: None,
                velocity: None,
                initial: None,
            });
            continue;
        }
        let Some(driver) = current.as_mut() else {
            continue;
        };
        // A MOTION block only continues with its own `, FIELD = value` lines
        // and their `\` continuations; any other line closes the block.
        let continuation = trimmed.starts_with(',') || trimmed.is_empty();
        if !continuation {
            if let Some(driver) = current.take() {
                drivers.push(driver);
            }
            continue;
        }
        if let Some(value) = field_value(line, ", NAME = ") {
            driver.name = value.trim().trim_matches('\'').to_string();
        } else if let Some(value) = field_value(line, ", JOINT = ") {
            driver.joint = value.trim().parse().ok();
        } else if let Some(value) = field_value(line, ", FUNCTION = ") {
            driver.function = value.trim().trim_end_matches('\\').trim().to_string();
            let (acceleration, velocity, initial) = polynomial(&driver.function);
            driver.acceleration = acceleration;
            driver.velocity = velocity;
            driver.initial = initial;
        }
    }
    if let Some(driver) = current.take() {
        drivers.push(driver);
    }
    drivers
        .into_iter()
        .filter(|driver| !driver.name.is_empty())
        .collect()
}

fn field_value<'a>(line: &'a str, marker: &str) -> Option<&'a str> {
    let trimmed = line.trim_start();
    trimmed.strip_prefix(marker)
}

/// Split `0.5*(a)*TIME**2+(b)*TIME+(c)` into its three coefficients.
fn polynomial(function: &str) -> (Option<f64>, Option<f64>, Option<f64>) {
    let Some(rest) = function.strip_prefix("0.5*(") else {
        return (None, None, None);
    };
    let Some((acceleration, rest)) = rest.split_once(")*TIME**2+(") else {
        return (None, None, None);
    };
    let Some((velocity, rest)) = rest.split_once(")*TIME+(") else {
        return (None, None, None);
    };
    let Some(initial) = rest.strip_suffix(')') else {
        return (None, None, None);
    };
    (
        acceleration.trim().parse().ok(),
        velocity.trim().parse().ok(),
        initial.trim().parse().ok(),
    )
}

pub fn rewrite_velocity(text: &str, name: &str, velocity: f64) -> anyhow::Result<String> {
    let mut output = String::with_capacity(text.len() + 16);
    let mut inside = false;
    let mut seen = false;
    let mut rewritten = false;
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim_start();
        if trimmed.starts_with("MOTION /") || trimmed.starts_with("MOTION/") {
            inside = true;
        } else if inside && trimmed.starts_with('!') {
            inside = false;
        }
        if inside {
            if let Some(value) = field_value(line, ", NAME = ") {
                if value.trim().trim_matches('\'') == name {
                    seen = true;
                }
            }
            if seen {
                if let Some(value) = field_value(line, ", FUNCTION = ") {
                    let (acceleration, current, initial) =
                        polynomial(value.trim().trim_end_matches('\\').trim());
                    let (Some(acceleration), Some(_), Some(initial)) =
                        (acceleration, current, initial)
                    else {
                        bail!("driver {name} does not use the standard quadratic FUNCTION shape");
                    };
                    let suffix = if line.trim_end().ends_with('\\') {
                        "\\"
                    } else {
                        ""
                    };
                    let newline = if line.ends_with('\n') { "\n" } else { "" };
                    output.push_str(&format!(
                        ", FUNCTION = 0.5*({acceleration})*TIME**2+({velocity})*TIME+({initial}){suffix}{newline}"
                    ));
                    inside = false;
                    seen = false;
                    rewritten = true;
                    continue;
                }
            }
        }
        output.push_str(line);
    }
    if !rewritten {
        bail!("no driver named {name} in the .rmd model");
    }
    Ok(output)
}

pub fn rewrite_function(text: &str, name: &str, function: &str) -> anyhow::Result<String> {
    let mut output = String::with_capacity(text.len() + function.len());
    let mut inside = false;
    let mut seen = false;
    let mut rewritten = false;
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim_start();
        if trimmed.starts_with("MOTION /") || trimmed.starts_with("MOTION/") {
            inside = true;
        } else if inside && trimmed.starts_with('!') {
            inside = false;
        }
        if inside {
            if let Some(value) = field_value(line, ", NAME = ") {
                if value.trim().trim_matches('\'') == name {
                    seen = true;
                }
            }
            if seen {
                if field_value(line, ", FUNCTION = ").is_some() {
                    let suffix = if line.trim_end().ends_with('\\') {
                        "\\"
                    } else {
                        ""
                    };
                    let newline = if line.ends_with('\n') { "\n" } else { "" };
                    output.push_str(&format!(", FUNCTION = {function}{suffix}{newline}"));
                    inside = false;
                    seen = false;
                    rewritten = true;
                    continue;
                }
            }
        }
        output.push_str(line);
    }
    if !rewritten {
        bail!("no driver named {name} in the .rmd model");
    }
    Ok(output)
}

/// Paired `.msg` path of a `.rmd` file.
#[must_use]
pub fn paired_msg(rmd: &Path) -> PathBuf {
    rmd.with_extension("msg")
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: &str = "MOTION / 1000000\n, NAME = 'J002.RMotion'\n, JOINT = 47\n, ROTATION\n, DISPLACEMENT\n, FUNCTION = 0.5*(0)*TIME**2+(3.141592653589792)*TIME+(0)\\\n\nJOINT / 70\n";

    #[test]
    fn parses_driver_coefficients() {
        let drivers = parse_drivers(MODEL);
        assert_eq!(drivers.len(), 1);
        assert_eq!(drivers[0].name, "J002.RMotion");
        assert_eq!(drivers[0].joint, Some(47));
        assert_eq!(drivers[0].acceleration, Some(0.0));
        assert!((drivers[0].velocity.unwrap() - 3.141592653589792).abs() < 1e-15);
        assert_eq!(drivers[0].initial, Some(0.0));
    }

    #[test]
    fn rewrites_only_the_velocity() {
        let output = rewrite_velocity(MODEL, "J002.RMotion", 6.283185307179584).unwrap();
        assert!(output.contains("(6.283185307179584)*TIME+"));
        assert!(output.contains("0.5*(0)*TIME**2+"));
        assert!(output.contains("JOINT / 70"));
    }

    #[test]
    fn rewrites_the_msg_range() {
        let msg = "info\n sim/dyn,end=5.000000,step=200\nmore";
        let output = rewrite_range(
            msg,
            SimulationRange {
                end_time: 10.0,
                steps: 400,
            },
        )
        .unwrap();
        assert!(output.contains("sim/dyn,end=10.000000,step=400"));
        assert_eq!(
            parse_range(&output).unwrap(),
            SimulationRange {
                end_time: 10.0,
                steps: 400
            }
        );
    }
}
