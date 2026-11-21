// SPDX-License-Identifier: Apache-2.0
//! Simcenter 3D Motion-style window: menu, ribbon, docked tree panels, and a
//! software-rendered 3D viewport.
//!
//! `nxview motion-ui <file.sim|assembly.prt> [--data motion_ui.json] [--root dir]`

use std::path::PathBuf;

use anyhow::{bail, Context};

#[cfg(feature = "viewer")]
use super::scene;

#[cfg(all(windows, feature = "viewer"))]
fn enable_dpi_awareness() {
    use windows_sys::Win32::UI::HiDpi::{
        SetProcessDpiAwarenessContext, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2,
    };
    // Without this, Windows virtualises the 1600x900 window at 200% display
    // scaling and the client area shrinks until the ribbon and dock clip.
    unsafe {
        SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    }
}

mod data;
mod layout;

#[cfg(feature = "viewer")]
mod app;
#[cfg(feature = "viewer")]
mod text;

#[cfg(feature = "viewer")]
pub(crate) use app::run;

pub(crate) const USAGE: &str = "usage: nxview motion-ui <file.sim|assembly.prt> \
[--data <motion_ui.json>] [--root <dir>] [--select <object>]";

/// Pixel surface shared by the chrome painter and the text renderer.
pub(crate) struct Canvas<'a> {
    pub pixels: &'a mut [u32],
    pub width: usize,
    pub height: usize,
}

impl Canvas<'_> {
    fn index(&self, x: i32, y: i32) -> Option<usize> {
        if x < 0 || y < 0 {
            return None;
        }
        let x = usize::try_from(x).ok()?;
        let y = usize::try_from(y).ok()?;
        if x >= self.width || y >= self.height {
            return None;
        }
        Some(y * self.width + x)
    }

    pub(crate) fn put(&mut self, x: i32, y: i32, color: u32) {
        if let Some(index) = self.index(x, y) {
            self.pixels[index] = color;
        }
    }

    pub(crate) fn blend(&mut self, x: i32, y: i32, color: u32, coverage: u8) {
        if coverage == 0 {
            return;
        }
        if let Some(index) = self.index(x, y) {
            let alpha = f32::from(coverage) / 255.0;
            self.pixels[index] = blend_color(color, self.pixels[index], alpha);
        }
    }

    pub(crate) fn fill_rect(&mut self, rect: layout::Rect, color: u32) {
        let x0 = rect.x.max(0);
        let y0 = rect.y.max(0);
        let x1 = rect.right().min(self.width as i32);
        let y1 = rect.bottom().min(self.height as i32);
        if x1 <= x0 || y1 <= y0 {
            return;
        }
        for y in y0..y1 {
            let start = y as usize * self.width + x0 as usize;
            let end = y as usize * self.width + x1 as usize;
            self.pixels[start..end].fill(color);
        }
    }

    pub(crate) fn outline_rect(&mut self, rect: layout::Rect, color: u32) {
        if rect.is_empty() {
            return;
        }
        self.fill_rect(layout::Rect::new(rect.x, rect.y, rect.w, 1), color);
        self.fill_rect(
            layout::Rect::new(rect.x, rect.bottom() - 1, rect.w, 1),
            color,
        );
        self.fill_rect(layout::Rect::new(rect.x, rect.y, 1, rect.h), color);
        self.fill_rect(
            layout::Rect::new(rect.right() - 1, rect.y, 1, rect.h),
            color,
        );
    }

    pub(crate) fn line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: u32) {
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut error = dx + dy;
        let mut x = x0;
        let mut y = y0;
        loop {
            self.put(x, y, color);
            if x == x1 && y == y1 {
                break;
            }
            let doubled = 2 * error;
            if doubled >= dy {
                error += dy;
                x += sx;
            }
            if doubled <= dx {
                error += dx;
                y += sy;
            }
        }
    }

    pub(crate) fn blit(&mut self, source: &[u32], width: usize, height: usize, x: i32, y: i32) {
        if width == 0 || height == 0 || source.len() < width * height {
            return;
        }
        let x0 = x.max(0);
        let y0 = y.max(0);
        let x1 = (x + width as i32).min(self.width as i32);
        let y1 = (y + height as i32).min(self.height as i32);
        if x1 <= x0 || y1 <= y0 {
            return;
        }
        let copy_width = (x1 - x0) as usize;
        for row in y0..y1 {
            let source_row = (row - y) as usize;
            let source_x = (x0 - x) as usize;
            let source_start = source_row * width + source_x;
            let target_start = row as usize * self.width + x0 as usize;
            self.pixels[target_start..target_start + copy_width]
                .copy_from_slice(&source[source_start..source_start + copy_width]);
        }
    }
}

pub(crate) fn blend_color(top: u32, bottom: u32, alpha: f32) -> u32 {
    let alpha = alpha.clamp(0.0, 1.0);
    let blend = |shift: u32| {
        let top_channel = (top >> shift) & 0xff;
        let bottom_channel = (bottom >> shift) & 0xff;
        let value = bottom_channel as f32 * (1.0 - alpha) + top_channel as f32 * alpha;
        (value.clamp(0.0, 255.0) as u32) << shift
    };
    blend(16) | blend(8) | blend(0)
}

/// Parse arguments, load the scene, and start the Motion window.
pub(crate) fn command(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut path = None;
    let mut data_path = None;
    let mut root = None;
    let mut selection = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--data" => {
                data_path = Some(PathBuf::from(
                    args.next().context("usage: --data <motion_ui.json>")?,
                ));
            }
            "--root" => {
                root = Some(PathBuf::from(
                    args.next().context("usage: --root <directory>")?,
                ));
            }
            "--select" => {
                selection = Some(args.next().context("usage: --select <object>")?);
            }
            other => {
                if path.replace(PathBuf::from(other)).is_some() {
                    bail!(USAGE);
                }
            }
        }
    }
    let path = path.context(USAGE)?;

    #[cfg(not(feature = "viewer"))]
    {
        let _ = (path, data_path, root);
        bail!("viewer not compiled: build with --features viewer");
    }

    #[cfg(feature = "viewer")]
    {
        #[cfg(windows)]
        enable_dpi_awareness();
        let mut loaded = scene::load(&path, root.as_deref(), super::chord(0.5))?;
        scene::print_summary(&loaded);
        let data = data::load_or_synthesize(data_path.as_deref(), &loaded)?;
        run(&mut loaded, &data, selection.as_deref())?;
        Ok(())
    }
}

#[cfg(test)]
mod canvas_tests {
    use super::layout::Rect;
    use super::*;

    #[test]
    fn fill_rect_clips_to_the_surface() {
        let mut pixels = vec![0_u32; 4 * 4];
        {
            let mut canvas = Canvas {
                pixels: &mut pixels,
                width: 4,
                height: 4,
            };
            canvas.fill_rect(Rect::new(-2, -2, 4, 4), 0x00112233);
            canvas.fill_rect(Rect::new(3, 3, 10, 10), 0x00445566);
        }
        assert_eq!(pixels[0], 0x00112233);
        assert_eq!(pixels[1], 0x00112233);
        assert_eq!(pixels[2], 0);
        assert_eq!(pixels[15], 0x00445566);
        assert_eq!(pixels[5], 0x00112233);
    }

    #[test]
    fn blit_copies_only_the_intersection() {
        let source = vec![1_u32, 2, 3, 4];
        let mut pixels = vec![0_u32; 4 * 4];
        {
            let mut canvas = Canvas {
                pixels: &mut pixels,
                width: 4,
                height: 4,
            };
            canvas.blit(&source, 2, 2, 3, 3);
        }
        assert_eq!(pixels[15], 1);
        assert_eq!(pixels[3 * 4 + 3], 1);
        assert_eq!(pixels[3 * 4 + 2], 0);
    }
}
