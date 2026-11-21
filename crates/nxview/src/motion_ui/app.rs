// SPDX-License-Identifier: Apache-2.0
//! Window, input handling, and painting for `nxview motion-ui`.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use glam::Vec3;
use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Window, WindowOptions};

use super::data::{
    flex_mode_label, node_properties, tree_nodes, MotionUiData, NodeKind, Property, TreeNode,
};
use super::layout::{
    file_menu_rect, file_menu_rows, hit, max_tree_scroll, menu_items, preferences_rect,
    preferences_rows, ribbon_groups, tree_rows, Layout, PanelGeometry, PanelId, Rect,
    FILE_MENU_ITEMS, FILE_MENU_ITEMS_EN, PREFERENCES_COMMANDS, PREFERENCES_ITEMS,
    PREFERENCES_ITEMS_EN, TREE_ROW_HEIGHT,
};
use super::text::TextRenderer;
use super::{blend_color, Canvas};
use crate::scene::Scene;
use crate::CameraState;

const C_BG: u32 = 0x00151a20;
const C_MENU: u32 = 0x00202932;
const C_MENU_HOT: u32 = 0x00303d49;
const C_ACCENT: u32 = 0x00e39a3b;
const C_ACCENT_TEXT: u32 = 0x00ffc46b;
const C_RIBBON: u32 = 0x001c242c;
const C_GROUP: u32 = 0x00242e38;
const C_GROUP_EDGE: u32 = 0x00303c47;
const C_TILE: u32 = 0x002a3540;
const C_TILE_HOT: u32 = 0x00384755;
const C_TILE_EDGE: u32 = 0x0036424d;
const C_DOCK: u32 = 0x001d252d;
const C_PANEL: u32 = 0x001b232a;
const C_HEADER: u32 = 0x002a343e;
const C_LINE: u32 = 0x00323c46;
const C_TEXT: u32 = 0x00e6edf3;
const C_DIM: u32 = 0x0097a3ae;
const C_FAINT: u32 = 0x006b7681;
const C_SELECT: u32 = 0x00405a70;
const C_STATUS: u32 = 0x00252e37;
const C_MENU_BG: u32 = 0x00242e38;
const C_MENU_EDGE: u32 = 0x00404c58;
const C_SHADOW: u32 = 0x000c1014;
const C_VIEW_BORDER: u32 = 0x004a5864;

/// minifb keeps the size passed to `Window::new` even when the Win32 window
/// is clamped by DPI virtualisation or the desktop work area. Use the real
/// client rectangle so the layout never renders into a clipped buffer.
#[cfg(windows)]
fn window_client_size(window: &Window) -> (usize, usize) {
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};
    use windows_sys::Win32::Foundation::RECT;
    use windows_sys::Win32::UI::WindowsAndMessaging::GetClientRect;

    if let Ok(handle) = window.window_handle() {
        if let RawWindowHandle::Win32(raw) = handle.as_raw() {
            let mut rect = RECT::default();
            let ok = unsafe { GetClientRect(raw.hwnd.get() as _, &mut rect) };
            if ok != 0 && rect.right > 0 && rect.bottom > 0 {
                return (rect.right as usize, rect.bottom as usize);
            }
        }
    }
    window.get_size()
}

#[cfg(not(windows))]
fn window_client_size(window: &Window) -> (usize, usize) {
    window.get_size()
}

const NODE_COLORS: [u32; 7] = [
    0x00e39a3b, // scenario
    0x006fa8d8, // assembly
    0x0097a3ae, // group
    0x007fc08a, // body
    0x00e09a5a, // joint
    0x00b48ad8, // coupler
    0x005ab7b0, // solution
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Focus {
    Navigator,
    Viewport,
    Ribbon,
}

struct UiState {
    collapsed: [bool; 4],
    selected_node: Option<usize>,
    selected_part: Option<usize>,
    selected_channel: Option<usize>,
    tree_scroll: i32,
    tree_scroll_remainder: f32,
    focus: Focus,
    last_tree_click: Option<(Instant, usize)>,
    file_menu_open: bool,
    prefs_open: bool,
    properties_open: bool,
    help_open: bool,
    active_menu: usize,
    hovered_menu: Option<usize>,
    hovered_tile: Option<(usize, usize)>,
    hovered_tree: Option<usize>,
    hovered_file: Option<usize>,
    hovered_pref: Option<usize>,
    message: String,
    quit: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            collapsed: [false; 4],
            selected_node: None,
            selected_part: None,
            selected_channel: None,
            tree_scroll: 0,
            tree_scroll_remainder: 0.0,
            focus: Focus::Viewport,
            last_tree_click: None,
            file_menu_open: false,
            prefs_open: false,
            properties_open: true,
            help_open: false,
            active_menu: 1,
            hovered_menu: None,
            hovered_tile: None,
            hovered_tree: None,
            hovered_file: None,
            hovered_pref: None,
            message: String::from("左键旋转 · 右键平移 · 滚轮缩放 · Space 播放"),
            quit: false,
        }
    }
}

impl UiState {
    fn close_menus(&mut self) {
        self.file_menu_open = false;
        self.prefs_open = false;
    }

    fn selected_node_label<'a>(&self, nodes: &'a [TreeNode]) -> Option<&'a str> {
        self.selected_node
            .and_then(|index| nodes.get(index))
            .map(|node| node.label.as_str())
    }

    fn activate_file_item(&mut self, row: usize) -> bool {
        match row {
            4 => {
                self.prefs_open = true;
                false
            }
            14 => true,
            _ => {
                self.message = format!(
                    "{}  [命令未恢复 / command not recovered]：占位功能（第一版）",
                    FILE_MENU_ITEMS[row]
                );
                self.close_menus();
                false
            }
        }
    }

    /// Handle a left press outside the 3D viewport. Returns true to quit.
    #[allow(clippy::too_many_arguments)]
    fn handle_click(
        &mut self,
        layout: Layout,
        panels: &[PanelGeometry; 4],
        nodes: &[TreeNode],
        data: &MotionUiData,
        scene: &Scene,
        x: i32,
        y: i32,
    ) -> bool {
        if layout.menu.contains(x, y) {
            if let Some(item) = hit(
                menu_items(layout.menu)
                    .into_iter()
                    .map(|item| ((item.index, item.label), item.rect)),
                x,
                y,
            ) {
                let was_file = self.file_menu_open;
                self.close_menus();
                self.active_menu = item.0;
                if item.0 == 0 {
                    self.file_menu_open = !was_file;
                } else {
                    self.message = format!("{}：占位功能（第一版）", item.1);
                }
            }
            return false;
        }

        if self.file_menu_open {
            let file = file_menu_rect(layout.menu);
            let prefs = preferences_rect(file);
            if let Some(row) = hit(file_menu_rows(file).into_iter().enumerate(), x, y) {
                return self.activate_file_item(row);
            }
            if self.prefs_open {
                if let Some(row) = hit(preferences_rows(prefs).into_iter().enumerate(), x, y) {
                    let command =
                        PREFERENCES_COMMANDS[row].unwrap_or("命令未恢复 / command not recovered");
                    self.message = format!(
                        "首选项 / {}  [{command}]：占位功能（第一版未执行）",
                        PREFERENCES_ITEMS[row]
                    );
                    self.close_menus();
                    return false;
                }
            }
            let inside = file.contains(x, y) || (self.prefs_open && prefs.contains(x, y));
            if !inside {
                self.close_menus();
            } else {
                return false;
            }
        }

        for panel in panels {
            if panel.header.contains(x, y) {
                let index = panel.id.index();
                self.collapsed[index] = !self.collapsed[index];
                self.message = format!(
                    "{}：{}",
                    panel.id.title(),
                    if self.collapsed[index] {
                        "已折叠"
                    } else {
                        "已展开"
                    }
                );
                return false;
            }
        }

        let navigator = panels[PanelId::Navigator.index()];
        if navigator.body.contains(x, y) {
            let rows = tree_rows(navigator.body, nodes, self.tree_scroll);
            if let Some(node) = hit(rows.iter().map(|row| (row.node, row.rect)), x, y) {
                let now = Instant::now();
                let double_click = self.last_tree_click.is_some_and(|(then, previous)| {
                    previous == node && now.duration_since(then) <= Duration::from_millis(450)
                });
                self.last_tree_click = Some((now, node));
                self.selected_node = Some(node);
                self.focus = Focus::Navigator;
                if double_click && node_properties(data, &nodes[node]).is_some() {
                    self.properties_open = true;
                    self.message = format!("属性 {}", nodes[node].label);
                }
                if nodes[node].kind == NodeKind::Body {
                    self.selected_part = scene
                        .parts
                        .iter()
                        .position(|part| part_matches(part, &nodes[node]));
                    if let Some(index) = self.selected_part {
                        self.message = format!("选择 {}", scene.parts[index].label);
                    }
                } else {
                    self.selected_part = None;
                }
                return false;
            }
        }

        let xy = panels[PanelId::XyResult.index()];
        let rows = channel_rows(xy.body, data.channels.len());
        if let Some(channel) = hit(
            rows.iter().enumerate().map(|(index, rect)| (index, *rect)),
            x,
            y,
        ) {
            self.selected_channel = Some(channel);
            self.message = format!("结果通道 {}", data.channels[channel].path);
            return false;
        }

        for (group_index, group) in ribbon_groups(layout.ribbon).iter().enumerate() {
            for (tile_index, tile) in group.tiles.iter().enumerate() {
                if tile.rect.intersect(layout.ribbon).contains(x, y) {
                    let command = if tile.command.is_empty() {
                        "命令未恢复 / command not recovered"
                    } else {
                        tile.command
                    };
                    self.message = format!(
                        "{} / {}  [{command}]：占位功能（第一版未执行）",
                        group.name, tile.label
                    );
                    self.hovered_tile = Some((group_index, tile_index));
                    self.focus = Focus::Ribbon;
                    return false;
                }
            }
        }

        self.close_menus();
        false
    }

    fn update_hover(
        &mut self,
        layout: Layout,
        panels: &[PanelGeometry; 4],
        nodes: &[TreeNode],
        mouse: (i32, i32),
    ) {
        self.hovered_menu = hit(
            menu_items(layout.menu)
                .into_iter()
                .map(|item| (item.index, item.rect)),
            mouse.0,
            mouse.1,
        );
        self.hovered_tree = {
            let navigator = panels[PanelId::Navigator.index()];
            hit(
                tree_rows(navigator.body, nodes, self.tree_scroll)
                    .iter()
                    .map(|row| (row.node, row.rect)),
                mouse.0,
                mouse.1,
            )
        };
        self.hovered_tile = None;
        for (group_index, group) in ribbon_groups(layout.ribbon).iter().enumerate() {
            for (tile_index, tile) in group.tiles.iter().enumerate() {
                if tile
                    .rect
                    .intersect(layout.ribbon)
                    .contains(mouse.0, mouse.1)
                {
                    self.hovered_tile = Some((group_index, tile_index));
                }
            }
        }

        self.hovered_file = None;
        self.hovered_pref = None;
        if self.file_menu_open {
            let file = file_menu_rect(layout.menu);
            let prefs = preferences_rect(file);
            self.hovered_file = hit(
                file_menu_rows(file).into_iter().enumerate(),
                mouse.0,
                mouse.1,
            );
            if self.hovered_file == Some(4) {
                self.prefs_open = true;
            }
            if self.prefs_open {
                self.hovered_pref = hit(
                    preferences_rows(prefs).into_iter().enumerate(),
                    mouse.0,
                    mouse.1,
                );
                if !file.contains(mouse.0, mouse.1) && self.hovered_pref.is_none() {
                    self.prefs_open = false;
                }
            }
        }
    }
}

fn part_matches(part: &crate::scene::ScenePart, node: &TreeNode) -> bool {
    if part.label == node.label {
        return true;
    }
    let source = node.detail.as_deref().unwrap_or("");
    part.label == file_stem(source) || part.label == file_stem(&node.label)
}

fn file_stem(name: &str) -> &str {
    name.rsplit_once('.').map_or(name, |(stem, extension)| {
        if extension.eq_ignore_ascii_case("prt") {
            stem
        } else {
            name
        }
    })
}

fn channel_rows(body: Rect, channels: usize) -> Vec<Rect> {
    let reserved = 48;
    let available = (body.h - reserved).max(0);
    let rows = (available / 18).max(0) as usize;
    (0..channels.min(rows))
        .map(|index| Rect::new(body.x, body.y + index as i32 * 18, body.w, 18))
        .collect()
}

pub fn run(
    scene: &mut Scene,
    data: &MotionUiData,
    initial_selection: Option<&str>,
) -> anyhow::Result<()> {
    let (target, extent) = crate::scene_camera(scene)?;
    let initial = CameraState {
        distance: extent * 1.9,
        ..CameraState::new(target, extent)
    };
    let mut camera = initial;
    let nodes = tree_nodes(data);
    let mut ui = UiState::default();
    if let Some(label) = initial_selection {
        if let Some((index, node)) = nodes
            .iter()
            .enumerate()
            .find(|(_, node)| node.label == label)
        {
            ui.selected_node = Some(index);
            if node.kind == NodeKind::Body {
                ui.selected_part = scene.parts.iter().position(|part| part.label == label);
            }
            ui.message = format!("选择 {label}");
        }
    }
    let mut text = TextRenderer::new();
    if !text.has_cjk_font() {
        ui.message = String::from("Left-drag orbit · Right-drag pan · Wheel zoom · Space play");
    }
    println!(
        "motion-ui: {} | {} node(s) | font {}",
        data.display_title(),
        nodes.len(),
        text.source()
    );
    println!("controls: Space play/pause, Left/Right step, Home frame 0");
    println!("          R reset, F fit, S screenshot, P properties, Esc close");
    println!("          left-drag orbit, right/middle-drag pan, wheel zoom");

    let mut window = Window::new(
        &format!("NX Motion — {}", data.display_title()),
        1600,
        900,
        WindowOptions {
            resize: true,
            ..Default::default()
        },
    )?;
    window.set_target_fps(60);

    let frames = scene.frame_count().max(1);
    let duration = scene
        .animation
        .as_ref()
        .map_or(5.0, |animation| animation.time(frames.saturating_sub(1)))
        .max(1e-6);
    let mut play_position = scene.frame as f64;
    let mut last_clock = Instant::now();
    let mut last_title = String::new();

    let mut frame_buffer: Vec<u32> = Vec::new();
    let mut viewport_buffer: Vec<u32> = Vec::new();
    let mut zbuf: Vec<f32> = Vec::new();
    let mut screen: Vec<Vec3> = Vec::new();
    let mut visible: Vec<bool> = Vec::new();
    let mut part_ids: Vec<u32> = Vec::new();
    let mut viewport_size = (0_usize, 0_usize);
    let mut viewport_dirty = true;
    let mut prev_left = false;
    let mut press: Option<(i32, i32, bool)> = None;
    let mut drag_distance = 0_i32;
    let mut last_orbit: Option<(f32, f32)> = None;
    let mut last_pan: Option<(i32, i32)> = None;
    let mut screenshot = 0_usize;
    let mut screenshot_requested = false;

    while window.is_open() && !ui.quit {
        let (width, height) = window_client_size(&window);
        if width == 0 || height == 0 {
            continue;
        }
        if frame_buffer.len() != width * height {
            frame_buffer = vec![C_BG; width * height];
            viewport_dirty = true;
        }
        let layout = Layout::new(width, height);
        let panels = layout.panels(ui.collapsed);
        let mouse = window
            .get_mouse_pos(MouseMode::Discard)
            .map_or((-1, -1), |(x, y)| (x as i32, y as i32));
        let properties_panel = if ui.properties_open {
            ui.selected_node
                .and_then(|index| nodes.get(index))
                .and_then(|node| node_properties(data, node))
                .map(|(_, properties)| properties_panel_rect(layout.viewport, properties.len()))
        } else {
            None
        };
        let mouse_in_viewport = layout.viewport.contains(mouse.0, mouse.1)
            && !properties_panel.is_some_and(|rect| rect.contains(mouse.0, mouse.1));

        if window.is_key_pressed(Key::Escape, KeyRepeat::No) {
            if ui.help_open {
                ui.help_open = false;
            } else if ui.file_menu_open {
                ui.close_menus();
            } else if ui.properties_open && ui.selected_node.is_some() {
                ui.properties_open = false;
                ui.message = String::from("属性面板已关闭");
            } else {
                ui.quit = true;
            }
        }
        if window.is_key_pressed(Key::F1, KeyRepeat::No) {
            ui.help_open = !ui.help_open;
            ui.message = if ui.help_open {
                String::from("帮助 / Help")
            } else {
                String::from("帮助已关闭")
            };
        }
        if window.is_key_pressed(Key::P, KeyRepeat::No) {
            ui.properties_open = !ui.properties_open;
            ui.message = if ui.properties_open {
                String::from("属性面板已打开")
            } else {
                String::from("属性面板已关闭")
            };
        }
        if window.is_key_pressed(Key::Space, KeyRepeat::No) {
            scene.playing = !scene.playing;
            play_position = scene.frame as f64;
            last_clock = Instant::now();
            ui.message = if scene.playing {
                String::from("播放")
            } else {
                String::from("暂停")
            };
        }
        if window.is_key_pressed(Key::Tab, KeyRepeat::No) {
            ui.focus = next_focus(ui.focus);
            ui.message = format!("焦点：{}", focus_label(ui.focus));
        }
        if window.is_key_pressed(Key::Enter, KeyRepeat::No)
            && ui.focus == Focus::Navigator
            && ui.selected_node.is_some()
        {
            ui.properties_open = !ui.properties_open;
            ui.message = if ui.properties_open {
                String::from("属性面板已打开")
            } else {
                String::from("属性面板已关闭")
            };
        }
        if ui.focus == Focus::Navigator {
            let delta = if window.is_key_pressed(Key::Down, KeyRepeat::No) {
                1
            } else if window.is_key_pressed(Key::Up, KeyRepeat::No) {
                -1
            } else {
                0
            };
            if delta != 0 {
                if let Some(next) = next_tree_index(ui.selected_node, delta, nodes.len()) {
                    ui.selected_node = Some(next);
                    if nodes[next].kind == NodeKind::Body {
                        ui.selected_part = scene
                            .parts
                            .iter()
                            .position(|part| part_matches(part, &nodes[next]));
                    } else {
                        ui.selected_part = None;
                    }
                    let navigator = panels[PanelId::Navigator.index()];
                    let row_y = next as i32 * TREE_ROW_HEIGHT;
                    let max = max_tree_scroll(navigator.body, nodes.len());
                    if row_y < ui.tree_scroll {
                        ui.tree_scroll = row_y;
                    } else if row_y + TREE_ROW_HEIGHT > ui.tree_scroll + navigator.body.h {
                        ui.tree_scroll = (row_y + TREE_ROW_HEIGHT - navigator.body.h).clamp(0, max);
                    }
                    ui.message = format!("选择 {}", nodes[next].label);
                }
            }
        }
        let viewport_focus = ui.focus == Focus::Viewport;
        let mut step = 0_i64;
        if viewport_focus
            && (window.is_key_pressed(Key::Right, KeyRepeat::No)
                || window.is_key_pressed(Key::Down, KeyRepeat::No))
        {
            step = 1;
        }
        if viewport_focus
            && (window.is_key_pressed(Key::Left, KeyRepeat::No)
                || window.is_key_pressed(Key::Up, KeyRepeat::No))
        {
            step = -1;
        }
        if viewport_focus && window.is_key_pressed(Key::PageUp, KeyRepeat::No) {
            step = 10;
        }
        if viewport_focus && window.is_key_pressed(Key::PageDown, KeyRepeat::No) {
            step = -10;
        }
        if viewport_focus && window.is_key_pressed(Key::Home, KeyRepeat::No) {
            scene.playing = false;
            scene.set_frame(0);
            play_position = 0.0;
            viewport_dirty = true;
        }
        if step != 0 {
            scene.playing = false;
            let next = (scene.frame as i64 + step).clamp(0, frames as i64 - 1) as usize;
            scene.set_frame(next);
            play_position = next as f64;
            viewport_dirty = true;
            ui.message = format!("帧 {}/{}", next + 1, frames);
        }
        if window.is_key_pressed(Key::R, KeyRepeat::No) {
            camera = initial;
            viewport_dirty = true;
            ui.message = String::from("相机已重置");
        }
        if window.is_key_pressed(Key::F, KeyRepeat::No) {
            camera.target = target;
            camera.distance = extent * 1.9;
            viewport_dirty = true;
            ui.message = String::from("相机已适配");
        }
        if window.is_key_pressed(Key::S, KeyRepeat::No) {
            screenshot_requested = true;
        }
        if window.is_key_pressed(Key::Equal, KeyRepeat::No)
            || window.is_key_pressed(Key::NumPadPlus, KeyRepeat::No)
        {
            scene.speed = (scene.speed * 1.25).min(16.0);
            ui.message = format!("速度 {:.2}x", scene.speed);
        }
        if window.is_key_pressed(Key::Minus, KeyRepeat::No)
            || window.is_key_pressed(Key::NumPadMinus, KeyRepeat::No)
        {
            scene.speed = (scene.speed / 1.25).max(0.05);
            ui.message = format!("速度 {:.2}x", scene.speed);
        }

        ui.update_hover(layout, &panels, &nodes, mouse);

        let left_down = window.get_mouse_down(MouseButton::Left);
        let left_pressed = left_down && !prev_left;
        let left_released = !left_down && prev_left;
        prev_left = left_down;

        if left_pressed {
            let over_file_popup = ui.file_menu_open && {
                let file = file_menu_rect(layout.menu);
                file.contains(mouse.0, mouse.1)
                    || (ui.prefs_open && preferences_rect(file).contains(mouse.0, mouse.1))
            };
            if mouse_in_viewport && !over_file_popup {
                ui.close_menus();
                ui.focus = Focus::Viewport;
                press = Some((mouse.0, mouse.1, true));
                drag_distance = 0;
                last_orbit = Some((mouse.0 as f32, mouse.1 as f32));
            } else {
                press = Some((mouse.0, mouse.1, false));
                let part_before = ui.selected_part;
                if ui.handle_click(layout, &panels, &nodes, data, scene, mouse.0, mouse.1) {
                    ui.quit = true;
                }
                if ui.selected_part != part_before {
                    viewport_dirty = true;
                }
            }
        }

        if left_down {
            if let Some((_, _, true)) = press {
                if mouse_in_viewport {
                    if let Some((last_x, last_y)) = last_orbit {
                        let dx = mouse.0 as f32 - last_x;
                        let dy = mouse.1 as f32 - last_y;
                        drag_distance += dx.abs() as i32 + dy.abs() as i32;
                        if drag_distance > 2 {
                            camera.orbit(dx, dy);
                            viewport_dirty = true;
                        }
                    }
                    last_orbit = Some((mouse.0 as f32, mouse.1 as f32));
                } else {
                    last_orbit = None;
                }
            }
        }

        if left_released {
            if let Some((press_x, press_y, true)) = press {
                if drag_distance <= 3
                    && (mouse.0 - press_x).abs() + (mouse.1 - press_y).abs() <= 3
                    && mouse_in_viewport
                {
                    let (viewport_width, viewport_height) = viewport_size;
                    let column = (mouse.0 - layout.viewport.x) as usize;
                    let row = (mouse.1 - layout.viewport.y) as usize;
                    if column < viewport_width
                        && row < viewport_height
                        && part_ids.len() == viewport_width * viewport_height
                    {
                        let id = part_ids[row * viewport_width + column];
                        let selected = if id == 0 { None } else { Some(id as usize - 1) };
                        if selected != ui.selected_part {
                            ui.selected_part = selected;
                            viewport_dirty = true;
                        }
                        ui.selected_node = None;
                        ui.message = selected.map_or_else(
                            || String::from("已清除选择"),
                            |index| format!("选择 {}", scene.parts[index].label),
                        );
                    }
                }
            }
            press = None;
            last_orbit = None;
        }

        let pan_down =
            window.get_mouse_down(MouseButton::Right) || window.get_mouse_down(MouseButton::Middle);
        if pan_down && mouse_in_viewport && layout.viewport.h > 0 {
            if let Some((last_x, last_y)) = last_pan {
                camera.pan(
                    (mouse.0 - last_x) as f32,
                    (mouse.1 - last_y) as f32,
                    layout.viewport.h as usize,
                );
                viewport_dirty = true;
            }
            last_pan = Some(mouse);
        } else {
            last_pan = None;
        }

        if let Some((_, scroll)) = window.get_scroll_wheel() {
            if mouse_in_viewport {
                camera.zoom(scroll, extent);
                viewport_dirty = true;
            } else {
                let navigator = panels[PanelId::Navigator.index()];
                if navigator.body.contains(mouse.0, mouse.1) {
                    // One wheel notch is three tree rows; keep fractional
                    // touchpad deltas so small movements do not jump.
                    let rows = wheel_tree_rows(scroll);
                    ui.tree_scroll_remainder += rows;
                    let whole = ui.tree_scroll_remainder.trunc();
                    if whole != 0.0 {
                        let max = max_tree_scroll(navigator.body, nodes.len());
                        ui.tree_scroll = (ui.tree_scroll - whole as i32).clamp(0, max);
                        ui.tree_scroll_remainder -= whole;
                    }
                }
            }
        }

        if scene.playing && frames > 1 {
            let elapsed = last_clock.elapsed().as_secs_f64();
            last_clock = Instant::now();
            play_position += elapsed * scene.speed * (frames - 1) as f64 / duration;
            if play_position >= frames as f64 {
                play_position %= frames as f64;
            }
            let next = play_position as usize;
            if next != scene.frame {
                scene.set_frame(next);
                viewport_dirty = true;
            }
        } else {
            last_clock = Instant::now();
        }

        let viewport_width = layout.viewport.w.max(0) as usize;
        let viewport_height = layout.viewport.h.max(0) as usize;
        if viewport_width > 0 && viewport_height > 0 {
            if viewport_size != (viewport_width, viewport_height) {
                viewport_size = (viewport_width, viewport_height);
                viewport_buffer = vec![0; viewport_width * viewport_height];
                zbuf = vec![f32::INFINITY; viewport_width * viewport_height];
                screen.clear();
                visible.clear();
                part_ids.clear();
                viewport_dirty = true;
            }
            if viewport_dirty {
                crate::render_scene(
                    scene,
                    true,
                    viewport_width,
                    viewport_height,
                    camera.target,
                    camera.yaw,
                    camera.pitch,
                    camera.distance,
                    extent,
                    &mut viewport_buffer,
                    &mut zbuf,
                    &mut screen,
                    &mut visible,
                    &mut part_ids,
                    ui.selected_part,
                );
                viewport_dirty = false;
            }
        } else if viewport_size != (0, 0) {
            viewport_size = (0, 0);
            viewport_buffer.clear();
            part_ids.clear();
        }

        let selected_label = ui
            .selected_node_label(&nodes)
            .map(str::to_string)
            .or_else(|| {
                ui.selected_part
                    .and_then(|index| scene.parts.get(index))
                    .map(|part| part.label.clone())
            })
            .unwrap_or_else(|| String::from("无"));

        {
            let mut canvas = Canvas {
                pixels: &mut frame_buffer,
                width,
                height,
            };
            canvas.fill_rect(Rect::new(0, 0, width as i32, height as i32), C_BG);
            if viewport_size.0 > 0 && viewport_size.1 > 0 {
                canvas.blit(
                    &viewport_buffer,
                    viewport_size.0,
                    viewport_size.1,
                    layout.viewport.x,
                    layout.viewport.y,
                );
            }
            let painter = Painter {
                layout,
                panels,
                data,
                nodes: &nodes,
                ui: &ui,
                scene,
                camera,
                mouse,
                frames,
                selected_label: &selected_label,
                chinese: text.has_cjk_font(),
            };
            painter.paint(&mut canvas, &mut text);
        }

        if screenshot_requested {
            let path = PathBuf::from(format!("nxview-motion-ui-{screenshot}.ppm"));
            crate::write_ppm(&path, &frame_buffer, width, height)?;
            println!("wrote {}", path.display());
            ui.message = format!("截图 {}", path.display());
            screenshot += 1;
            screenshot_requested = false;
        }

        let title = format!(
            "NX Motion — {} — {}/{} [{}]",
            data.display_title(),
            scene.frame + 1,
            frames,
            if scene.playing { "play" } else { "pause" }
        );
        if title != last_title {
            window.set_title(&title);
            last_title = title;
        }
        window.update_with_buffer(&frame_buffer, width, height)?;
    }
    Ok(())
}

fn focus_label(focus: Focus) -> &'static str {
    match focus {
        Focus::Navigator => "Motion 导航器",
        Focus::Viewport => "3D 视口",
        Focus::Ribbon => "Ribbon",
    }
}

fn next_focus(focus: Focus) -> Focus {
    match focus {
        Focus::Navigator => Focus::Viewport,
        Focus::Viewport => Focus::Ribbon,
        Focus::Ribbon => Focus::Navigator,
    }
}

fn wheel_tree_rows(scroll: f32) -> f32 {
    (scroll / crate::WHEEL_NOTCH * 3.0).clamp(-9.0, 9.0)
}

fn next_tree_index(current: Option<usize>, delta: i32, len: usize) -> Option<usize> {
    if len == 0 {
        return None;
    }
    let next = current.map_or(0, |index| {
        (index as i32 + delta).clamp(0, len as i32 - 1) as usize
    });
    Some(next)
}

fn properties_panel_rect(viewport: Rect, property_count: usize) -> Rect {
    let width = 340.min((viewport.w - 28).max(220));
    let height = (52 + property_count as i32 * 22 + 26).min((viewport.h - 28).max(120));
    Rect::new(
        viewport.right() - width - 14,
        viewport.y + 14,
        width,
        height,
    )
}

fn format_property_value(property: &Property) -> String {
    let mut text = match &property.value {
        serde_json::Value::Null => String::from("—"),
        serde_json::Value::Bool(value) => value.to_string(),
        serde_json::Value::Number(value) => {
            if let Some(integer) = value.as_i64() {
                integer.to_string()
            } else if let Some(integer) = value.as_u64() {
                integer.to_string()
            } else if let Some(float) = value.as_f64() {
                let mut text = format!("{float:.6}");
                if text.contains('.') {
                    text = text.trim_end_matches('0').trim_end_matches('.').to_string();
                }
                text
            } else {
                value.to_string()
            }
        }
        serde_json::Value::String(value) => value.clone(),
        serde_json::Value::Array(values) => values
            .iter()
            .map(|value| value.to_string())
            .collect::<Vec<_>>()
            .join(", "),
        serde_json::Value::Object(_) => String::from("{…}"),
    };
    if let Some(unit) = property.unit.as_deref().filter(|unit| !unit.is_empty()) {
        if !text.is_empty() && text != "—" {
            text.push(' ');
            text.push_str(unit);
        }
    }
    text
}

struct Painter<'a> {
    layout: Layout,
    panels: [PanelGeometry; 4],
    data: &'a MotionUiData,
    nodes: &'a [TreeNode],
    ui: &'a UiState,
    scene: &'a Scene,
    camera: CameraState,
    mouse: (i32, i32),
    frames: usize,
    selected_label: &'a str,
    chinese: bool,
}

impl Painter<'_> {
    fn label<'b>(&self, zh: &'b str, en: &'b str) -> &'b str {
        if self.chinese {
            zh
        } else {
            en
        }
    }

    fn paint(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        canvas.fill_rect(self.layout.ribbon, C_RIBBON);
        canvas.fill_rect(self.layout.dock, C_DOCK);
        self.paint_menu(canvas, text);
        self.paint_ribbon(canvas, text);
        self.paint_dock(canvas, text);
        self.paint_status(canvas, text);
        self.paint_viewport(canvas, text);
        let focus_rect = match self.ui.focus {
            Focus::Navigator => self.panels[PanelId::Navigator.index()].body,
            Focus::Viewport => self.layout.viewport,
            Focus::Ribbon => self.layout.ribbon,
        };
        if !focus_rect.is_empty() {
            canvas.outline_rect(focus_rect.inset(1, 1, 1, 1), C_ACCENT);
        }
        self.paint_properties(canvas, text);
        if self.ui.file_menu_open {
            self.paint_file_menu(canvas, text);
        }
        self.paint_help(canvas, text);
    }

    fn paint_menu(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        let menu = self.layout.menu;
        canvas.fill_rect(menu, C_MENU);
        canvas.fill_rect(Rect::new(menu.x, menu.bottom() - 1, menu.w, 1), C_LINE);
        for item in menu_items(menu) {
            let active =
                (item.index == 0 && self.ui.file_menu_open) || self.ui.active_menu == item.index;
            if active {
                canvas.fill_rect(item.rect, C_MENU_HOT);
            } else if self.ui.hovered_menu == Some(item.index) {
                canvas.fill_rect(item.rect, blend_color(C_TEXT, C_MENU, 0.10));
            }
            let color = if active { C_ACCENT_TEXT } else { C_TEXT };
            text.draw_centered(
                canvas,
                self.label(item.label, item.english),
                item.rect,
                14.0,
                color,
            );
        }
        canvas.fill_rect(
            Rect::new(menu.x + 8 + 12, menu.bottom() - 3, 64 - 24, 2),
            C_ACCENT,
        );
        let right = format!("NX Motion  ·  {}", self.data.display_title());
        let width = text.text_width(&right, 11.0);
        text.draw(
            canvas,
            &right,
            menu.right() - width - 12,
            menu.y + 8,
            11.0,
            C_DIM,
            menu,
        );
    }

    fn paint_ribbon(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        let ribbon = self.layout.ribbon;
        canvas.fill_rect(ribbon, C_RIBBON);
        canvas.fill_rect(
            Rect::new(ribbon.x, ribbon.bottom() - 1, ribbon.w, 1),
            C_LINE,
        );
        for (group_index, group) in ribbon_groups(ribbon).iter().enumerate() {
            let visible = group.rect.intersect(ribbon);
            if visible.is_empty() {
                continue;
            }
            canvas.fill_rect(visible, C_GROUP);
            canvas.outline_rect(visible, C_GROUP_EDGE);
            for (tile_index, tile) in group.tiles.iter().enumerate() {
                let tile_rect = tile.rect.intersect(ribbon);
                if tile_rect.is_empty() {
                    continue;
                }
                let hot = self.ui.hovered_tile == Some((group_index, tile_index));
                canvas.fill_rect(tile_rect, if hot { C_TILE_HOT } else { C_TILE });
                canvas.outline_rect(tile_rect, C_TILE_EDGE);
                draw_tile_label(
                    text,
                    canvas,
                    tile_rect,
                    self.label(tile.label, tile.english),
                );
            }
            let title_rect = Rect::new(group.rect.x, group.rect.bottom() - 16, group.rect.w, 14)
                .intersect(ribbon);
            text.draw_centered(
                canvas,
                self.label(group.name, group.english),
                title_rect,
                10.0,
                C_DIM,
            );
        }
    }

    fn paint_dock(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        for panel in &self.panels {
            canvas.fill_rect(panel.header, C_HEADER);
            canvas.fill_rect(
                Rect::new(panel.header.x, panel.header.bottom() - 1, panel.header.w, 1),
                C_LINE,
            );
            draw_chevron(
                canvas,
                panel.header.x + 13,
                panel.header.y + panel.header.h / 2,
                panel.collapsed,
                C_DIM,
            );
            text.draw(
                canvas,
                self.label(panel.id.title(), panel.id.title_en()),
                panel.header.x + 26,
                panel.header.y + 6,
                12.0,
                C_TEXT,
                panel.header.inset(0, 0, 6, 0),
            );
        }

        let navigator = self.panels[PanelId::Navigator.index()];
        if !navigator.body.is_empty() {
            canvas.fill_rect(navigator.body, C_PANEL);
            let rows = tree_rows(navigator.body, self.nodes, self.ui.tree_scroll);
            for row in &rows {
                let selected = self.ui.selected_node == Some(row.node);
                let hovered = self.ui.hovered_tree == Some(row.node);
                if selected {
                    canvas.fill_rect(row.rect, C_SELECT);
                } else if hovered {
                    canvas.fill_rect(row.rect, blend_color(C_TEXT, C_PANEL, 0.07));
                }
                let node = &self.nodes[row.node];
                let indent = 6 + row.depth as i32 * 12;
                let color = NODE_COLORS[node.kind as usize];
                canvas.fill_rect(
                    Rect::new(navigator.body.x + indent, row.rect.y + 7, 6, 6),
                    color,
                );
                let clip = Rect::new(
                    navigator.body.x,
                    row.rect.y,
                    (navigator.body.w - 4).max(0),
                    row.rect.h,
                );
                text.draw(
                    canvas,
                    &node.label,
                    navigator.body.x + indent + 12,
                    row.rect.y + 3,
                    12.0,
                    if selected { 0x00ffffff } else { C_TEXT },
                    clip,
                );
            }
            let max = max_tree_scroll(navigator.body, self.nodes.len());
            if max > 0 {
                let track = Rect::new(
                    navigator.body.right() - 5,
                    navigator.body.y + 2,
                    3,
                    (navigator.body.h - 4).max(0),
                );
                canvas.fill_rect(track, 0x00252f38);
                let thumb_h = (track.h as f32 * navigator.body.h as f32
                    / (self.nodes.len() as f32 * TREE_ROW_HEIGHT as f32))
                    .max(16.0) as i32;
                let travel = (track.h - thumb_h).max(0) as f32;
                let offset = travel * self.ui.tree_scroll as f32 / max as f32;
                canvas.fill_rect(
                    Rect::new(track.x, track.y + offset as i32, track.w, thumb_h),
                    C_DIM,
                );
            }
        }

        let solution = self.panels[PanelId::Solution.index()];
        if !solution.body.is_empty() {
            canvas.fill_rect(solution.body, C_PANEL);
            let inner = solution.body.inset(10, 6, 10, 4);
            let mut y = inner.y;
            if let Some(plan) = &self.data.solution {
                text.draw(
                    canvas,
                    &format!(
                        "{}  {}",
                        self.label("解算方案", "Solution"),
                        if plan.name.is_empty() {
                            "--"
                        } else {
                            plan.name.as_str()
                        }
                    ),
                    inner.x,
                    y,
                    11.5,
                    C_TEXT,
                    inner,
                );
                y += 18;
                text.draw(
                    canvas,
                    &format!(
                        "{}      {}",
                        self.label("时间", "Time"),
                        plan.time
                            .map_or_else(|| String::from("--"), |time| format!("{time} s"))
                    ),
                    inner.x,
                    y,
                    11.5,
                    C_DIM,
                    inner,
                );
                y += 18;
                text.draw(
                    canvas,
                    &format!(
                        "{}      {}",
                        self.label("步数", "Steps"),
                        plan.steps
                            .map_or_else(|| String::from("--"), |steps| steps.to_string())
                    ),
                    inner.x,
                    y,
                    11.5,
                    C_DIM,
                    inner,
                );
                y += 18;
                text.draw(
                    canvas,
                    &format!(
                        "{}      {}",
                        self.label("状态", "Status"),
                        if self.scene.playing {
                            self.label("运行中", "Running")
                        } else {
                            self.label("暂停", "Paused")
                        }
                    ),
                    inner.x,
                    y,
                    11.5,
                    C_DIM,
                    inner,
                );
            } else {
                text.draw(
                    canvas,
                    self.label("无解算方案", "No solution"),
                    inner.x,
                    inner.y,
                    11.5,
                    C_FAINT,
                    inner,
                );
            }
        }

        let xy = self.panels[PanelId::XyResult.index()];
        if !xy.body.is_empty() {
            canvas.fill_rect(xy.body, C_PANEL);
            if self.data.channels.is_empty() {
                text.draw(
                    canvas,
                    self.label("无结果通道", "No result channels"),
                    xy.body.x + 10,
                    xy.body.y + 8,
                    11.5,
                    C_FAINT,
                    xy.body,
                );
            } else {
                let rows = channel_rows(xy.body, self.data.channels.len());
                for (index, rect) in rows.iter().enumerate() {
                    let channel = &self.data.channels[index];
                    if self.ui.selected_channel == Some(index) {
                        canvas.fill_rect(*rect, C_SELECT);
                    } else if rect.contains(self.mouse.0, self.mouse.1) {
                        canvas.fill_rect(*rect, blend_color(C_TEXT, C_PANEL, 0.07));
                    }
                    let clip = Rect::new(rect.x + 8, rect.y, (rect.w - 70).max(0), rect.h);
                    text.draw(
                        canvas,
                        &channel.path,
                        rect.x + 8,
                        rect.y + 3,
                        11.0,
                        C_TEXT,
                        clip,
                    );
                    let unit_width = text.text_width(&channel.unit, 10.0);
                    text.draw(
                        canvas,
                        &channel.unit,
                        rect.right() - unit_width - 10,
                        rect.y + 3,
                        10.0,
                        C_DIM,
                        *rect,
                    );
                }
                let list_bottom = rows.last().map_or(xy.body.y, |row| row.bottom());
                let plot = Rect::new(
                    xy.body.x + 8,
                    list_bottom + 4,
                    (xy.body.w - 16).max(0),
                    (xy.body.bottom() - list_bottom - 10).max(0),
                );
                if !plot.is_empty() {
                    canvas.fill_rect(plot, 0x00161d24);
                    canvas.outline_rect(plot, C_LINE);
                    for step in 1..4 {
                        let x = plot.x + plot.w * step / 4;
                        canvas.line(x, plot.y + 1, x, plot.bottom() - 2, 0x002a343d);
                        let y = plot.y + plot.h * step / 4;
                        canvas.line(plot.x + 1, y, plot.right() - 2, y, 0x002a343d);
                    }
                    let label = if self.ui.selected_channel.is_some() {
                        self.label("无采样数据", "No sample data")
                    } else {
                        self.label("选择通道", "Select a channel")
                    };
                    text.draw_centered(canvas, label, plot, 10.0, C_FAINT);
                }
            }
        }

        let flex = self.panels[PanelId::FlexBody.index()];
        if !flex.body.is_empty() {
            canvas.fill_rect(flex.body, C_PANEL);
            if self.data.flex_modes.is_empty() {
                text.draw(
                    canvas,
                    self.label("无柔性体模态数据", "No flexible body mode data"),
                    flex.body.x + 10,
                    flex.body.y + 8,
                    11.5,
                    C_FAINT,
                    flex.body,
                );
            } else {
                for (index, mode) in self.data.flex_modes.iter().enumerate() {
                    let y = flex.body.y + 6 + index as i32 * 18;
                    if y + 18 > flex.body.bottom() {
                        break;
                    }
                    let label = flex_mode_label(mode);
                    text.draw(
                        canvas,
                        &format!("{}. {label}", index + 1),
                        flex.body.x + 10,
                        y,
                        11.0,
                        C_TEXT,
                        flex.body.inset(6, 0, 6, 0),
                    );
                }
            }
        }
    }

    fn paint_status(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        let status = self.layout.status;
        if status.is_empty() {
            return;
        }
        canvas.fill_rect(status, C_STATUS);
        canvas.fill_rect(Rect::new(status.x, status.y, status.w, 1), C_LINE);
        let width = status.w.max(1);
        let widths = [
            width * 32 / 100,
            width * 16 / 100,
            width * 24 / 100,
            width * 13 / 100,
        ];
        let mut x = status.x;
        let mut rects = Vec::new();
        for part in widths {
            rects.push(Rect::new(x, status.y, part, status.h));
            x += part;
        }
        rects.push(Rect::new(x, status.y, status.right() - x, status.h));
        for rect in rects.iter().skip(1) {
            canvas.fill_rect(Rect::new(rect.x, rect.y + 4, 1, rect.h - 8), C_LINE);
        }
        let hover_tip = self.ui.hovered_tile.and_then(|(group_index, tile_index)| {
            let groups = ribbon_groups(self.layout.ribbon);
            let group = groups.get(group_index)?;
            let tile = group.tiles.get(tile_index)?;
            let command = if tile.command.is_empty() {
                "命令未恢复 / command not recovered"
            } else {
                tile.command
            };
            Some(format!("{} / {}  [{command}]", group.name, tile.label))
        });
        let labels = [
            format!("{}: {}", self.label("文件", "File"), self.data.assembly),
            format!(
                "{}: {}/{}",
                self.label("帧", "Frame"),
                self.scene.frame + 1,
                self.frames
            ),
            format!(
                "{}: {}",
                self.label("选择", "Selection"),
                self.selected_label
            ),
            format!(
                "{}: {}",
                self.label("模式", "Mode"),
                if self.scene.playing {
                    self.label("播放", "Play")
                } else {
                    self.label("暂停", "Pause")
                }
            ),
            if let Some(tip) = hover_tip {
                tip
            } else if self.ui.message.is_empty() {
                self.label("就绪", "Ready").to_string()
            } else {
                self.ui.message.clone()
            },
        ];
        for (rect, label) in rects.iter().zip(labels.iter()) {
            text.draw(
                canvas,
                label,
                rect.x + 8,
                rect.y + 5,
                11.0,
                C_DIM,
                rect.inset(0, 0, 6, 0),
            );
        }
    }

    fn paint_help(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        if !self.ui.help_open {
            return;
        }
        let width = 620.min(self.layout.width - 40).max(360);
        let height = 330.min(self.layout.height - 40).max(240);
        let rect = Rect::new(
            (self.layout.width - width) / 2,
            (self.layout.height - height) / 2,
            width,
            height,
        );
        canvas.fill_rect(Rect::new(rect.x + 5, rect.y + 5, rect.w, rect.h), C_SHADOW);
        canvas.fill_rect(rect, C_MENU_BG);
        canvas.outline_rect(rect, C_MENU_EDGE);
        canvas.fill_rect(Rect::new(rect.x, rect.y, rect.w, 30), C_HEADER);
        text.draw(
            canvas,
            self.label("NX Motion · 交互帮助", "NX Motion · Interaction help"),
            rect.x + 12,
            rect.y + 8,
            13.0,
            C_TEXT,
            rect,
        );
        let lines = if self.chinese {
            [
                "Tab        切换焦点：Motion 导航器 / 3D 视口 / Ribbon",
                "导航器     ↑/↓ 选择对象，Enter 打开属性，双击对象打开属性",
                "视口       左键旋转，右键/中键平移，滚轮缩放（每格 12%）",
                "动画       Space 播放/暂停，←/→ 或 ↑/↓ 步进，Home 首帧",
                "面板       P 属性，F1 帮助，R 重置，F 适配，S 截图",
                "文件       文件菜单 / 首选项 / 退出",
            ]
        } else {
            [
                "Tab        cycle focus: Motion Navigator / 3D viewport / Ribbon",
                "Navigator  Up/Down select, Enter properties, double-click properties",
                "Viewport   left orbit, right/middle pan, wheel zoom (12% per notch)",
                "Animation  Space play/pause, arrows step, Home first frame",
                "Panels     P properties, F1 help, R reset, F fit, S screenshot",
                "File       File menu / Preferences / Exit",
            ]
        };
        for (index, line) in lines.iter().enumerate() {
            text.draw(
                canvas,
                line,
                rect.x + 16,
                rect.y + 44 + index as i32 * 28,
                11.5,
                C_TEXT,
                rect,
            );
        }
        text.draw(
            canvas,
            self.label("F1 / Esc 关闭", "F1 / Esc close"),
            rect.x + 16,
            rect.bottom() - 26,
            10.0,
            C_FAINT,
            rect,
        );
    }

    fn paint_properties(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        if !self.ui.properties_open {
            return;
        }
        let Some(node) = self
            .ui
            .selected_node
            .and_then(|index| self.nodes.get(index))
        else {
            return;
        };
        let Some((name, properties)) = node_properties(self.data, node) else {
            return;
        };
        let row_height = 22;
        let rect = properties_panel_rect(self.layout.viewport, properties.len());
        let shadow = Rect::new(rect.x + 4, rect.y + 4, rect.w, rect.h);
        canvas.fill_rect(shadow, C_SHADOW);
        canvas.fill_rect(rect, C_MENU_BG);
        canvas.outline_rect(rect, C_MENU_EDGE);
        canvas.fill_rect(Rect::new(rect.x, rect.y, rect.w, 28), C_HEADER);
        text.draw(
            canvas,
            &format!("{} · {}", name, self.label("属性", "Properties")),
            rect.x + 10,
            rect.y + 7,
            12.0,
            C_TEXT,
            rect,
        );
        let mut y = rect.y + 36;
        if properties.is_empty() {
            text.draw(
                canvas,
                self.label("无可编辑参数", "No parameters"),
                rect.x + 10,
                y,
                11.0,
                C_DIM,
                rect,
            );
        }
        for property in properties {
            if y + row_height > rect.bottom() - 24 {
                break;
            }
            let label = if self.chinese {
                &property.label_zh
            } else {
                &property.label_en
            };
            text.draw(canvas, label, rect.x + 10, y, 11.0, C_DIM, rect);
            let value = format_property_value(property);
            let value_width = text.text_width(&value, 11.0);
            let color = if property.writable {
                NODE_COLORS[3]
            } else if property.nxopen_writable {
                C_ACCENT_TEXT
            } else {
                C_FAINT
            };
            text.draw(
                canvas,
                &value,
                (rect.right() - value_width - 10).max(rect.x + 110),
                y,
                11.0,
                color,
                rect,
            );
            y += row_height;
        }
        text.draw(
            canvas,
            if self.chinese {
                "P 切换 · 绿=离线可写 · 橙=NXOpen"
            } else {
                "P toggle · green=offline · amber=NXOpen"
            },
            rect.x + 10,
            rect.bottom() - 18,
            9.5,
            C_FAINT,
            rect,
        );
    }

    fn paint_viewport(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        let viewport = self.layout.viewport;
        if viewport.is_empty() {
            return;
        }
        canvas.outline_rect(viewport, C_VIEW_BORDER);
        let chip = Rect::new(viewport.x + 10, viewport.y + 10, 56, 22);
        canvas.fill_rect(chip, 0x00263039);
        canvas.outline_rect(chip, C_LINE);
        text.draw_centered(
            canvas,
            self.label("透视", "Perspective"),
            chip,
            11.0,
            C_TEXT,
        );

        let frame_label = format!(
            "{} {}/{}  t={:.3}s",
            self.label("帧", "Frame"),
            self.scene.frame + 1,
            self.frames,
            self.scene.time()
        );
        let frame_width = text.text_width(&frame_label, 11.0);
        let frame_rect = Rect::new(
            viewport.right() - frame_width - 18,
            viewport.y + 10,
            frame_width + 8,
            22,
        );
        canvas.fill_rect(frame_rect, 0x00263039);
        text.draw(
            canvas,
            &frame_label,
            frame_rect.x + 4,
            frame_rect.y + 5,
            11.0,
            C_DIM,
            frame_rect,
        );

        self.paint_triad(canvas, text, viewport);

        let hint = self.label(
            "左键旋转  右键平移  滚轮缩放",
            "Left-drag orbit  Right-drag pan  Wheel zoom",
        );
        let hint_width = text.text_width(hint, 10.0);
        text.draw(
            canvas,
            hint,
            viewport.right() - hint_width - 12,
            viewport.bottom() - 24,
            10.0,
            C_FAINT,
            viewport,
        );
    }

    fn paint_triad(&self, canvas: &mut Canvas, text: &mut TextRenderer, viewport: Rect) {
        let origin = (viewport.x + 34, viewport.bottom() - 36);
        let eye = self.camera.eye();
        let forward = (self.camera.target - eye).normalize_or_zero();
        let right = forward.cross(Vec3::Y).normalize_or_zero();
        let up = right.cross(forward).normalize_or_zero();
        let axes = [
            (Vec3::X, 0x00e0655f, "X"),
            (Vec3::Y, 0x007fc06a, "Y"),
            (Vec3::Z, 0x005f8fe0, "Z"),
        ];
        for (axis, color, label) in axes {
            let dx = axis.dot(right) * 20.0;
            let dy = axis.dot(up) * 20.0;
            let end = (origin.0 + dx as i32, origin.1 - dy as i32);
            canvas.line(origin.0, origin.1, end.0, end.1, color);
            text.draw(canvas, label, end.0 - 3, end.1 - 6, 10.0, color, viewport);
        }
    }

    fn paint_file_menu(&self, canvas: &mut Canvas, text: &mut TextRenderer) {
        let file = file_menu_rect(self.layout.menu);
        let shadow = Rect::new(file.x + 4, file.y + 4, file.w, file.h);
        canvas.fill_rect(shadow, C_SHADOW);
        canvas.fill_rect(file, C_MENU_BG);
        canvas.outline_rect(file, C_MENU_EDGE);
        for (index, row) in file_menu_rows(file).iter().enumerate() {
            let hot = self.ui.hovered_file == Some(index) || (index == 4 && self.ui.prefs_open);
            if hot {
                canvas.fill_rect(*row, C_MENU_HOT);
            }
            text.draw(
                canvas,
                self.label(FILE_MENU_ITEMS[index], FILE_MENU_ITEMS_EN[index]),
                row.x + 14,
                row.y + 6,
                12.0,
                C_TEXT,
                *row,
            );
            if index == 4 {
                draw_chevron(canvas, row.right() - 16, row.y + row.h / 2, false, C_DIM);
            }
            if matches!(index, 3 | 9 | 13) {
                canvas.fill_rect(
                    Rect::new(row.x + 8, row.bottom() - 1, row.w - 16, 1),
                    C_LINE,
                );
            }
        }

        if self.ui.prefs_open {
            let prefs = preferences_rect(file);
            canvas.fill_rect(
                Rect::new(prefs.x + 4, prefs.y + 4, prefs.w, prefs.h),
                C_SHADOW,
            );
            canvas.fill_rect(prefs, C_MENU_BG);
            canvas.outline_rect(prefs, C_MENU_EDGE);
            for (index, row) in preferences_rows(prefs).iter().enumerate() {
                if self.ui.hovered_pref == Some(index) {
                    canvas.fill_rect(*row, C_MENU_HOT);
                }
                text.draw(
                    canvas,
                    self.label(PREFERENCES_ITEMS[index], PREFERENCES_ITEMS_EN[index]),
                    row.x + 14,
                    row.y + 6,
                    12.0,
                    C_TEXT,
                    *row,
                );
            }
        }
    }
}

fn draw_chevron(canvas: &mut Canvas, x: i32, y: i32, collapsed: bool, color: u32) {
    if collapsed {
        canvas.line(x - 2, y - 3, x + 1, y, color);
        canvas.line(x + 1, y, x - 2, y + 3, color);
    } else {
        canvas.line(x - 3, y - 2, x, y + 1, color);
        canvas.line(x, y + 1, x + 3, y - 2, color);
    }
}

fn draw_tile_label(text: &mut TextRenderer, canvas: &mut Canvas, rect: Rect, label: &str) {
    let px = 9.0;
    let width = text.text_width(label, px);
    let available = (rect.w - 4).max(0);
    if width <= available {
        text.draw(
            canvas,
            label,
            rect.x + (rect.w - width) / 2,
            rect.y + (rect.h - 11) / 2,
            px,
            C_TEXT,
            rect,
        );
        return;
    }
    let characters: Vec<char> = label.chars().collect();
    let middle = characters.len().div_ceil(2);
    let split = characters
        .iter()
        .enumerate()
        .filter(|(_, ch)| ch.is_whitespace())
        .min_by_key(|(index, _)| index.abs_diff(middle))
        .map_or(middle, |(index, _)| index);
    let first: String = characters[..split]
        .iter()
        .collect::<String>()
        .trim()
        .to_string();
    let second: String = characters[split..]
        .iter()
        .collect::<String>()
        .trim()
        .to_string();
    for (line, part) in [(0, &first), (1, &second)] {
        let part_width = text.text_width(part, px);
        text.draw(
            canvas,
            part,
            rect.x + (rect.w - part_width) / 2,
            rect.y + 4 + line * 10,
            px,
            C_TEXT,
            rect,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn focus_cycles_through_navigator_viewport_and_ribbon() {
        assert_eq!(next_focus(Focus::Navigator), Focus::Viewport);
        assert_eq!(next_focus(Focus::Viewport), Focus::Ribbon);
        assert_eq!(next_focus(Focus::Ribbon), Focus::Navigator);
    }

    #[test]
    fn one_wheel_notch_scrolls_three_tree_rows() {
        assert!((wheel_tree_rows(crate::WHEEL_NOTCH) - 3.0).abs() < 1e-5);
        assert!((wheel_tree_rows(crate::WHEEL_NOTCH / 4.0) - 0.75).abs() < 1e-5);
        assert_eq!(wheel_tree_rows(1000.0), 9.0);
        assert_eq!(wheel_tree_rows(-1000.0), -9.0);
    }

    #[test]
    fn tree_index_starts_at_zero_and_clamps() {
        assert_eq!(next_tree_index(None, 1, 4), Some(0));
        assert_eq!(next_tree_index(Some(0), -1, 4), Some(0));
        assert_eq!(next_tree_index(Some(3), 1, 4), Some(3));
        assert_eq!(next_tree_index(None, 1, 0), None);
    }

    #[test]
    fn property_value_formats_number_and_unit() {
        let mut property = Property::default();
        property.value = serde_json::json!(0.5);
        property.unit = Some(String::from("kg"));
        assert_eq!(format_property_value(&property), "0.5 kg");
        property.value = serde_json::json!(null);
        assert_eq!(format_property_value(&property), "—");
    }

    #[test]
    fn properties_panel_stays_inside_the_viewport() {
        let rect = properties_panel_rect(Rect::new(320, 124, 1280, 752), 5);
        assert!(rect.x >= 320);
        assert!(rect.right() <= 1600);
        assert!(rect.y >= 124);
        assert!(rect.bottom() <= 876);
    }
}
