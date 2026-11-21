// SPDX-License-Identifier: Apache-2.0
//! Rectangles and hit-test geometry for the Motion window chrome.

use super::data::TreeNode;

/// Screen-space rectangle in physical window pixels.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rect {
    pub x: i32,
    pub y: i32,
    pub w: i32,
    pub h: i32,
}

impl Rect {
    pub const fn new(x: i32, y: i32, w: i32, h: i32) -> Self {
        Self { x, y, w, h }
    }

    pub const fn right(self) -> i32 {
        self.x + self.w
    }

    pub const fn bottom(self) -> i32 {
        self.y + self.h
    }

    pub const fn is_empty(self) -> bool {
        self.w <= 0 || self.h <= 0
    }

    pub const fn contains(self, x: i32, y: i32) -> bool {
        x >= self.x && x < self.right() && y >= self.y && y < self.bottom()
    }

    pub const fn inset(self, left: i32, top: i32, right: i32, bottom: i32) -> Self {
        Self {
            x: self.x + left,
            y: self.y + top,
            w: self.w - left - right,
            h: self.h - top - bottom,
        }
    }

    pub const fn intersect(self, other: Self) -> Self {
        let x = if self.x > other.x { self.x } else { other.x };
        let y = if self.y > other.y { self.y } else { other.y };
        let right = if self.right() < other.right() {
            self.right()
        } else {
            other.right()
        };
        let bottom = if self.bottom() < other.bottom() {
            self.bottom()
        } else {
            other.bottom()
        };
        Self {
            x,
            y,
            w: right - x,
            h: bottom - y,
        }
    }
}

pub const MENU_HEIGHT: i32 = 28;
pub const RIBBON_HEIGHT: i32 = 96;
pub const DOCK_WIDTH: i32 = 320;
pub const STATUS_HEIGHT: i32 = 24;
pub const PANEL_HEADER_HEIGHT: i32 = 26;
pub const MIN_VIEWPORT_WIDTH: i32 = 200;

pub const MENU_LABELS: [&str; 8] = [
    "文件",
    "主页",
    "分析",
    "结果",
    "几何体",
    "视图",
    "显示",
    "工具",
];

pub const MENU_LABELS_EN: [&str; 8] = [
    "File", "Home", "Analysis", "Results", "Geometry", "View", "Display", "Tools",
];

pub const FILE_MENU_ITEMS: [&str; 15] = [
    "新建",
    "打开",
    "关闭",
    "保存",
    "首选项",
    "打印",
    "绘图",
    "导入",
    "导出",
    "导出运动",
    "实用工具",
    "执行",
    "属性",
    "帮助",
    "退出",
];

pub const FILE_MENU_ITEMS_EN: [&str; 15] = [
    "New",
    "Open",
    "Close",
    "Save",
    "Preferences",
    "Print",
    "Plot",
    "Import",
    "Export",
    "Export Motion",
    "Utilities",
    "Execute",
    "Properties",
    "Help",
    "Exit",
];

pub const PREFERENCES_ITEMS: [&str; 6] = [
    "运动",
    "函数图",
    "求解器首选项",
    "装配加载选项",
    "PMI",
    "装配",
];

pub const PREFERENCES_ITEMS_EN: [&str; 6] = [
    "Motion",
    "Function Graph",
    "Solver Preferences",
    "Assembly Load Options",
    "PMI",
    "Assembly",
];

/// Recovered command IDs for the Motion preference flyout. The last three
/// entries have no Motion command in `motion_ui_strings.json`.
pub const PREFERENCES_COMMANDS: [Option<&str>; 6] = [
    Some("UG_MECH_PREFERENCES_MOTION"),
    Some("UG_MECH_PREFERENCES_XYPLOT"),
    Some("UG_PREFERENCES_SOLVER"),
    None,
    None,
    None,
];

pub const MENU_ITEM_WIDTH: i32 = 64;
pub const FILE_MENU_WIDTH: i32 = 240;
pub const FLYOUT_WIDTH: i32 = 220;
pub const MENU_ROW_HEIGHT: i32 = 26;

pub struct RibbonButtonSpec {
    pub label: &'static str,
    pub english: &'static str,
    pub command: &'static str,
}

pub struct RibbonGroupSpec {
    pub name: &'static str,
    pub english: &'static str,
    pub buttons: &'static [RibbonButtonSpec],
}

const fn button(
    label: &'static str,
    english: &'static str,
    command: &'static str,
) -> RibbonButtonSpec {
    RibbonButtonSpec {
        label,
        english,
        command,
    }
}

/// Home tab groups and command IDs from `MOTION-UI-RE-2026-11-20.md` §3.3/§4.
pub const RIBBON_GROUPS: &[RibbonGroupSpec] = &[
    RibbonGroupSpec {
        name: "解算方案",
        english: "Solution",
        buttons: &[
            button("解算方案", "Solution", "UG_MECH_INSERT_SOLUTION"),
            button(
                "初始条件",
                "Initial Conditions",
                "UG_MECH_INSERT_INITIAL_CONDITION",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "机构",
        english: "Mechanism",
        buttons: &[
            button("运动体", "Motion Body", "UG_MECH_INSERT_LINK"),
            button("运动副", "Joint", "UG_MECH_INSERT_JOINT"),
            button(
                "复合运动副",
                "Compound Joint",
                "UG_MECH_INSERT_COMPOUND_JOINT",
            ),
            button("驱动体", "Driver", "UG_MECH_INSERT_DRIVER"),
            button("标记", "Marker", "UG_MECH_INSERT_MARKER"),
            button("曲线", "Profile", "UG_MECH_INSERT_PROFILE"),
            button(
                "仅 MDF 单元",
                "MDF Only Elements",
                "UG_MECH_SHOW_MDF_ONLY_ELEMENTS",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "耦合副",
        english: "Coupler",
        buttons: &[
            button(
                "运动体耦合副",
                "Motion Body Coupler",
                "UG_MECH_INSERT_JNT_LINK_COUPLER",
            ),
            button("齿轮耦合副", "Gear Coupler", "UG_MECH_INSERT_JNT_GEAR"),
        ],
    },
    RibbonGroupSpec {
        name: "连接副",
        english: "Connector",
        buttons: &[
            button("弹簧", "Spring", "UG_MECH_INSERT_FRC_SPNG"),
            button("阻尼器", "Damper", "UG_MECH_INSERT_FRC_DMP"),
            button("弹簧阻尼器", "Spring Damper", "UG_MECH_INSERT_FRC_SPNG_DMP"),
            button("衬套", "Bushing", "UG_MECH_INSERT_BUSHING"),
            button("梁单元力", "Beam Force", "UG_MECH_INSERT_FRC_BEAM_FORCE"),
            button("轴承", "Bearing", "UG_MECH_INSERT_BEARING"),
        ],
    },
    RibbonGroupSpec {
        name: "接触",
        english: "Contact",
        buttons: &[
            button("3D 接触", "3D Contact", "UG_MECH_INSERT_FRC_CONT"),
            button("齿轮接触", "Gear Contact", "UG_MECH_INSERT_GEAR_CONTACT"),
            button(
                "分析接触",
                "Analytical Contact",
                "UG_MECH_INSERT_ANALYTICAL_CONTACT",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "约束",
        english: "Constraint",
        buttons: &[
            button("点在线上副", "Point on Curve", "UG_MECH_INSERT_JNT_PT_CRV"),
            button("线在线上副", "Curve on Curve", "UG_MECH_INSERT_JNT_CRV_CRV"),
            button(
                "点在面上副",
                "Point on Surface",
                "UG_MECH_INSERT_PNT_SURFACE",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "加载",
        english: "Load",
        buttons: &[
            button("标量力", "Scalar Force", "UG_MECH_INSERT_FRC_SCL_FRC"),
            button("标量扭矩", "Scalar Torque", "UG_MECH_INSERT_FRC_SCL_TRQ"),
            button("矢量力", "Vector Force", "UG_MECH_INSERT_FRC_VECT_FRC"),
            button("矢量扭矩", "Vector Torque", "UG_MECH_INSERT_FRC_VECT_TRQ"),
            button(
                "电磁力",
                "Electromagnetic Force",
                "UG_MECH_INSERT_ELECTROMAGNETIC_FORCE",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "子机构",
        english: "Submechanism",
        buttons: &[
            button("添加", "Add", "UG_MECH_ADD_SUBMECHANISM"),
            button("移动", "Move", "UG_MECH_MOVE_SUBMECHANISM"),
        ],
    },
    RibbonGroupSpec {
        name: "柔性",
        english: "Flexibility",
        buttons: &[
            button("柔性体", "Flexible Body", "UG_MECH_INSERT_FLEXBODY"),
            button(
                "样条梁属性",
                "Spline Beam Property",
                "UG_MECH_INSERT_SPLINE_BEAM_PROPERTY",
            ),
            button(
                "自动柔性体",
                "Automatic Flex",
                "UG_MECH_INSERT_AUTOMATIC_FLEX",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "控制",
        english: "Control",
        buttons: &[
            button(
                "控制块图表",
                "Control Block Diagram",
                "UG_MECH_CREATE_CONTROL_BLOCK_DIAGRAM",
            ),
            button("控制输入", "Control Input", "UG_MECH_INSERT_CONTROL_INPUT"),
            button(
                "控制输出",
                "Control Output",
                "UG_MECH_INSERT_CONTROL_OUTPUT",
            ),
        ],
    },
    RibbonGroupSpec {
        name: "车辆",
        english: "Vehicle",
        buttons: &[
            button("轮胎", "Tire", "UG_MECH_INSERT_TIRE"),
            button("路面", "Road", "UG_MECH_INSERT_ROAD"),
            button(
                "基本轮胎属性",
                "Basic Tire Property",
                "UG_MECH_INSERT_TIRE_PROPERTY_BASIC",
            ),
            button(
                "车辆报告",
                "Vehicle Reporting",
                "UG_MECH_INSERT_VEHICLE_REPORTING",
            ),
            button(
                "悬架报告",
                "Suspension Reporting",
                "UG_MECH_INSERT_SUSPENSION_REPORTING",
            ),
            button("簧载质量", "Sprung Mass", "UG_MECH_INSERT_SPRUNG_MASS"),
        ],
    },
    RibbonGroupSpec {
        name: "传动机构",
        english: "Powertrain",
        buttons: &[button("待补", "Not recovered", "")],
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanelId {
    Navigator,
    Solution,
    XyResult,
    FlexBody,
}

impl PanelId {
    pub const ALL: [PanelId; 4] = [
        PanelId::Navigator,
        PanelId::Solution,
        PanelId::XyResult,
        PanelId::FlexBody,
    ];

    pub const fn index(self) -> usize {
        match self {
            PanelId::Navigator => 0,
            PanelId::Solution => 1,
            PanelId::XyResult => 2,
            PanelId::FlexBody => 3,
        }
    }

    pub const fn title(self) -> &'static str {
        match self {
            PanelId::Navigator => "Motion 导航器",
            PanelId::Solution => "求解视图",
            PanelId::XyResult => "XY 结果视图",
            PanelId::FlexBody => "Motion 导航器柔性体模式局部放大图",
        }
    }

    pub const fn title_en(self) -> &'static str {
        match self {
            PanelId::Navigator => "Motion Navigator",
            PanelId::Solution => "Solution View",
            PanelId::XyResult => "XY Result View",
            PanelId::FlexBody => "Motion Navigator Flex Mode Details View",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelGeometry {
    pub id: PanelId,
    pub header: Rect,
    pub body: Rect,
    pub collapsed: bool,
}

/// Main window split. Sizes follow `NXRE-MOTION-UI-SPEC.md` at 1600x900.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    pub width: i32,
    pub height: i32,
    pub menu: Rect,
    pub ribbon: Rect,
    pub dock: Rect,
    pub viewport: Rect,
    pub status: Rect,
}

impl Layout {
    pub fn new(width: usize, height: usize) -> Self {
        let width = i32::try_from(width).unwrap_or(i32::MAX);
        let height = i32::try_from(height).unwrap_or(i32::MAX);
        let menu_h = MENU_HEIGHT.min(height);
        let ribbon_h = RIBBON_HEIGHT.min((height - menu_h).max(0));
        let status_h = STATUS_HEIGHT.min((height - menu_h - ribbon_h).max(0));
        let content_top = menu_h + ribbon_h;
        let content_h = (height - content_top - status_h).max(0);
        let dock_w = DOCK_WIDTH.min((width - MIN_VIEWPORT_WIDTH).max(0));
        let viewport_w = (width - dock_w).max(0);
        Self {
            width,
            height,
            menu: Rect::new(0, 0, width, menu_h),
            ribbon: Rect::new(0, menu_h, width, ribbon_h),
            dock: Rect::new(0, content_top, dock_w, content_h),
            viewport: Rect::new(dock_w, content_top, viewport_w, content_h),
            status: Rect::new(0, content_top + content_h, width, status_h),
        }
    }

    /// Panel headers and bodies down the dock, in spec order.
    pub fn panels(self, collapsed: [bool; 4]) -> [PanelGeometry; 4] {
        let header_h = PANEL_HEADER_HEIGHT;
        let body_space = (self.dock.h - 4 * header_h).max(0);
        let expanded = |id: PanelId| !collapsed[id.index()];
        let fixed_body = (if expanded(PanelId::Solution) { 84 } else { 0 })
            + (if expanded(PanelId::XyResult) { 128 } else { 0 })
            + (if expanded(PanelId::FlexBody) { 96 } else { 0 });
        let mut body_heights = [0_i32; 4];
        body_heights[PanelId::Navigator.index()] = if expanded(PanelId::Navigator) {
            (body_space - fixed_body).max(0)
        } else {
            0
        };
        body_heights[PanelId::Solution.index()] = if expanded(PanelId::Solution) {
            fixed_body.min(body_space).min(84)
        } else {
            0
        };
        body_heights[PanelId::XyResult.index()] = if expanded(PanelId::XyResult) {
            fixed_body.min(body_space).min(128)
        } else {
            0
        };
        body_heights[PanelId::FlexBody.index()] = if expanded(PanelId::FlexBody) {
            fixed_body.min(body_space).min(96)
        } else {
            0
        };

        let mut y = self.dock.y;
        let mut result = [PanelGeometry {
            id: PanelId::Navigator,
            header: Rect::default(),
            body: Rect::default(),
            collapsed: true,
        }; 4];
        for id in PanelId::ALL {
            let index = id.index();
            let header = Rect::new(self.dock.x, y, self.dock.w, header_h);
            y += header_h;
            let body_h = body_heights[index].min((self.dock.bottom() - y).max(0));
            let body = Rect::new(self.dock.x, y, self.dock.w, body_h);
            y += body_h;
            result[index] = PanelGeometry {
                id,
                header,
                body,
                collapsed: collapsed[index],
            };
        }
        result
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MenuHit {
    pub index: usize,
    pub label: &'static str,
    pub english: &'static str,
    pub rect: Rect,
}

pub fn menu_items(area: Rect) -> Vec<MenuHit> {
    MENU_LABELS
        .iter()
        .zip(MENU_LABELS_EN)
        .enumerate()
        .map(|(index, (label, english))| MenuHit {
            index,
            label,
            english,
            rect: Rect::new(
                area.x + 8 + index as i32 * MENU_ITEM_WIDTH,
                area.y,
                MENU_ITEM_WIDTH,
                area.h,
            ),
        })
        .collect()
}

pub fn file_menu_rect(menu: Rect) -> Rect {
    Rect::new(
        menu.x + 8,
        menu.bottom(),
        FILE_MENU_WIDTH,
        MENU_ROW_HEIGHT * FILE_MENU_ITEMS.len() as i32,
    )
}

pub fn file_menu_rows(area: Rect) -> Vec<Rect> {
    (0..FILE_MENU_ITEMS.len())
        .map(|index| {
            Rect::new(
                area.x,
                area.y + index as i32 * MENU_ROW_HEIGHT,
                area.w,
                MENU_ROW_HEIGHT,
            )
        })
        .collect()
}

pub fn preferences_rect(file_menu: Rect) -> Rect {
    Rect::new(
        file_menu.right(),
        file_menu.y + 4 * MENU_ROW_HEIGHT,
        FLYOUT_WIDTH,
        MENU_ROW_HEIGHT * PREFERENCES_ITEMS.len() as i32,
    )
}

pub fn preferences_rows(area: Rect) -> Vec<Rect> {
    (0..PREFERENCES_ITEMS.len())
        .map(|index| {
            Rect::new(
                area.x,
                area.y + index as i32 * MENU_ROW_HEIGHT,
                area.w,
                MENU_ROW_HEIGHT,
            )
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RibbonTile {
    pub label: &'static str,
    pub english: &'static str,
    pub command: &'static str,
    pub rect: Rect,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RibbonGroup {
    pub name: &'static str,
    pub english: &'static str,
    pub rect: Rect,
    pub tiles: Vec<RibbonTile>,
}

/// Two-row group tiles, compressed horizontally when the window is narrow.
pub fn ribbon_groups(area: Rect) -> Vec<RibbonGroup> {
    const BASE_TILE_WIDTH: i32 = 58;
    const MIN_TILE_WIDTH: i32 = 30;
    const TILE_GAP: i32 = 3;
    const GROUP_GAP: i32 = 6;
    const PAD: i32 = 4;
    const TILE_HEIGHT: i32 = 28;
    const TILE_TOP: i32 = 6;

    let columns = |count: usize| count.div_ceil(2);
    let total_width = |tile_width: i32| -> i32 {
        RIBBON_GROUPS
            .iter()
            .map(|group| {
                let cols = columns(group.buttons.len()) as i32;
                PAD * 2 + cols * tile_width + (cols - 1).max(0) * TILE_GAP + GROUP_GAP
            })
            .sum()
    };
    let desired = total_width(BASE_TILE_WIDTH);
    let mut tile_width = if desired > area.w && desired > 0 {
        ((BASE_TILE_WIDTH as f32 * area.w as f32 / desired as f32).floor() as i32)
            .clamp(MIN_TILE_WIDTH, BASE_TILE_WIDTH)
    } else {
        BASE_TILE_WIDTH
    };
    while tile_width > MIN_TILE_WIDTH && total_width(tile_width) > area.w {
        tile_width -= 1;
    }

    let mut groups = Vec::new();
    let mut x = area.x + 2;
    for group in RIBBON_GROUPS {
        let cols = columns(group.buttons.len()) as i32;
        let group_w = PAD * 2 + cols * tile_width + (cols - 1).max(0) * TILE_GAP;
        let rect = Rect::new(x, area.y + 2, group_w, (area.h - 4).max(0));
        let mut tiles = Vec::new();
        for (index, spec) in group.buttons.iter().enumerate() {
            let column = index / 2;
            let row = index % 2;
            let tile = Rect::new(
                rect.x + PAD + column as i32 * (tile_width + TILE_GAP),
                area.y + TILE_TOP + row as i32 * (TILE_HEIGHT + 2),
                tile_width,
                TILE_HEIGHT,
            );
            tiles.push(RibbonTile {
                label: spec.label,
                english: spec.english,
                command: spec.command,
                rect: tile,
            });
        }
        groups.push(RibbonGroup {
            name: group.name,
            english: group.english,
            rect,
            tiles,
        });
        x += group_w + GROUP_GAP;
    }
    groups
}

pub const TREE_ROW_HEIGHT: i32 = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TreeRow {
    pub node: usize,
    pub depth: usize,
    pub rect: Rect,
}

pub fn max_tree_scroll(area: Rect, node_count: usize) -> i32 {
    let content = i32::try_from(node_count).unwrap_or(i32::MAX) * TREE_ROW_HEIGHT;
    (content - area.h).max(0)
}

pub fn tree_rows(area: Rect, nodes: &[TreeNode], scroll: i32) -> Vec<TreeRow> {
    let scroll = scroll.clamp(0, max_tree_scroll(area, nodes.len()));
    let mut rows = Vec::new();
    for (node, item) in nodes.iter().enumerate() {
        let y = area.y + node as i32 * TREE_ROW_HEIGHT - scroll;
        if y + TREE_ROW_HEIGHT <= area.y || y >= area.bottom() {
            continue;
        }
        rows.push(TreeRow {
            node,
            depth: item.depth,
            rect: Rect::new(area.x, y, area.w, TREE_ROW_HEIGHT),
        });
    }
    rows
}

pub fn hit<T: Copy>(rects: impl IntoIterator<Item = (T, Rect)>, x: i32, y: i32) -> Option<T> {
    rects
        .into_iter()
        .find(|(_, rect)| rect.contains(x, y))
        .map(|(value, _)| value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion_ui::data::NodeKind;

    fn node(depth: usize) -> TreeNode {
        TreeNode {
            label: String::from("node"),
            kind: NodeKind::Body,
            depth,
            detail: None,
        }
    }

    #[test]
    fn layout_matches_spec_at_1600x900() {
        let layout = Layout::new(1600, 900);
        assert_eq!(layout.menu, Rect::new(0, 0, 1600, 28));
        assert_eq!(layout.ribbon, Rect::new(0, 28, 1600, 96));
        assert_eq!(layout.dock, Rect::new(0, 124, 320, 752));
        assert_eq!(layout.viewport, Rect::new(320, 124, 1280, 752));
        assert_eq!(layout.status, Rect::new(0, 876, 1600, 24));
    }

    #[test]
    fn dock_shrinks_before_the_viewport_disappears() {
        let layout = Layout::new(800, 600);
        assert_eq!(layout.dock.w, 320);
        assert_eq!(layout.viewport.w, 480);
        let layout = Layout::new(250, 300);
        assert_eq!(layout.dock.w, 50);
        assert_eq!(layout.viewport.w, 200);
        let layout = Layout::new(100, 80);
        assert_eq!(layout.dock.w, 0);
        assert_eq!(layout.viewport.w, 100);
        assert!(layout.viewport.h >= 0);
    }

    #[test]
    fn small_window_keeps_rows_inside_the_window() {
        let layout = Layout::new(320, 140);
        assert_eq!(layout.menu.h, 28);
        assert_eq!(layout.ribbon.h, 96);
        assert_eq!(layout.status.h, 16);
        assert_eq!(layout.dock.h, 0);
        assert_eq!(layout.viewport.h, 0);
    }

    #[test]
    fn panel_order_and_headers_follow_the_spec() {
        let layout = Layout::new(1600, 900);
        let panels = layout.panels([false; 4]);
        assert_eq!(panels[0].id, PanelId::Navigator);
        assert_eq!(panels[0].header, Rect::new(0, 124, 320, 26));
        assert_eq!(panels[0].body.h, 340);
        assert_eq!(panels[1].header, Rect::new(0, 490, 320, 26));
        assert_eq!(panels[1].body.h, 84);
        assert_eq!(panels[2].body.h, 128);
        assert_eq!(panels[3].body.h, 96);
        assert_eq!(panels[3].body.bottom(), layout.dock.bottom());
    }

    #[test]
    fn collapsed_panels_keep_only_their_header() {
        let layout = Layout::new(1600, 900);
        let panels = layout.panels([false, true, true, true]);
        assert_eq!(panels[0].body.h, 648);
        assert_eq!(panels[1].body.h, 0);
        assert_eq!(panels[2].body.h, 0);
        assert_eq!(panels[3].body.h, 0);
        assert!(panels[3].header.bottom() <= layout.dock.bottom());
        assert!(panels.iter().filter(|panel| panel.collapsed).count() == 3);
    }

    #[test]
    fn menu_hit_testing_uses_label_cells() {
        let menu = Layout::new(1600, 900).menu;
        let items = menu_items(menu);
        assert_eq!(items.len(), 8);
        assert_eq!(items[0].label, "文件");
        assert_eq!(items[0].english, "File");
        assert_eq!(items[2].english, "Analysis");
        assert_eq!(
            hit(items.iter().map(|item| (item.index, item.rect)), 20, 14),
            Some(0)
        );
        assert_eq!(
            hit(items.iter().map(|item| (item.index, item.rect)), 90, 14),
            Some(1)
        );
        assert_eq!(
            hit(items.iter().map(|item| (item.index, item.rect)), 600, 14),
            None
        );
        assert_eq!(
            hit(items.iter().map(|item| (item.index, item.rect)), 20, 40),
            None
        );
    }

    #[test]
    fn file_menu_rows_are_relative_to_the_menu() {
        let menu = Layout::new(1600, 900).menu;
        let rect = file_menu_rect(menu);
        assert_eq!(rect, Rect::new(8, 28, 240, 390));
        let rows = file_menu_rows(rect);
        assert_eq!(rows.len(), FILE_MENU_ITEMS.len());
        assert!(rows[4].contains(rect.x + 10, rect.y + 4 * MENU_ROW_HEIGHT + 10));
        let prefs = preferences_rect(rect);
        assert_eq!(prefs, Rect::new(248, 28 + 104, 220, 156));
        assert_eq!(preferences_rows(prefs).len(), 6);
    }

    #[test]
    fn ribbon_tiles_stay_inside_the_ribbon_and_do_not_overlap() {
        let area = Layout::new(1600, 900).ribbon;
        let groups = ribbon_groups(area);
        assert_eq!(groups.len(), RIBBON_GROUPS.len());
        let mut previous_right = i32::MIN;
        let mut tile_count = 0;
        for group in &groups {
            assert!(group.rect.x >= previous_right);
            assert!(group.rect.right() <= area.right());
            previous_right = group.rect.right();
            for tile in &group.tiles {
                assert!(tile.rect.x >= group.rect.x);
                assert!(tile.rect.right() <= group.rect.right());
                assert!(tile.rect.bottom() <= group.rect.bottom());
                tile_count += 1;
            }
        }
        assert_eq!(tile_count, 43);
    }

    #[test]
    fn recovered_commands_map_to_ribbon_tiles() {
        let groups = ribbon_groups(Rect::new(0, 0, 1600, 96));
        let mut commands = std::collections::HashSet::new();
        for group in &groups {
            assert!(!group.name.is_empty());
            assert!(!group.english.is_empty());
            for tile in &group.tiles {
                assert!(!tile.label.is_empty());
                assert!(!tile.english.is_empty());
                if tile.command.is_empty() {
                    continue;
                }
                assert!(tile.command.starts_with("UG_") || tile.command.starts_with("NX_"));
                assert!(commands.insert(tile.command), "duplicate {}", tile.command);
            }
        }
        assert_eq!(commands.len(), 42);
        assert_eq!(groups[3].name, "连接副");
        assert_eq!(groups[11].name, "传动机构");
        assert_eq!(groups[11].tiles[0].command, "");
    }

    #[test]
    fn recovered_preference_commands_and_panel_titles_match_the_report() {
        assert_eq!(
            PREFERENCES_COMMANDS,
            [
                Some("UG_MECH_PREFERENCES_MOTION"),
                Some("UG_MECH_PREFERENCES_XYPLOT"),
                Some("UG_PREFERENCES_SOLVER"),
                None,
                None,
                None
            ]
        );
        assert_eq!(PanelId::Navigator.title(), "Motion 导航器");
        assert_eq!(PanelId::Solution.title(), "求解视图");
        assert_eq!(PanelId::XyResult.title(), "XY 结果视图");
        assert_eq!(
            PanelId::FlexBody.title(),
            "Motion 导航器柔性体模式局部放大图"
        );
        assert_eq!(PanelId::Navigator.title_en(), "Motion Navigator");
    }

    #[test]
    fn narrow_ribbon_compresses_tiles() {
        let wide = ribbon_groups(Rect::new(0, 0, 1600, 96));
        let narrow = ribbon_groups(Rect::new(0, 0, 900, 96));
        assert!(wide[0].tiles[0].rect.w > narrow[0].tiles[0].rect.w);
        assert!(narrow[0].tiles[0].rect.w >= 30);
        assert!(wide.last().expect("groups").rect.right() <= 1600);
    }

    #[test]
    fn tree_rows_scroll_and_hit_test() {
        let area = Rect::new(0, 100, 320, 60);
        let nodes: Vec<TreeNode> = (0..10).map(|index| node(index % 3)).collect();
        let rows = tree_rows(area, &nodes, 0);
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].node, 0);
        assert_eq!(rows[0].rect, Rect::new(0, 100, 320, 20));
        assert_eq!(
            hit(rows.iter().map(|row| (row.node, row.rect)), 10, 125),
            Some(1)
        );
        let rows = tree_rows(area, &nodes, 25);
        assert_eq!(rows[0].node, 1);
        assert_eq!(rows[0].rect.y, 95);
        assert_eq!(max_tree_scroll(area, nodes.len()), 140);
        let rows = tree_rows(area, &nodes, 10_000);
        assert_eq!(rows[0].node, 7);
        assert_eq!(
            hit(rows.iter().map(|row| (row.node, row.rect)), 10, 105),
            Some(7)
        );
    }
}
