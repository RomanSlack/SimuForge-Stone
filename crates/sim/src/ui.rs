//! egui-based GUI for SimuForge-Stone.
//!
//! Dark-themed industrial cockpit UI: Palantir data density,
//! Apple restraint, Blender viewport-centric layout.

use egui::{Color32, Frame, Margin, RichText, Stroke};

// ────────────────────────── Color Palette ──────────────────────────
#[allow(dead_code)]
const BG_1: Color32 = Color32::from_rgb(24, 26, 32);
const BG_2: Color32 = Color32::from_rgb(30, 32, 40);
const BG_3: Color32 = Color32::from_rgb(40, 42, 50);

const BORDER_0: Color32 = Color32::from_rgb(42, 45, 53);
const BORDER_1: Color32 = Color32::from_rgb(58, 61, 69);

const TEXT_0: Color32 = Color32::from_rgb(240, 241, 244);
const TEXT_1: Color32 = Color32::from_rgb(176, 180, 188);
const TEXT_2: Color32 = Color32::from_rgb(110, 115, 128);

const ORANGE: Color32 = Color32::from_rgb(245, 124, 32);
const BLUE: Color32 = Color32::from_rgb(74, 138, 229);
const GREEN: Color32 = Color32::from_rgb(61, 204, 96);
const RED: Color32 = Color32::from_rgb(229, 69, 58);
const YELLOW: Color32 = Color32::from_rgb(245, 200, 66);
const CYAN: Color32 = Color32::from_rgb(66, 191, 217);

// Semi-transparent panel fills
fn panel_fill() -> Color32 {
    Color32::from_rgba_unmultiplied(24, 26, 32, 230)
}
fn header_fill() -> Color32 {
    Color32::from_rgba_unmultiplied(30, 32, 40, 240)
}

// ────────────────────────── Data Types ──────────────────────────

/// Read-only snapshot of all data the UI displays (populated each frame).
pub struct UiData {
    pub mode_manual: bool,
    pub paused: bool,
    /// None = no carving session. 0=Idle, 1=Running, 2=Paused, 3=Complete.
    pub carving_state: Option<u8>,
    pub speed_multiplier: f64,
    pub fps: f64,

    pub carving_progress: f32,
    pub carving_done: usize,
    pub carving_total: usize,
    pub carving_eta: f64,

    pub joint_angles: [f64; 6],
    pub joint_mins: [f64; 6],
    pub joint_maxs: [f64; 6],
    pub joint_torques: [f64; 6],

    pub ik_converged: bool,
    pub target_pos: [f64; 3],
    pub tool_pos: [f64; 3],
    pub inside_block: bool,
    pub tracking_error: f64,

    pub spindle_on: bool,
    pub cutting_force: f64,

    pub sim_time: f64,
    pub chunk_count: usize,
    pub total_triangles: usize,

    pub tool_type: &'static str,
    pub tool_radius_mm: f64,
    pub tool_cutting_length_mm: f64,
    pub gcode_file: Option<String>,
    pub gcode_waypoints: usize,
    pub gcode_est_time: f64,
}

/// Actions the UI wants the application to perform.
pub enum UiAction {
    TogglePause,
    SpeedUp,
    SpeedDown,
}

// ────────────────────────── Theme Setup ──────────────────────────

pub fn setup_fonts(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();

    fonts.font_data.insert(
        "inter".into(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/Inter-Regular.ttf"
        ))),
    );
    fonts.font_data.insert(
        "jbmono".into(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/JBMono-Regular.ttf"
        ))),
    );

    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "inter".into());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .insert(0, "jbmono".into());

    ctx.set_fonts(fonts);

    ctx.style_mut(|style| {
        use egui::{FontFamily, FontId, TextStyle};
        style
            .text_styles
            .insert(TextStyle::Heading, FontId::new(18.0, FontFamily::Proportional));
        style
            .text_styles
            .insert(TextStyle::Body, FontId::new(13.0, FontFamily::Proportional));
        style
            .text_styles
            .insert(TextStyle::Button, FontId::new(13.0, FontFamily::Proportional));
        style
            .text_styles
            .insert(TextStyle::Small, FontId::new(11.0, FontFamily::Proportional));
        style
            .text_styles
            .insert(TextStyle::Monospace, FontId::new(13.0, FontFamily::Monospace));
    });
}

pub fn setup_theme(ctx: &egui::Context) {
    ctx.set_visuals(egui::Visuals::dark());

    ctx.style_mut(|style| {
        let v = &mut style.visuals;
        v.panel_fill = panel_fill();
        v.window_fill = Color32::from_rgba_unmultiplied(24, 26, 32, 235);
        v.extreme_bg_color = Color32::from_rgb(13, 14, 18);
        v.faint_bg_color = BG_2;

        // Widgets
        v.widgets.inactive.bg_fill = BG_3;
        v.widgets.inactive.weak_bg_fill = Color32::from_rgb(35, 37, 45);
        v.widgets.inactive.bg_stroke = Stroke::new(1.0, BORDER_1);
        v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT_1);
        v.widgets.inactive.corner_radius = egui::CornerRadius::same(4);

        v.widgets.hovered.bg_fill = Color32::from_rgb(53, 56, 64);
        v.widgets.hovered.weak_bg_fill = Color32::from_rgb(48, 51, 59);
        v.widgets.hovered.bg_stroke = Stroke::new(1.0, BLUE);
        v.widgets.hovered.fg_stroke = Stroke::new(1.0, TEXT_0);
        v.widgets.hovered.corner_radius = egui::CornerRadius::same(4);

        v.widgets.active.bg_fill = Color32::from_rgb(69, 72, 80);
        v.widgets.active.weak_bg_fill = Color32::from_rgb(60, 63, 71);
        v.widgets.active.bg_stroke = Stroke::new(1.0, ORANGE);
        v.widgets.active.fg_stroke = Stroke::new(1.0, TEXT_0);
        v.widgets.active.corner_radius = egui::CornerRadius::same(4);

        v.widgets.open.bg_fill = Color32::from_rgb(53, 56, 64);
        v.widgets.open.weak_bg_fill = Color32::from_rgb(48, 51, 59);
        v.widgets.open.bg_stroke = Stroke::new(1.0, ORANGE);
        v.widgets.open.fg_stroke = Stroke::new(1.0, TEXT_0);
        v.widgets.open.corner_radius = egui::CornerRadius::same(4);

        v.selection.bg_fill = Color32::from_rgba_unmultiplied(74, 138, 229, 80);
        v.selection.stroke = Stroke::new(1.0, BLUE);

        v.window_shadow = egui::epaint::Shadow::NONE;
        v.window_stroke = Stroke::new(1.0, BORDER_0);
        v.window_corner_radius = egui::CornerRadius::same(6);

        style.spacing.item_spacing = egui::vec2(8.0, 4.0);
        style.spacing.window_margin = Margin::same(12);
        style.spacing.button_padding = egui::vec2(8.0, 4.0);
    });
}

// ────────────────────────── Main UI Build ──────────────────────────

pub fn build_ui(ctx: &egui::Context, data: &UiData) -> Vec<UiAction> {
    let mut actions = Vec::new();
    header_panel(ctx, data, &mut actions);
    bottom_panel(ctx, data, &mut actions);
    left_panel(ctx, data);
    right_panel(ctx, data);
    actions
}

// ────────────────────────── Header Bar ──────────────────────────

fn header_panel(ctx: &egui::Context, data: &UiData, _actions: &mut Vec<UiAction>) {
    egui::TopBottomPanel::top("header")
        .exact_height(36.0)
        .frame(
            Frame::new()
                .fill(header_fill())
                .inner_margin(Margin::symmetric(12, 0))
                .stroke(Stroke::new(1.0, BORDER_0)),
        )
        .show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                // App name
                ui.label(RichText::new("SimuForge").size(16.0).color(TEXT_0).strong());
                ui.add_space(4.0);
                ui.label(RichText::new("Stone").size(16.0).color(ORANGE).strong());

                ui.separator();

                // Mode badge
                let (mode_color, mode_text) = if data.mode_manual {
                    (CYAN, "MANUAL")
                } else {
                    (ORANGE, "CARVING")
                };
                let dot_rect = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover()).0;
                ui.painter()
                    .circle_filled(dot_rect.center(), 3.5, mode_color);
                ui.label(RichText::new(mode_text).color(mode_color).size(12.0).strong());

                // Status
                ui.separator();
                let (status_color, status_text) = state_label(data);
                ui.label(RichText::new(status_text).color(status_color).size(12.0));

                // Carving progress (compact)
                if !data.mode_manual && data.carving_total > 0 {
                    ui.separator();
                    let bar_width = 100.0;
                    progress_bar(ui, bar_width, 4.0, data.carving_progress, GREEN);
                    ui.label(
                        RichText::new(format!("{:.0}%", data.carving_progress * 100.0))
                            .monospace()
                            .color(TEXT_1)
                            .size(11.0),
                    );
                }

                // Speed
                if !data.mode_manual {
                    ui.separator();
                    let spd_color =
                        if (data.speed_multiplier - 1.0).abs() < 0.01 { TEXT_2 } else { YELLOW };
                    ui.label(
                        RichText::new(format!("{}x", data.speed_multiplier))
                            .monospace()
                            .color(spd_color)
                            .size(12.0),
                    );
                }

                // FPS (right-aligned)
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let fps_color = if data.fps >= 55.0 {
                        TEXT_2
                    } else if data.fps >= 30.0 {
                        YELLOW
                    } else {
                        RED
                    };
                    ui.label(
                        RichText::new(format!("{:.0} FPS", data.fps))
                            .monospace()
                            .color(fps_color)
                            .size(11.0),
                    );
                });
            });
        });
}

// ────────────────────────── Bottom Panel ──────────────────────────

fn bottom_panel(ctx: &egui::Context, data: &UiData, actions: &mut Vec<UiAction>) {
    egui::TopBottomPanel::bottom("timeline")
        .exact_height(40.0)
        .frame(
            Frame::new()
                .fill(header_fill())
                .inner_margin(Margin::symmetric(12, 0))
                .stroke(Stroke::new(1.0, BORDER_0)),
        )
        .show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                // Play/pause toggle
                let (btn_text, btn_color) = if is_running(data) {
                    ("Pause", YELLOW)
                } else {
                    ("Play", GREEN)
                };
                let btn = ui.add(
                    egui::Button::new(RichText::new(btn_text).color(btn_color).size(12.0))
                        .min_size(egui::vec2(52.0, 24.0)),
                );
                if btn.clicked() {
                    actions.push(UiAction::TogglePause);
                }

                ui.separator();

                // Speed controls
                if ui
                    .add(egui::Button::new(
                        RichText::new(" - ").monospace().color(TEXT_1),
                    ))
                    .clicked()
                {
                    actions.push(UiAction::SpeedDown);
                }
                ui.label(
                    RichText::new(format!("{}x", data.speed_multiplier))
                        .monospace()
                        .color(TEXT_0)
                        .size(12.0),
                );
                if ui
                    .add(egui::Button::new(
                        RichText::new(" + ").monospace().color(TEXT_1),
                    ))
                    .clicked()
                {
                    actions.push(UiAction::SpeedUp);
                }

                ui.separator();

                // Progress bar (wider)
                if !data.mode_manual && data.carving_total > 0 {
                    let available = ui.available_width() - 120.0;
                    progress_bar(ui, available.max(60.0), 6.0, data.carving_progress, GREEN);

                    ui.label(
                        RichText::new(format!(
                            "{}/{}",
                            data.carving_done, data.carving_total
                        ))
                        .monospace()
                        .color(TEXT_2)
                        .size(11.0),
                    );
                }

                // Right side: time/ETA
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if data.carving_eta > 0.0 && data.carving_state == Some(1) {
                        let eta = data.carving_eta / data.speed_multiplier;
                        ui.label(
                            RichText::new(format!("ETA {:.0}s", eta))
                                .monospace()
                                .color(TEXT_2)
                                .size(11.0),
                        );
                    }
                    ui.label(
                        RichText::new(format!("{:.1}s", data.sim_time))
                            .monospace()
                            .color(TEXT_2)
                            .size(11.0),
                    );
                });
            });
        });
}

// ────────────────────────── Left Panel (Arm State) ──────────────────────────

fn left_panel(ctx: &egui::Context, data: &UiData) {
    egui::SidePanel::left("arm_state")
        .exact_width(280.0)
        .resizable(false)
        .frame(
            Frame::new()
                .fill(panel_fill())
                .inner_margin(Margin::same(10))
                .stroke(Stroke::new(1.0, BORDER_0)),
        )
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // ── Joints ──
                egui::CollapsingHeader::new(
                    RichText::new("JOINTS").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    let jc = [ORANGE, ORANGE, ORANGE, CYAN, CYAN, CYAN];
                    for i in 0..6 {
                        let deg = data.joint_angles[i];
                        let min = data.joint_mins[i];
                        let max = data.joint_maxs[i];
                        let range = max - min;

                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("J{}", i + 1))
                                    .color(jc[i])
                                    .size(11.0)
                                    .strong(),
                            );
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    ui.label(
                                        RichText::new(format!("{:>7.1}\u{00B0}", deg))
                                            .monospace()
                                            .color(TEXT_0)
                                            .size(12.0),
                                    );
                                },
                            );
                        });

                        if range > 0.0 {
                            let frac = ((deg - min) / range).clamp(0.0, 1.0) as f32;
                            let zero_frac = ((0.0 - min) / range).clamp(0.0, 1.0) as f32;
                            joint_angle_bar(ui, frac, zero_frac, jc[i]);
                        }
                        ui.add_space(2.0);
                    }
                });

                ui.add_space(4.0);

                // ── Torques ──
                egui::CollapsingHeader::new(
                    RichText::new("TORQUES").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    for i in 0..6 {
                        let torque = data.joint_torques[i];
                        let max_t = if i < 3 { 1200.0 } else { 150.0 };
                        let frac = (torque / max_t).clamp(-1.0, 1.0) as f32;

                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("J{}", i + 1))
                                    .color(TEXT_2)
                                    .size(11.0),
                            );
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    ui.label(
                                        RichText::new(format!("{:>7.1} Nm", torque))
                                            .monospace()
                                            .color(TEXT_1)
                                            .size(11.0),
                                    );
                                },
                            );
                        });

                        torque_bar(ui, frac);
                        ui.add_space(2.0);
                    }
                });

                ui.add_space(4.0);

                // ── Position ──
                egui::CollapsingHeader::new(
                    RichText::new("POSITION").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    // IK status
                    let (ik_color, ik_text) = if data.ik_converged {
                        (GREEN, "IK Converged")
                    } else {
                        (RED, "IK Failed")
                    };
                    status_dot(ui, ik_color, ik_text);

                    ui.add_space(4.0);

                    // Target position
                    ui.label(RichText::new("Target").color(TEXT_2).size(11.0));
                    ui.label(
                        RichText::new(format!(
                            "  X {:>6.1}  Y {:>6.1}  Z {:>6.1}",
                            data.target_pos[0], data.target_pos[1], data.target_pos[2]
                        ))
                        .monospace()
                        .color(TEXT_0)
                        .size(11.0),
                    );

                    ui.add_space(2.0);

                    // Actual tool position
                    ui.label(RichText::new("Actual").color(TEXT_2).size(11.0));
                    ui.label(
                        RichText::new(format!(
                            "  X {:>6.1}  Y {:>6.1}  Z {:>6.1}",
                            data.tool_pos[0], data.tool_pos[1], data.tool_pos[2]
                        ))
                        .monospace()
                        .color(TEXT_1)
                        .size(11.0),
                    );

                    ui.add_space(2.0);

                    // Tracking error
                    ui.label(
                        RichText::new(format!("Error: {:.2} mm", data.tracking_error))
                            .monospace()
                            .color(TEXT_2)
                            .size(11.0),
                    );

                    ui.add_space(4.0);

                    // Inside/outside block
                    let (in_c, in_t) = if data.inside_block {
                        (GREEN, "In Block")
                    } else {
                        (RED, "Outside")
                    };
                    status_dot(ui, in_c, in_t);
                });

                ui.add_space(4.0);

                // ── Cutting ──
                egui::CollapsingHeader::new(
                    RichText::new("CUTTING").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    let (sp_c, sp_t) = if data.spindle_on {
                        (GREEN, "Spindle ON")
                    } else {
                        (RED, "Spindle OFF")
                    };
                    status_dot(ui, sp_c, sp_t);

                    ui.add_space(2.0);

                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Force").color(TEXT_2).size(11.0));
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                ui.label(
                                    RichText::new(format!("{:.1} N", data.cutting_force))
                                        .monospace()
                                        .color(TEXT_1)
                                        .size(11.0),
                                );
                            },
                        );
                    });
                });
            });
        });
}

// ────────────────────────── Right Panel (Properties) ──────────────────────────

fn right_panel(ctx: &egui::Context, data: &UiData) {
    egui::SidePanel::right("properties")
        .exact_width(240.0)
        .resizable(false)
        .frame(
            Frame::new()
                .fill(panel_fill())
                .inner_margin(Margin::same(10))
                .stroke(Stroke::new(1.0, BORDER_0)),
        )
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // ── Simulation ──
                egui::CollapsingHeader::new(
                    RichText::new("SIMULATION").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    prop_row(ui, "Time", &format!("{:.2} s", data.sim_time));
                    prop_row(ui, "Physics", "1000 Hz");
                    prop_row(
                        ui,
                        "Mesh",
                        &format!("{} ch  {}K tri", data.chunk_count, data.total_triangles / 1000),
                    );
                });

                ui.add_space(4.0);

                // ── Workpiece ──
                egui::CollapsingHeader::new(
                    RichText::new("WORKPIECE").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    prop_row(ui, "Material", "Marble");
                    prop_row(ui, "Resolution", "0.5 mm");
                });

                ui.add_space(4.0);

                // ── Tool ──
                egui::CollapsingHeader::new(
                    RichText::new("TOOL").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    prop_row(ui, "Type", data.tool_type);
                    prop_row(ui, "Radius", &format!("{:.1} mm", data.tool_radius_mm));
                    prop_row(
                        ui,
                        "Length",
                        &format!("{:.1} mm", data.tool_cutting_length_mm),
                    );
                });

                ui.add_space(4.0);

                // ── G-Code ──
                egui::CollapsingHeader::new(
                    RichText::new("G-CODE").color(TEXT_1).size(11.0).strong(),
                )
                .default_open(true)
                .show(ui, |ui| {
                    if let Some(path) = &data.gcode_file {
                        // Show just filename
                        let name = std::path::Path::new(path)
                            .file_name()
                            .map(|n| n.to_string_lossy().to_string())
                            .unwrap_or_else(|| path.clone());
                        prop_row(ui, "File", &name);
                        prop_row(ui, "Points", &format!("{}", data.gcode_waypoints));
                        prop_row(ui, "Est.", &format!("{:.0} s", data.gcode_est_time));
                    } else {
                        ui.label(RichText::new("No file loaded").color(TEXT_2).size(11.0));
                        ui.label(
                            RichText::new("Press G to load").color(TEXT_2).size(11.0).italics(),
                        );
                    }
                });

                ui.add_space(8.0);

                // ── Keyboard shortcuts ──
                egui::CollapsingHeader::new(
                    RichText::new("CONTROLS").color(TEXT_2).size(11.0),
                )
                .default_open(false)
                .show(ui, |ui| {
                    shortcut_row(ui, "Space", "Play / Pause");
                    shortcut_row(ui, "WASDQE", "Move tool");
                    shortcut_row(ui, "IJKL", "Rotate tool");
                    shortcut_row(ui, "Shift", "Slow movement");
                    shortcut_row(ui, "X", "Toggle spindle");
                    shortcut_row(ui, "G", "Load G-code");
                    shortcut_row(ui, "R", "Reset target");
                    shortcut_row(ui, "Shift+R", "Full reset");
                    shortcut_row(ui, "+/-", "Speed up/down");
                    shortcut_row(ui, "1/2", "Camera presets");
                    shortcut_row(ui, "M", "Manual mode");
                    shortcut_row(ui, "Esc", "Quit");
                });
            });
        });
}

// ────────────────────────── Helper Widgets ──────────────────────────

/// Joint angle bar with zero-mark.
fn joint_angle_bar(ui: &mut egui::Ui, frac: f32, zero_frac: f32, color: Color32) {
    let width = ui.available_width();
    let height = 4.0;
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let p = ui.painter();

    // Background
    p.rect_filled(rect, 2.0, BG_3);
    // Fill
    if frac > 0.001 {
        let fill = egui::Rect::from_min_size(
            rect.min,
            egui::vec2(rect.width() * frac.clamp(0.0, 1.0), height),
        );
        p.rect_filled(fill, 2.0, color);
    }
    // Zero mark
    let zx = rect.min.x + rect.width() * zero_frac;
    p.line_segment(
        [egui::pos2(zx, rect.min.y), egui::pos2(zx, rect.max.y)],
        Stroke::new(1.0, TEXT_0),
    );
}

/// Centered torque bar (negative = left/red, positive = right/green).
fn torque_bar(ui: &mut egui::Ui, frac: f32) {
    let width = ui.available_width();
    let height = 4.0;
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let p = ui.painter();

    p.rect_filled(rect, 2.0, BG_3);

    let mid_x = rect.center().x;
    let clamped = frac.clamp(-1.0, 1.0);
    if clamped.abs() > 0.001 {
        let color = if clamped >= 0.0 { GREEN } else { RED };
        let fill = if clamped >= 0.0 {
            egui::Rect::from_min_max(
                egui::pos2(mid_x, rect.min.y),
                egui::pos2(mid_x + rect.width() * 0.5 * clamped, rect.max.y),
            )
        } else {
            egui::Rect::from_min_max(
                egui::pos2(mid_x + rect.width() * 0.5 * clamped, rect.min.y),
                egui::pos2(mid_x, rect.max.y),
            )
        };
        p.rect_filled(fill, 2.0, color);
    }
    // Center mark
    p.line_segment(
        [egui::pos2(mid_x, rect.min.y), egui::pos2(mid_x, rect.max.y)],
        Stroke::new(1.0, TEXT_2),
    );
}

/// Compact progress bar.
fn progress_bar(ui: &mut egui::Ui, width: f32, height: f32, frac: f32, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let p = ui.painter();
    p.rect_filled(rect, height * 0.5, BG_3);
    if frac > 0.001 {
        let fill = egui::Rect::from_min_size(
            rect.min,
            egui::vec2(rect.width() * frac.clamp(0.0, 1.0), height),
        );
        p.rect_filled(fill, height * 0.5, color);
    }
}

/// Colored status dot + label.
fn status_dot(ui: &mut egui::Ui, color: Color32, text: &str) {
    ui.horizontal(|ui| {
        let (dot_rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
        ui.painter().circle_filled(dot_rect.center(), 3.0, color);
        ui.label(RichText::new(text).color(color).size(11.0));
    });
}

/// Property row: label on left, value on right (monospace).
fn prop_row(ui: &mut egui::Ui, label: &str, value: &str) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).color(TEXT_2).size(11.0));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(RichText::new(value).monospace().color(TEXT_0).size(11.0));
        });
    });
}

/// Keyboard shortcut row.
fn shortcut_row(ui: &mut egui::Ui, key: &str, desc: &str) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(key).monospace().color(ORANGE).size(10.0));
        ui.label(RichText::new(desc).color(TEXT_2).size(10.0));
    });
}

/// Get state label from data.
fn state_label(data: &UiData) -> (Color32, &'static str) {
    if data.mode_manual {
        if data.paused {
            (YELLOW, "Paused")
        } else {
            (GREEN, "Running")
        }
    } else {
        match data.carving_state {
            Some(0) => (TEXT_2, "Idle"),
            Some(1) => (GREEN, "Running"),
            Some(2) => (YELLOW, "Paused"),
            Some(3) => (CYAN, "Complete"),
            _ => (TEXT_2, "No Session"),
        }
    }
}

/// Check if simulation is actively running.
fn is_running(data: &UiData) -> bool {
    if data.mode_manual {
        !data.paused
    } else {
        data.carving_state == Some(1)
    }
}
