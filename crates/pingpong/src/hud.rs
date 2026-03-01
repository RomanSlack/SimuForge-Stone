//! egui scoreboard, debug overlay, and audio visualization.

use crate::game::{GamePhase, GameState};
use simuforge_audio::AudioEngine;

/// Draw the HUD overlay.
pub fn draw_hud(
    ctx: &egui::Context,
    game: &GameState,
    fps: f64,
    p1_ai: bool,
    p2_ai: bool,
    audio: &AudioEngine,
    show_audio_panel: bool,
) {
    egui::Area::new(egui::Id::new("scoreboard"))
        .fixed_pos(egui::pos2(10.0, 10.0))
        .show(ctx, |ui| {
            ui.visuals_mut().override_text_color = Some(egui::Color32::WHITE);

            // Score
            let p1_label = if p1_ai { "AI" } else { "You" };
            let p2_label = if p2_ai { "AI" } else { "You" };
            ui.label(
                egui::RichText::new(format!(
                    "{} {}  :  {} {}",
                    p1_label, game.score[0], game.score[1], p2_label
                ))
                .size(28.0)
                .strong(),
            );

            // Phase info
            let phase_text = match game.phase {
                GamePhase::WaitingForServe => "Press SPACE to serve",
                GamePhase::Rally => "Rally!",
                GamePhase::PointScored => "Point!",
            };
            ui.label(
                egui::RichText::new(phase_text)
                    .size(16.0)
                    .color(egui::Color32::YELLOW),
            );

            ui.add_space(4.0);
            ui.label(
                egui::RichText::new(format!("{:.0} FPS", fps))
                    .size(13.0)
                    .color(egui::Color32::LIGHT_GRAY),
            );
            ui.label(
                egui::RichText::new("Space: Serve | Tab: P1 AI | R: Reset | P: Pause")
                    .size(11.0)
                    .color(egui::Color32::GRAY),
            );
            ui.label(
                egui::RichText::new("1-4: Camera | LMB: Rotate | MMB: Pan | A: Audio panel")
                    .size(11.0)
                    .color(egui::Color32::GRAY),
            );
        });

    // Audio visualization panel (togglable with 'A' key)
    if show_audio_panel {
        egui::Area::new(egui::Id::new("audio_panel"))
            .fixed_pos(egui::pos2(10.0, 180.0))
            .show(ctx, |ui| {
                ui.visuals_mut().override_text_color = Some(egui::Color32::WHITE);

                ui.label(
                    egui::RichText::new("Audio Engine")
                        .size(16.0)
                        .strong()
                        .color(egui::Color32::from_rgb(100, 200, 255)),
                );

                let enabled_text = if audio.is_enabled() { "Active" } else { "Disabled" };
                let rate = audio.sample_rate();
                ui.label(
                    egui::RichText::new(format!("{} | {:.0} Hz", enabled_text, rate))
                        .size(12.0)
                        .color(egui::Color32::LIGHT_GRAY),
                );

                let voices = audio.voice_snapshot();
                let active = voices.len();
                ui.label(
                    egui::RichText::new(format!("Voices: {}/16", active))
                        .size(12.0)
                        .color(if active > 0 {
                            egui::Color32::from_rgb(100, 255, 100)
                        } else {
                            egui::Color32::GRAY
                        }),
                );

                ui.add_space(4.0);

                // Show each active voice with a mini visualization
                for voice in &voices {
                    let pan_bar = pan_to_bar(voice.pan);
                    let vol_pct = (voice.volume * 100.0) as u32;
                    let color = voice_color(voice.name);
                    ui.label(
                        egui::RichText::new(format!(
                            "{:<6} {} vol:{:>3}% t:{:.0}ms",
                            voice.name,
                            pan_bar,
                            vol_pct,
                            voice.time * 1000.0,
                        ))
                        .size(11.0)
                        .color(color)
                        .family(egui::FontFamily::Monospace),
                    );
                }

                if voices.is_empty() {
                    ui.label(
                        egui::RichText::new("  (no active voices)")
                            .size(11.0)
                            .color(egui::Color32::DARK_GRAY),
                    );
                }

                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new("Synthesis: physics-driven procedural")
                        .size(10.0)
                        .color(egui::Color32::DARK_GRAY),
                );
                ui.label(
                    egui::RichText::new("Ball speed -> intensity, position -> pan")
                        .size(10.0)
                        .color(egui::Color32::DARK_GRAY),
                );
            });
    }
}

/// Convert pan value (-1..1) to a visual stereo position indicator.
fn pan_to_bar(pan: f32) -> String {
    let pos = ((pan + 1.0) * 5.0) as usize; // 0..10
    let pos = pos.min(10);
    let mut bar = ['_'; 11];
    bar[pos] = '|';
    format!("L[{}]R", bar.iter().collect::<String>())
}

/// Color for each voice type.
fn voice_color(name: &str) -> egui::Color32 {
    match name {
        "Table" => egui::Color32::from_rgb(100, 255, 100),  // green
        "Paddle" => egui::Color32::from_rgb(255, 180, 50),  // orange
        "Net" => egui::Color32::from_rgb(200, 200, 255),     // light blue
        "Floor" => egui::Color32::from_rgb(200, 150, 100),   // brown
        _ => egui::Color32::WHITE,
    }
}
