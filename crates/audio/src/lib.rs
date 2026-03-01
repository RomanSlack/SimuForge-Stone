//! Procedural audio engine for SimuForge scenarios.
//!
//! Uses cpal for audio output. Voices are mixed in a ring buffer
//! and consumed by the cpal output stream callback.

pub mod spatial;
pub mod synth;

use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use synth::Voice;

/// Maximum number of simultaneous voices.
const MAX_VOICES: usize = 16;

/// Shared state between the main thread (voice triggering) and audio thread (mixing).
struct AudioState {
    voices: Vec<Box<dyn Voice>>,
    sample_rate: f32,
    master_volume: f32,
}

/// Audio engine: manages cpal output stream and voice pool.
pub struct AudioEngine {
    state: Arc<Mutex<AudioState>>,
    _stream: Option<cpal::Stream>,
    enabled: bool,
}

impl AudioEngine {
    /// Create and start the audio engine.
    /// Returns a disabled engine if audio initialization fails (no crash).
    pub fn new() -> Self {
        match Self::try_init() {
            Ok(engine) => engine,
            Err(e) => {
                eprintln!("[audio] Failed to initialize: {e}. Running without audio.");
                Self {
                    state: Arc::new(Mutex::new(AudioState {
                        voices: Vec::new(),
                        sample_rate: 44100.0,
                        master_volume: 0.5,
                    })),
                    _stream: None,
                    enabled: false,
                }
            }
        }
    }

    fn try_init() -> Result<Self, String> {
        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .ok_or("No audio output device found")?;

        let config = device
            .default_output_config()
            .map_err(|e| format!("No default output config: {e}"))?;

        let sample_rate = config.sample_rate().0 as f32;
        let channels = config.channels() as usize;

        let state = Arc::new(Mutex::new(AudioState {
            voices: Vec::new(),
            sample_rate,
            master_volume: 0.5,
        }));

        let state_clone = Arc::clone(&state);

        let stream = match config.sample_format() {
            cpal::SampleFormat::F32 => device
                .build_output_stream(
                    &config.into(),
                    move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                        fill_buffer_f32(data, channels, &state_clone);
                    },
                    |err| eprintln!("[audio] Stream error: {err}"),
                    None,
                )
                .map_err(|e| format!("Failed to build f32 stream: {e}"))?,
            cpal::SampleFormat::I16 => device
                .build_output_stream(
                    &config.into(),
                    move |data: &mut [i16], _: &cpal::OutputCallbackInfo| {
                        fill_buffer_i16(data, channels, &state_clone);
                    },
                    |err| eprintln!("[audio] Stream error: {err}"),
                    None,
                )
                .map_err(|e| format!("Failed to build i16 stream: {e}"))?,
            format => return Err(format!("Unsupported sample format: {format:?}")),
        };

        stream.play().map_err(|e| format!("Failed to play stream: {e}"))?;

        Ok(Self {
            state,
            _stream: Some(stream),
            enabled: true,
        })
    }

    /// Add a voice to the mixer. If at capacity, the oldest voice is dropped.
    pub fn play(&self, voice: Box<dyn Voice>) {
        if !self.enabled {
            return;
        }
        if let Ok(mut state) = self.state.lock() {
            // Remove finished voices first
            state.voices.retain(|v| !v.is_done());
            // Drop oldest if at capacity
            if state.voices.len() >= MAX_VOICES {
                state.voices.remove(0);
            }
            state.voices.push(voice);
        }
    }

    /// Set master volume (0.0 to 1.0).
    pub fn set_volume(&self, volume: f32) {
        if let Ok(mut state) = self.state.lock() {
            state.master_volume = volume.clamp(0.0, 1.0);
        }
    }

    /// Whether the audio engine is active.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get snapshot of active voice info for visualization.
    pub fn voice_snapshot(&self) -> Vec<VoiceInfo> {
        if let Ok(state) = self.state.lock() {
            state
                .voices
                .iter()
                .filter(|v| !v.is_done())
                .map(|v| VoiceInfo {
                    name: v.name(),
                    pan: v.pan(),
                    volume: v.volume(),
                    time: v.time(),
                    done: v.is_done(),
                })
                .collect()
        } else {
            Vec::new()
        }
    }

    /// Get the current number of active voices.
    pub fn active_voice_count(&self) -> usize {
        if let Ok(state) = self.state.lock() {
            state.voices.iter().filter(|v| !v.is_done()).count()
        } else {
            0
        }
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> f32 {
        if let Ok(state) = self.state.lock() {
            state.sample_rate
        } else {
            44100.0
        }
    }
}

/// Snapshot of a single voice for visualization.
#[derive(Debug, Clone)]
pub struct VoiceInfo {
    pub name: &'static str,
    pub pan: f32,
    pub volume: f32,
    pub time: f32,
    pub done: bool,
}

/// Fill an f32 output buffer with mixed audio from all active voices.
fn fill_buffer_f32(data: &mut [f32], channels: usize, state: &Arc<Mutex<AudioState>>) {
    // Zero the buffer
    for s in data.iter_mut() {
        *s = 0.0;
    }

    let Ok(mut state) = state.lock() else {
        return;
    };

    let sample_rate = state.sample_rate;
    let master = state.master_volume;
    let num_frames = data.len() / channels;

    for frame in 0..num_frames {
        let mut left = 0.0_f32;
        let mut right = 0.0_f32;

        for voice in state.voices.iter_mut() {
            if let Some((l, r)) = voice.next_stereo(sample_rate) {
                left += l;
                right += r;
            }
        }

        // Apply master volume and soft clip
        left = soft_clip(left * master);
        right = soft_clip(right * master);

        let base = frame * channels;
        if channels >= 2 {
            data[base] = left;
            data[base + 1] = right;
            // Fill remaining channels with silence
            for ch in 2..channels {
                data[base + ch] = 0.0;
            }
        } else {
            // Mono: mix down
            data[base] = (left + right) * 0.5;
        }
    }

    // Clean up finished voices
    state.voices.retain(|v| !v.is_done());
}

/// Fill an i16 output buffer.
fn fill_buffer_i16(data: &mut [i16], channels: usize, state: &Arc<Mutex<AudioState>>) {
    // Use f32 intermediary
    let mut f32_buf = vec![0.0f32; data.len()];
    fill_buffer_f32(&mut f32_buf, channels, state);

    for (out, &sample) in data.iter_mut().zip(f32_buf.iter()) {
        *out = (sample * 32767.0).clamp(-32768.0, 32767.0) as i16;
    }
}

/// Soft clipping (tanh-like) to prevent harsh distortion.
fn soft_clip(x: f32) -> f32 {
    if x.abs() < 0.5 {
        x
    } else {
        x.signum() * (1.0 - (-2.0 * (x.abs() - 0.5)).exp() * 0.5)
    }
}
