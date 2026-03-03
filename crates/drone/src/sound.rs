//! Drone engine sound voice.
//!
//! Implements the `Voice` trait for a looping engine drone sound:
//! sawtooth + filtered noise at ~80 Hz fundamental.

use simuforge_audio::synth::Voice;
use simuforge_audio::spatial;

/// Drone engine voice — continuous looping sound during flight.
pub struct DroneEngineVoice {
    /// Phase of the sawtooth oscillator (0..1).
    phase: f64,
    /// Fundamental frequency (Hz). Scales with airspeed.
    frequency: f64,
    /// Volume (0..1). Scales with 1/distance from camera.
    volume: f32,
    /// Stereo pan (-1..1).
    pan_value: f32,
    /// Running time (seconds).
    time: f32,
    /// Whether the engine is running.
    active: bool,
    /// Simple LCG noise state.
    noise_state: u32,
    /// Low-pass filter state for noise.
    noise_lp: f64,
}

impl DroneEngineVoice {
    pub fn new() -> Self {
        Self {
            phase: 0.0,
            frequency: 80.0,
            volume: 0.3,
            pan_value: 0.0,
            time: 0.0,
            active: false,
            noise_state: 54321,
            noise_lp: 0.0,
        }
    }

    /// Start the engine sound.
    pub fn start(&mut self) {
        self.active = true;
    }

    /// Stop the engine sound.
    pub fn stop(&mut self) {
        self.active = false;
    }

    /// Update pitch and volume from flight state.
    /// `airspeed`: m/s, `distance`: meters from camera to drone.
    pub fn update(&mut self, airspeed: f64, distance: f64, pan: f32) {
        self.frequency = 80.0 + airspeed * 0.5;
        // Volume decreases with distance, minimum audible
        self.volume = (1.0 / (1.0 + distance as f32 * 0.002)).clamp(0.02, 0.8);
        self.pan_value = pan;
    }

    fn noise(&mut self) -> f64 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f64 / 32768.0) - 1.0
    }
}

impl Voice for DroneEngineVoice {
    fn name(&self) -> &'static str {
        "DroneEngine"
    }

    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if !self.active {
            return None;
        }

        let dt = 1.0 / sample_rate as f64;
        self.time += dt as f32;

        // Sawtooth oscillator
        self.phase += self.frequency * dt;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        let saw = (self.phase * 2.0 - 1.0) as f32;

        // Filtered noise (low-pass at ~200 Hz for engine rumble)
        let raw_noise = self.noise();
        let alpha = (200.0 * dt * std::f64::consts::TAU).min(1.0);
        self.noise_lp += alpha * (raw_noise - self.noise_lp);
        let noise_sample = self.noise_lp as f32;

        // Mix: 60% sawtooth + 40% noise
        let sample = saw * 0.6 + noise_sample * 0.4;

        // Amplitude envelope — slight warmup
        let env = (self.time * 2.0).min(1.0);

        Some(sample * env * 0.3)
    }

    fn is_done(&self) -> bool {
        !self.active
    }

    fn pan(&self) -> f32 {
        self.pan_value
    }

    fn volume(&self) -> f32 {
        self.volume
    }

    fn time(&self) -> f32 {
        self.time
    }

    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| {
            let (l, r) = spatial::stereo_pan(self.pan());
            (s * l * self.volume(), s * r * self.volume())
        })
    }
}
