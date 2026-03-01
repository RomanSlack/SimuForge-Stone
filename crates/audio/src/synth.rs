//! Voice trait and procedural synthesis voices.
//!
//! Each voice produces mono samples that are then spatialized by the engine.

use crate::spatial;

/// A synthesis voice that produces audio samples.
pub trait Voice: Send {
    /// Voice type name for display.
    fn name(&self) -> &'static str;

    /// Generate the next sample at the given sample rate.
    /// Returns None when the voice is finished.
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32>;

    /// Whether this voice has finished playing.
    fn is_done(&self) -> bool;

    /// Stereo pan position (-1..1).
    fn pan(&self) -> f32;

    /// Volume (0..1).
    fn volume(&self) -> f32;

    /// Current time into the voice's playback (seconds).
    fn time(&self) -> f32;

    /// Generate a stereo sample pair.
    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| {
            let (l, r) = spatial::stereo_pan(self.pan());
            (s * l * self.volume(), s * r * self.volume())
        })
    }
}

// ---------------------------------------------------------------------------
// Table bounce: damped sine at ~11 kHz + noise burst, 5-10ms core, 50ms tail
// ---------------------------------------------------------------------------

pub struct TableBounceVoice {
    phase: f32,
    time: f32,
    frequency: f32,
    pan_value: f32,
    volume_value: f32,
    // Simple LCG state for noise
    noise_state: u32,
}

impl TableBounceVoice {
    pub fn new(pan: f32, intensity: f32) -> Self {
        Self {
            phase: 0.0,
            time: 0.0,
            frequency: 11000.0 + (intensity - 0.5) * 2000.0, // 10-12 kHz
            pan_value: pan,
            volume_value: (intensity * 0.6).min(1.0),
            noise_state: 12345,
        }
    }

    fn noise(&mut self) -> f32 {
        // Simple LCG pseudo-random
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f32 / 32768.0) - 1.0
    }
}

impl Voice for TableBounceVoice {
    fn name(&self) -> &'static str { "Table" }
    fn time(&self) -> f32 { self.time }
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 0.05 {
            return None; // 50ms total duration
        }

        let dt = 1.0 / sample_rate;
        self.time += dt;

        // Core damped sine (~11 kHz, fast decay)
        self.phase += self.frequency * dt;
        let sine = (self.phase * 2.0 * std::f32::consts::PI).sin();
        let core_env = (-self.time / 0.003).exp(); // 3ms decay constant

        // Noise burst (very short, gives the "click" character)
        let noise = self.noise() * (-self.time / 0.002).exp(); // 2ms noise burst

        // Combine
        let sample = sine * core_env * 0.7 + noise * 0.3;

        Some(sample)
    }

    fn is_done(&self) -> bool {
        self.time > 0.05
    }

    fn pan(&self) -> f32 {
        self.pan_value
    }

    fn volume(&self) -> f32 {
        self.volume_value
    }
}

// ---------------------------------------------------------------------------
// Paddle hit: two damped sines (459 Hz + 3.5 kHz) + bandpass noise, 10-30ms
// ---------------------------------------------------------------------------

pub struct PaddleHitVoice {
    phase_low: f32,
    phase_high: f32,
    time: f32,
    pan_value: f32,
    volume_value: f32,
    noise_state: u32,
    // Simple bandpass state
    bp_prev: f32,
}

impl PaddleHitVoice {
    pub fn new(pan: f32, intensity: f32) -> Self {
        Self {
            phase_low: 0.0,
            phase_high: 0.0,
            time: 0.0,
            pan_value: pan,
            volume_value: (intensity * 0.8).min(1.0),
            noise_state: 67890,
            bp_prev: 0.0,
        }
    }

    fn noise(&mut self) -> f32 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f32 / 32768.0) - 1.0
    }
}

impl Voice for PaddleHitVoice {
    fn name(&self) -> &'static str { "Paddle" }
    fn time(&self) -> f32 { self.time }
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 0.06 {
            return None; // 60ms total
        }

        let dt = 1.0 / sample_rate;
        self.time += dt;

        let tau = std::f32::consts::TAU;

        // Low tone — rubber thud at 459 Hz
        self.phase_low += 459.0 * dt;
        let low = (self.phase_low * tau).sin() * (-self.time / 0.015).exp();

        // High tone — impact crack at 3500 Hz
        self.phase_high += 3500.0 * dt;
        let high = (self.phase_high * tau).sin() * (-self.time / 0.005).exp();

        // Bandpass noise (simple first-order highpass on noise)
        let raw_noise = self.noise();
        let hp_noise = raw_noise - self.bp_prev;
        self.bp_prev = raw_noise * 0.95; // leaky integrator
        let bp_env = (-self.time / 0.008).exp();

        let sample = low * 0.5 + high * 0.3 + hp_noise * bp_env * 0.2;

        Some(sample)
    }

    fn is_done(&self) -> bool {
        self.time > 0.06
    }

    fn pan(&self) -> f32 {
        self.pan_value
    }

    fn volume(&self) -> f32 {
        self.volume_value
    }
}

// ---------------------------------------------------------------------------
// Net hit: low-pass filtered thud at ~200 Hz, 20ms
// ---------------------------------------------------------------------------

pub struct NetHitVoice {
    phase: f32,
    time: f32,
    pan_value: f32,
    volume_value: f32,
    lp_state: f32,
    noise_state: u32,
}

impl NetHitVoice {
    pub fn new(pan: f32) -> Self {
        Self {
            phase: 0.0,
            time: 0.0,
            pan_value: pan,
            volume_value: 0.4,
            lp_state: 0.0,
            noise_state: 11111,
        }
    }

    fn noise(&mut self) -> f32 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f32 / 32768.0) - 1.0
    }
}

impl Voice for NetHitVoice {
    fn name(&self) -> &'static str { "Net" }
    fn time(&self) -> f32 { self.time }
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 0.04 {
            return None;
        }

        let dt = 1.0 / sample_rate;
        self.time += dt;

        // Low sine at 200 Hz
        self.phase += 200.0 * dt;
        let sine = (self.phase * std::f32::consts::TAU).sin();
        let env = (-self.time / 0.012).exp();

        // Low-pass filtered noise for "thud" texture
        let raw = self.noise();
        let alpha = (200.0 * std::f32::consts::TAU * dt).min(1.0); // ~200 Hz cutoff
        self.lp_state += alpha * (raw - self.lp_state);

        let sample = sine * env * 0.6 + self.lp_state * env * 0.4;

        Some(sample)
    }

    fn is_done(&self) -> bool {
        self.time > 0.04
    }

    fn pan(&self) -> f32 {
        self.pan_value
    }

    fn volume(&self) -> f32 {
        self.volume_value
    }
}

// ---------------------------------------------------------------------------
// Floor bounce: mid-tone thud at ~300 Hz, shorter than table
// ---------------------------------------------------------------------------

pub struct FloorBounceVoice {
    phase: f32,
    time: f32,
    pan_value: f32,
    volume_value: f32,
    noise_state: u32,
}

impl FloorBounceVoice {
    pub fn new(pan: f32) -> Self {
        Self {
            phase: 0.0,
            time: 0.0,
            pan_value: pan,
            volume_value: 0.3,
            noise_state: 55555,
        }
    }

    fn noise(&mut self) -> f32 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f32 / 32768.0) - 1.0
    }
}

impl Voice for FloorBounceVoice {
    fn name(&self) -> &'static str { "Floor" }
    fn time(&self) -> f32 { self.time }
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 0.03 {
            return None;
        }

        let dt = 1.0 / sample_rate;
        self.time += dt;

        self.phase += 300.0 * dt;
        let sine = (self.phase * std::f32::consts::TAU).sin();
        let env = (-self.time / 0.008).exp();
        let noise = self.noise() * (-self.time / 0.003).exp();

        let sample = sine * env * 0.6 + noise * 0.4;

        Some(sample)
    }

    fn is_done(&self) -> bool {
        self.time > 0.03
    }

    fn pan(&self) -> f32 {
        self.pan_value
    }

    fn volume(&self) -> f32 {
        self.volume_value
    }
}
