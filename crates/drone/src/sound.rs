//! Drone audio synthesis.
//!
//! Three voice types:
//! - **EngineVoice**: 2-stroke piston engine (3-harmonic sawtooth model + rumble)
//! - **WindVoice**: aerodynamic wind noise (3-band: rumble, vortex, boundary layer)
//! - **BoosterVoice**: one-shot RATO booster roar (3.5s)
//!
//! `SharedVoice<V>` wraps a voice in `Arc<Mutex<V>>` so the main thread can
//! call `update_params()` while the audio thread calls `next_sample()`.

use std::sync::{Arc, Mutex};

use simuforge_audio::synth::Voice;

// ── SharedVoice wrapper ─────────────────────────────────────────────────────

/// Thread-safe voice wrapper. The audio engine holds a `Box<dyn Voice>` which
/// delegates to the inner `Arc<Mutex<V>>`. The main thread also holds a clone
/// of the `Arc` for calling `update_params()` each frame.
pub struct SharedVoice<V: Voice> {
    inner: Arc<Mutex<V>>,
}

impl<V: Voice> SharedVoice<V> {
    pub fn new(inner: Arc<Mutex<V>>) -> Self {
        Self { inner }
    }
}

impl<V: Voice + 'static> Voice for SharedVoice<V> {
    fn name(&self) -> &'static str {
        if let Ok(v) = self.inner.lock() { v.name() } else { "?" }
    }
    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if let Ok(mut v) = self.inner.lock() { v.next_sample(sample_rate) } else { Some(0.0) }
    }
    fn is_done(&self) -> bool {
        if let Ok(v) = self.inner.lock() { v.is_done() } else { true }
    }
    fn pan(&self) -> f32 {
        if let Ok(v) = self.inner.lock() { v.pan() } else { 0.0 }
    }
    fn volume(&self) -> f32 {
        if let Ok(v) = self.inner.lock() { v.volume() } else { 0.0 }
    }
    fn time(&self) -> f32 {
        if let Ok(v) = self.inner.lock() { v.time() } else { 0.0 }
    }
    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        if let Ok(mut v) = self.inner.lock() { v.next_stereo(sample_rate) } else { Some((0.0, 0.0)) }
    }
}

// ── Engine Voice (3-harmonic model) ─────────────────────────────────────────

/// Persistent 2-stroke piston engine sound.
///
/// 3-harmonic sawtooth model:
/// - F0 (intake buzz): 60% at idle → 30% at max RPM
/// - 2·F0 (exhaust): 25% → 40%
/// - 3·F0 (vibration): 15% → 30%
/// Plus low-pass filtered noise for mechanical rumble.
pub struct EngineVoice {
    phase: f64,
    phase2: f64,
    phase3: f64,
    frequency: f64,
    target_freq: f64,
    pub volume: f32,
    pan_value: f32,
    time: f32,
    running: bool,
    envelope: f32,
    noise_state: u32,
    noise_lp: f64,
    freq_lp: f64,
    /// Throttle fraction 0..1 for harmonic mix interpolation.
    throttle: f64,
}

impl EngineVoice {
    pub fn new() -> Self {
        Self {
            phase: 0.0,
            phase2: 0.0,
            phase3: 0.0,
            frequency: 80.0,
            target_freq: 80.0,
            volume: 0.5,
            pan_value: 0.0,
            time: 0.0,
            running: false,
            envelope: 0.0,
            noise_state: 54321,
            noise_lp: 0.0,
            freq_lp: 80.0,
            throttle: 0.0,
        }
    }

    pub fn set_running(&mut self, running: bool) {
        self.running = running;
    }

    /// Update from flight state each frame.
    /// `airspeed` in m/s, `distance` from camera in meters.
    pub fn update_params(&mut self, airspeed: f64, distance: f64, pan: f32) {
        // RPM scales with airspeed: idle at 80 Hz, cruise (51 m/s) at ~105 Hz
        self.target_freq = 80.0 + airspeed * 0.5;
        // Throttle fraction for harmonic mix (0 = idle, 1 = cruise+)
        self.throttle = (airspeed / 51.0).clamp(0.0, 1.0);
        // Volume falls off with distance (inverse-distance with floor)
        self.volume = (2.0 / (1.0 + distance as f32 * 0.005)).clamp(0.01, 0.7);
        self.pan_value = pan;
    }

    fn noise(&mut self) -> f64 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f64 / 32768.0) - 1.0
    }
}

impl Voice for EngineVoice {
    fn name(&self) -> &'static str { "Engine" }

    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        let dt = 1.0 / sample_rate as f64;
        self.time += dt as f32;

        // Envelope: ramp up 0.3s, ramp down 0.5s
        let target_env = if self.running { 1.0_f32 } else { 0.0 };
        let rate = if self.running { 3.0 } else { 2.0 };
        self.envelope += (target_env - self.envelope) * rate * dt as f32;

        if self.envelope < 0.001 && !self.running {
            return Some(0.0);
        }

        // Smooth frequency
        let freq_alpha = (20.0 * dt).min(1.0);
        self.freq_lp += freq_alpha * (self.target_freq - self.freq_lp);
        self.frequency = self.freq_lp;

        let t = self.throttle;

        // F0 sawtooth (intake buzz): amplitude 60% at idle → 30% at max
        self.phase += self.frequency * dt;
        if self.phase >= 1.0 { self.phase -= 1.0; }
        let saw1 = (self.phase * 2.0 - 1.0) as f32;
        let amp1 = (0.60 - 0.30 * t) as f32;

        // 2·F0 sawtooth (exhaust): 25% → 40%
        self.phase2 += self.frequency * 2.0 * dt;
        if self.phase2 >= 1.0 { self.phase2 -= 1.0; }
        let saw2 = (self.phase2 * 2.0 - 1.0) as f32;
        let amp2 = (0.25 + 0.15 * t) as f32;

        // 3·F0 sawtooth (vibration): 15% → 30%
        self.phase3 += self.frequency * 3.0 * dt;
        if self.phase3 >= 1.0 { self.phase3 -= 1.0; }
        let saw3 = (self.phase3 * 2.0 - 1.0) as f32;
        let amp3 = (0.15 + 0.15 * t) as f32;

        // Mechanical rumble (low-pass filtered noise at ~150 Hz)
        let raw = self.noise();
        let alpha = (150.0 * dt * std::f64::consts::TAU).min(1.0);
        self.noise_lp += alpha * (raw - self.noise_lp);
        let rumble = self.noise_lp as f32;

        let sample = saw1 * amp1 + saw2 * amp2 + saw3 * amp3 + rumble * 0.15;

        Some(sample * self.envelope * 0.35)
    }

    fn is_done(&self) -> bool {
        false // always alive — never removed from voice pool
    }

    fn pan(&self) -> f32 { self.pan_value }
    fn volume(&self) -> f32 { self.volume }
    fn time(&self) -> f32 { self.time }

    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| {
            let v = s * self.volume();
            (v, v)
        })
    }
}

// ── Wind Voice (3-band model) ───────────────────────────────────────────────

/// Aerodynamic wind noise — 3-band model:
/// - Low rumble (LP ~200Hz): amplitude ∝ v^2.5, brown noise character
/// - Mid vortex shedding (BP ~500-1500Hz): amplitude ∝ v^4
/// - High boundary layer (HP ~2kHz): amplitude ∝ v^3
///
/// All normalized to cruise speed (51 m/s).
pub struct WindVoice {
    pub volume: f32,
    pan_value: f32,
    time: f32,
    // Three independent noise generators for the bands
    noise_state_lo: u32,
    noise_state_mid: u32,
    noise_state_hi: u32,
    // Filter states
    lp_lo: f64,       // low band LP
    lp_mid1: f64,     // mid band LP (bandpass = LP - HP)
    lp_mid2: f64,     // mid band LP cascade
    hp_mid: f64,      // mid band HP
    hp_hi: f64,       // high band HP
    hp_hi2: f64,      // high band HP cascade
    // Per-band amplitudes (updated from airspeed)
    amp_lo: f64,
    amp_mid: f64,
    amp_hi: f64,
    airspeed: f64,
}

impl WindVoice {
    pub fn new() -> Self {
        Self {
            volume: 0.0,
            pan_value: 0.0,
            time: 0.0,
            noise_state_lo: 98765,
            noise_state_mid: 13579,
            noise_state_hi: 24680,
            lp_lo: 0.0,
            lp_mid1: 0.0,
            lp_mid2: 0.0,
            hp_mid: 0.0,
            hp_hi: 0.0,
            hp_hi2: 0.0,
            amp_lo: 0.0,
            amp_mid: 0.0,
            amp_hi: 0.0,
            airspeed: 0.0,
        }
    }

    /// Update from flight state.
    pub fn update_params(&mut self, airspeed: f64, distance: f64, pan: f32) {
        self.airspeed = airspeed;
        let v_norm = airspeed / 51.0; // normalized to cruise speed

        // Band amplitudes with different power laws
        self.amp_lo = v_norm.powf(2.5).min(2.0);
        self.amp_mid = v_norm.powf(4.0).min(3.0);
        self.amp_hi = v_norm.powf(3.0).min(2.5);

        // Overall volume: distance attenuation
        let dist_factor = (1.5 / (1.0 + distance as f32 * 0.003)).clamp(0.01, 0.6);
        self.volume = dist_factor;
        self.pan_value = pan;
    }

    fn noise(state: &mut u32) -> f64 {
        *state = state.wrapping_mul(1103515245).wrapping_add(12345);
        ((*state >> 16) as f64 / 32768.0) - 1.0
    }
}

impl Voice for WindVoice {
    fn name(&self) -> &'static str { "Wind" }

    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        let dt = 1.0 / sample_rate as f64;
        self.time += dt as f32;

        // Skip when nearly silent
        if self.volume < 0.001 && self.amp_lo < 0.001 {
            return Some(0.0);
        }

        // Low rumble: brown noise (LP filtered at ~200 Hz)
        let raw_lo = Self::noise(&mut self.noise_state_lo);
        let lp_alpha = (200.0 * dt * std::f64::consts::TAU).min(1.0);
        self.lp_lo += lp_alpha * (raw_lo - self.lp_lo);
        let lo = self.lp_lo * self.amp_lo;

        // Mid vortex shedding: bandpass ~500-1500 Hz
        let raw_mid = Self::noise(&mut self.noise_state_mid);
        let lp_mid_alpha = (1500.0 * dt * std::f64::consts::TAU).min(1.0);
        self.lp_mid1 += lp_mid_alpha * (raw_mid - self.lp_mid1);
        self.lp_mid2 += lp_mid_alpha * (self.lp_mid1 - self.lp_mid2);
        let hp_mid_alpha = 1.0 - (500.0 * dt * std::f64::consts::TAU).min(0.99);
        self.hp_mid = hp_mid_alpha * (self.hp_mid + self.lp_mid2 - self.lp_mid1);
        let mid = (self.lp_mid2 * 0.6 + self.hp_mid * 0.4) * self.amp_mid;

        // High boundary layer: HP ~2 kHz
        let raw_hi = Self::noise(&mut self.noise_state_hi);
        let hp_hi_alpha = 1.0 - (2000.0 * dt * std::f64::consts::TAU).min(0.99);
        self.hp_hi = hp_hi_alpha * (self.hp_hi + raw_hi);
        self.hp_hi2 = hp_hi_alpha * (self.hp_hi2 + self.hp_hi);
        let hi = self.hp_hi2 * self.amp_hi;

        let sample = (lo * 0.4 + mid * 0.35 + hi * 0.25) as f32;

        Some(sample * 0.35)
    }

    fn is_done(&self) -> bool {
        false // always alive
    }

    fn pan(&self) -> f32 { self.pan_value }
    fn volume(&self) -> f32 { self.volume }
    fn time(&self) -> f32 { self.time }

    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| {
            let v = s * self.volume();
            (v, v)
        })
    }
}

// ── Booster Roar (one-shot) ──────────────────────────────────────────────────

/// RATO booster ignition — short burst of low-frequency roar.
pub struct BoosterVoice {
    time: f32,
    pan_value: f32,
    noise_state: u32,
    lp: f64,
}

impl BoosterVoice {
    pub fn new(pan: f32) -> Self {
        Self { time: 0.0, pan_value: pan, noise_state: 11111, lp: 0.0 }
    }

    fn noise(&mut self) -> f64 {
        self.noise_state = self.noise_state.wrapping_mul(1103515245).wrapping_add(12345);
        ((self.noise_state >> 16) as f64 / 32768.0) - 1.0
    }
}

impl Voice for BoosterVoice {
    fn name(&self) -> &'static str { "Booster" }

    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 3.5 {
            return None;
        }
        let dt = 1.0 / sample_rate as f64;
        self.time += dt as f32;

        let raw = self.noise();
        // Very low cutoff for deep rumble
        let alpha = (80.0 * dt * std::f64::consts::TAU).min(1.0);
        self.lp += alpha * (raw - self.lp);

        // Envelope: sharp attack, sustain 3s, quick fadeout
        let env = if self.time < 0.1 {
            self.time / 0.1
        } else if self.time < 3.0 {
            1.0
        } else {
            1.0 - (self.time - 3.0) / 0.5
        };

        Some(self.lp as f32 * env.max(0.0) * 0.8)
    }

    fn is_done(&self) -> bool { self.time > 3.5 }
    fn pan(&self) -> f32 { self.pan_value }
    fn volume(&self) -> f32 { 0.9 }
    fn time(&self) -> f32 { self.time }

    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| {
            let v = s * self.volume();
            (v, v)
        })
    }
}

/// Munition impact explosion — multi-layer synthesis:
/// - Shockwave transient (< 2ms pressure spike)
/// - Primary blast wave (20-80 Hz boom, fast decay)
/// - Debris/fragmentation crackle (200-2kHz, medium decay)
/// - Fireball sub-bass rumble (15-40 Hz, slow swell + decay)
/// - Echo/reverb tail (delayed + filtered reflections)
pub struct ImpactVoice {
    time: f64,
    // Independent noise generators per layer
    noise_blast: u32,
    noise_debris: u32,
    noise_fireball: u32,
    noise_echo: u32,
    // Blast wave filters (cascaded 2-pole LP for steep rolloff)
    blast_lp1: f64,
    blast_lp2: f64,
    // Debris band-pass (LP then HP)
    debris_lp1: f64,
    debris_lp2: f64,
    debris_hp: f64,
    // Fireball sub-bass (very low LP)
    fire_lp1: f64,
    fire_lp2: f64,
    fire_lp3: f64,
    // Echo delay line (circular buffer)
    echo_buf: Vec<f64>,
    echo_write: usize,
    echo_lp: f64,
    // DC blocker state
    dc_x1: f64,
    dc_y1: f64,
}

impl ImpactVoice {
    pub fn new() -> Self {
        Self {
            time: 0.0,
            noise_blast: 77777,
            noise_debris: 33333,
            noise_fireball: 55555,
            noise_echo: 99999,
            blast_lp1: 0.0, blast_lp2: 0.0,
            debris_lp1: 0.0, debris_lp2: 0.0, debris_hp: 0.0,
            fire_lp1: 0.0, fire_lp2: 0.0, fire_lp3: 0.0,
            echo_buf: vec![0.0; 22050], // 0.5s at 44.1kHz
            echo_write: 0,
            echo_lp: 0.0,
            dc_x1: 0.0,
            dc_y1: 0.0,
        }
    }

    fn noise(state: &mut u32) -> f64 {
        *state = state.wrapping_mul(1103515245).wrapping_add(12345);
        ((*state >> 16) as f64 / 32768.0) - 1.0
    }

    /// Warm saturation — hard tanh, preserves loudness
    fn saturate(x: f64) -> f64 {
        x.tanh()
    }
}

impl Voice for ImpactVoice {
    fn name(&self) -> &'static str { "Impact" }

    fn next_sample(&mut self, sample_rate: f32) -> Option<f32> {
        if self.time > 4.0 {
            return None;
        }
        let dt = 1.0 / sample_rate as f64;
        let t = self.time;
        self.time += dt;

        let tau = std::f64::consts::TAU;

        // ── Layer 1: Shockwave transient (0-15ms) ──
        // Hard pressure spike — full-bandwidth noise burst that slams speakers
        let shock = if t < 0.015 {
            let shock_raw = Self::noise(&mut self.noise_echo); // reuse a noise gen
            let shock_env = (-t * 200.0).exp(); // 5ms time constant
            shock_raw * shock_env * 4.0
        } else {
            0.0
        };

        // ── Layer 2: Primary blast wave (20-100 Hz) ──
        // White noise → cascaded 2-pole LP, slower decay for chest punch
        let blast_raw = Self::noise(&mut self.noise_blast);
        let blast_cutoff = 100.0 - t * 50.0; // 100→50 Hz over 1s
        let blast_alpha = (blast_cutoff.max(30.0) * dt * tau).min(0.5);
        self.blast_lp1 += blast_alpha * (blast_raw - self.blast_lp1);
        self.blast_lp2 += blast_alpha * (self.blast_lp1 - self.blast_lp2);
        let blast_env = (-t * 3.0).exp(); // ~330ms decay — sustains the thump
        let blast = self.blast_lp2 * blast_env * 8.0;

        // ── Layer 3: Debris / fragmentation crackle (300-3000 Hz) ──
        let debris_raw = Self::noise(&mut self.noise_debris);
        let debris_lp_alpha = (3000.0 * dt * tau).min(0.8);
        self.debris_lp1 += debris_lp_alpha * (debris_raw - self.debris_lp1);
        self.debris_lp2 += debris_lp_alpha * (self.debris_lp1 - self.debris_lp2);
        let debris_hp_alpha = (300.0 * dt * tau).min(0.5);
        self.debris_hp += debris_hp_alpha * (self.debris_lp2 - self.debris_hp);
        let debris_bp = self.debris_lp2 - self.debris_hp;
        // Envelope: sharp onset at 5ms, decays over ~600ms
        let debris_onset = (t / 0.005).min(1.0);
        let debris_decay = (-((t - 0.005).max(0.0)) * 3.0).exp();
        let debris = debris_bp * debris_onset * debris_decay * 3.0;

        // ── Layer 4: Fireball sub-bass rumble (20-50 Hz) ──
        let fire_raw = Self::noise(&mut self.noise_fireball);
        let fire_freq = 35.0 + 15.0 * (-t * 1.5).exp(); // 50→35 Hz
        let fire_alpha = (fire_freq * dt * tau).min(0.3);
        self.fire_lp1 += fire_alpha * (fire_raw - self.fire_lp1);
        self.fire_lp2 += fire_alpha * (self.fire_lp1 - self.fire_lp2);
        self.fire_lp3 += fire_alpha * (self.fire_lp2 - self.fire_lp3);
        // Slow swell, long sustain
        let fire_swell = (t / 0.05).min(1.0);
        let fire_decay = (-((t - 0.2).max(0.0)) * 0.6).exp();
        let fireball = self.fire_lp3 * fire_swell * fire_decay * 6.0;

        // ── Layer 5: Echo / reverb tail ──
        let buf_len = self.echo_buf.len();
        let tap1 = self.echo_buf[(self.echo_write + buf_len - (0.08 * sample_rate as f64) as usize) % buf_len];
        let tap2 = self.echo_buf[(self.echo_write + buf_len - (0.19 * sample_rate as f64) as usize) % buf_len];
        let tap3 = self.echo_buf[(self.echo_write + buf_len - (0.33 * sample_rate as f64) as usize) % buf_len];
        let tap4 = self.echo_buf[(self.echo_write + buf_len - (0.47 * sample_rate as f64) as usize) % buf_len];
        let echo_raw = tap1 * 0.5 + tap2 * 0.35 + tap3 * 0.25 + tap4 * 0.15;
        let echo_alpha = (800.0 * dt * tau).min(0.5);
        self.echo_lp += echo_alpha * (echo_raw - self.echo_lp);
        let echo = self.echo_lp * (-t * 0.5).exp();

        // ── Mix ──
        let dry = shock + blast + debris + fireball;
        let mix = dry + echo;

        // Feed dry signal into echo buffer
        self.echo_buf[self.echo_write] = dry * 0.6;
        self.echo_write = (self.echo_write + 1) % buf_len;

        // Drive hard into saturation for that compressed explosion feel
        let saturated = Self::saturate(mix * 1.8);

        // DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1], R = 0.997
        let dc_r = 0.997;
        let dc_out = saturated - self.dc_x1 + dc_r * self.dc_y1;
        self.dc_x1 = saturated;
        self.dc_y1 = dc_out;

        Some(dc_out as f32)
    }

    fn is_done(&self) -> bool { self.time > 4.0 }
    fn pan(&self) -> f32 { 0.0 }
    fn volume(&self) -> f32 { 1.0 }
    fn time(&self) -> f32 { self.time as f32 }

    fn next_stereo(&mut self, sample_rate: f32) -> Option<(f32, f32)> {
        self.next_sample(sample_rate).map(|s| (s, s))
    }
}
