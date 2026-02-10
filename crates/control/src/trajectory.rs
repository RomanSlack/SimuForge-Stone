//! Trajectory planning with trapezoidal velocity profiles.

use nalgebra::Vector3;

/// A waypoint in Cartesian space.
#[derive(Debug, Clone)]
pub struct Waypoint {
    /// Target position (m).
    pub position: Vector3<f64>,
    /// Feed rate for this segment (m/s).
    pub feed_rate: f64,
}

/// Trapezoidal velocity profile between two points.
#[derive(Debug, Clone)]
pub struct TrapezoidalProfile {
    /// Start position.
    pub start: Vector3<f64>,
    /// End position.
    pub end: Vector3<f64>,
    /// Direction unit vector.
    pub direction: Vector3<f64>,
    /// Total distance (m).
    pub distance: f64,
    /// Peak velocity (m/s).
    pub v_max: f64,
    /// Acceleration (m/s²).
    pub accel: f64,
    /// Time for acceleration phase (s).
    pub t_accel: f64,
    /// Time for cruise phase (s).
    pub t_cruise: f64,
    /// Time for deceleration phase (s).
    pub t_decel: f64,
    /// Total time (s).
    pub total_time: f64,
}

impl TrapezoidalProfile {
    /// Create a trapezoidal profile from start to end with given constraints.
    pub fn new(start: Vector3<f64>, end: Vector3<f64>, v_max: f64, accel: f64) -> Self {
        let diff = end - start;
        let distance = diff.norm();
        let direction = if distance > 1e-9 {
            diff / distance
        } else {
            Vector3::zeros()
        };

        // Check if we can reach full speed
        let d_accel = v_max * v_max / (2.0 * accel);

        let (t_accel, t_cruise, t_decel, actual_v_max);
        if 2.0 * d_accel >= distance {
            // Triangular profile — can't reach full speed
            actual_v_max = (accel * distance).sqrt();
            let ta = actual_v_max / accel;
            t_accel = ta;
            t_cruise = 0.0;
            t_decel = ta;
        } else {
            // Trapezoidal profile
            actual_v_max = v_max;
            t_accel = v_max / accel;
            let d_cruise = distance - 2.0 * d_accel;
            t_cruise = d_cruise / v_max;
            t_decel = v_max / accel;
        }

        let total_time = t_accel + t_cruise + t_decel;

        Self {
            start,
            end,
            direction,
            distance,
            v_max: actual_v_max,
            accel,
            t_accel,
            t_cruise,
            t_decel,
            total_time,
        }
    }

    /// Evaluate position at time t (seconds from start).
    pub fn position_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, self.total_time);
        let s = self.distance_at(t);
        self.start + self.direction * s
    }

    /// Evaluate velocity at time t.
    pub fn velocity_at(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.total_time);
        if t < self.t_accel {
            self.accel * t
        } else if t < self.t_accel + self.t_cruise {
            self.v_max
        } else {
            let t_decel = t - self.t_accel - self.t_cruise;
            (self.v_max - self.accel * t_decel).max(0.0)
        }
    }

    /// Distance traveled at time t.
    fn distance_at(&self, t: f64) -> f64 {
        if t <= self.t_accel {
            0.5 * self.accel * t * t
        } else if t <= self.t_accel + self.t_cruise {
            let d_accel = 0.5 * self.accel * self.t_accel * self.t_accel;
            let dt = t - self.t_accel;
            d_accel + self.v_max * dt
        } else {
            let d_accel = 0.5 * self.accel * self.t_accel * self.t_accel;
            let d_cruise = self.v_max * self.t_cruise;
            let dt = t - self.t_accel - self.t_cruise;
            d_accel + d_cruise + self.v_max * dt - 0.5 * self.accel * dt * dt
        }
    }

    /// Is the profile complete at time t?
    pub fn is_complete(&self, t: f64) -> bool {
        t >= self.total_time
    }
}

/// Trajectory planner: sequences of waypoints with trapezoidal profiles.
pub struct TrajectoryPlanner {
    /// Sequence of motion profiles.
    pub segments: Vec<TrapezoidalProfile>,
    /// Current segment index.
    pub current_segment: usize,
    /// Time elapsed in current segment.
    pub segment_time: f64,
    /// Default acceleration (m/s²).
    pub default_accel: f64,
}

impl TrajectoryPlanner {
    pub fn new(accel: f64) -> Self {
        Self {
            segments: Vec::new(),
            current_segment: 0,
            segment_time: 0.0,
            default_accel: accel,
        }
    }

    /// Plan a trajectory from a list of waypoints.
    pub fn plan(&mut self, waypoints: &[Waypoint]) {
        self.segments.clear();
        self.current_segment = 0;
        self.segment_time = 0.0;

        for pair in waypoints.windows(2) {
            let profile = TrapezoidalProfile::new(
                pair[0].position,
                pair[1].position,
                pair[1].feed_rate,
                self.default_accel,
            );
            self.segments.push(profile);
        }
    }

    /// Step the trajectory forward by dt. Returns current target position.
    pub fn step(&mut self, dt: f64) -> Option<Vector3<f64>> {
        if self.current_segment >= self.segments.len() {
            return None;
        }

        self.segment_time += dt;
        let seg = &self.segments[self.current_segment];

        if seg.is_complete(self.segment_time) {
            self.current_segment += 1;
            self.segment_time = 0.0;
            if self.current_segment >= self.segments.len() {
                return None;
            }
        }

        let seg = &self.segments[self.current_segment];
        Some(seg.position_at(self.segment_time))
    }

    /// Is the trajectory complete?
    pub fn is_complete(&self) -> bool {
        self.current_segment >= self.segments.len()
    }

    /// Total estimated time for the entire trajectory.
    pub fn total_time(&self) -> f64 {
        self.segments.iter().map(|s| s.total_time).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trapezoidal_profile() {
        let start = Vector3::new(0.0, 0.0, 0.0);
        let end = Vector3::new(1.0, 0.0, 0.0);
        let profile = TrapezoidalProfile::new(start, end, 0.5, 2.0);

        // At t=0, should be at start
        let p0 = profile.position_at(0.0);
        assert!((p0 - start).norm() < 1e-10);

        // At t=total, should be at end
        let pf = profile.position_at(profile.total_time);
        assert!(
            (pf - end).norm() < 1e-6,
            "Final position error: {:?} vs {:?}",
            pf,
            end
        );
    }

    #[test]
    fn test_trajectory_planner() {
        let waypoints = vec![
            Waypoint {
                position: Vector3::new(0.0, 0.0, 0.0),
                feed_rate: 0.1,
            },
            Waypoint {
                position: Vector3::new(0.1, 0.0, 0.0),
                feed_rate: 0.1,
            },
            Waypoint {
                position: Vector3::new(0.1, 0.1, 0.0),
                feed_rate: 0.1,
            },
        ];

        let mut planner = TrajectoryPlanner::new(1.0);
        planner.plan(&waypoints);

        assert_eq!(planner.segments.len(), 2);
        assert!(!planner.is_complete());

        // Step until complete
        let mut steps = 0;
        while planner.step(0.001).is_some() {
            steps += 1;
            if steps > 100000 {
                panic!("Trajectory didn't complete");
            }
        }
        assert!(planner.is_complete());
    }
}
