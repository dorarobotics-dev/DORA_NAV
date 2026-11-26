use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingState {
    Initializing,
    Mapping,
    Degraded,
    TrackingLost,
}

impl TrackingState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Initializing => "initializing",
            Self::Mapping => "mapping",
            Self::Degraded => "degraded",
            Self::TrackingLost => "tracking_lost",
        }
    }
}

/// Matching health, separate from the lifecycle of map saving and node shutdown.
pub struct TrackingMonitor {
    lost_after_frames: usize,
    pose_timeout: Duration,
    consecutive_failures: usize,
    last_valid: Option<(f64, Instant)>,
}

impl TrackingMonitor {
    pub fn new(lost_after_frames: usize, pose_timeout: Duration) -> Self {
        Self {
            lost_after_frames,
            pose_timeout,
            consecutive_failures: 0,
            last_valid: None,
        }
    }

    pub fn record_valid(&mut self, timestamp: f64, now: Instant) {
        self.last_valid = Some((timestamp, now));
        self.consecutive_failures = 0;
    }

    pub fn record_rejected(&mut self) {
        // The normal IMU / initial-map bootstrap does not yield valid poses.
        if self.has_valid_pose() {
            self.consecutive_failures = self.consecutive_failures.saturating_add(1);
        }
    }

    pub fn state(&self, now: Instant) -> TrackingState {
        let Some((_, received)) = self.last_valid else {
            return TrackingState::Initializing;
        };
        if self.consecutive_failures >= self.lost_after_frames
            || now.saturating_duration_since(received) >= self.pose_timeout
        {
            TrackingState::TrackingLost
        } else if self.consecutive_failures > 0 {
            TrackingState::Degraded
        } else {
            TrackingState::Mapping
        }
    }

    pub fn has_valid_pose(&self) -> bool {
        self.last_valid.is_some()
    }
    pub fn consecutive_failures(&self) -> usize {
        self.consecutive_failures
    }
    pub fn last_valid_timestamp(&self) -> Option<f64> {
        self.last_valid.map(|(stamp, _)| stamp)
    }
    pub fn pose_age(&self, now: Instant) -> Option<f64> {
        self.last_valid
            .map(|(_, received)| now.saturating_duration_since(received).as_secs_f64())
    }
    pub fn pose_valid(&self, now: Instant) -> bool {
        self.state(now) == TrackingState::Mapping
    }
}
