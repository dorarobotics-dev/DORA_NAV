mod config;
mod global_map;
mod tracking;
mod wire;

use anyhow::{Result, anyhow, ensure};
use config::Config;
use dora_node_api::futures::{StreamExt, executor::block_on};
use dora_node_api::{DoraNode, Event, EventStream, MetadataParameters, StopCause, into_vec};
use fast_lio::{
    laser_mapping::LaserMapping,
    types::{AviaMsg, ImuRaw},
};
use global_map::GlobalMap;
use std::{
    collections::VecDeque,
    path::PathBuf,
    time::{Duration, Instant},
};
use tracking::{TrackingMonitor, TrackingState};

struct MappingNode {
    cfg: Config,
    core: LaserMapping,
    map: GlobalMap,
    scans: VecDeque<AviaMsg>,
    imus: VecDeque<ImuRaw>,
    last_imu: Option<f64>,
    last_scan_start: Option<f64>,
    last_scan_end: Option<f64>,
    last_fed_imu: Option<f64>,
    last_imu_arrival: Instant,
    last_scan_arrival: Instant,
    received_scans: usize,
    processed_scans: usize,
    accepted_scans: usize,
    last_output_time: f64,
    tracking: TrackingMonitor,
    last_rejection_reason: &'static str,
}

impl MappingNode {
    fn new(cfg: Config) -> Self {
        let core = LaserMapping::new(&cfg.core());
        let map = GlobalMap::new(cfg.output.voxel_size, cfg.output.max_points);
        let tracking = TrackingMonitor::new(
            cfg.tracking.lost_after_frames,
            Duration::from_secs_f64(cfg.tracking.pose_timeout),
        );
        Self {
            cfg,
            core,
            map,
            scans: VecDeque::new(),
            imus: VecDeque::new(),
            last_imu: None,
            last_scan_start: None,
            last_scan_end: None,
            last_fed_imu: None,
            last_imu_arrival: Instant::now(),
            last_scan_arrival: Instant::now(),
            received_scans: 0,
            processed_scans: 0,
            accepted_scans: 0,
            last_output_time: 0.0,
            tracking,
            last_rejection_reason: "initializing",
        }
    }

    fn input(&mut self, id: &str, bytes: &[u8]) -> Result<()> {
        match id {
            "imu" => {
                let imu = wire::imu(bytes, self.cfg.imu.acceleration_scale)?;
                let stamp = imu.stamp - self.cfg.imu.time_offset_lidar_to_imu;
                ensure!(
                    stamp.is_finite() && stamp > 0.0,
                    "invalid aligned IMU timestamp"
                );
                if let Some(last) = self.last_imu {
                    ensure!(stamp > last, "IMU timestamps must strictly increase");
                    ensure!(
                        stamp - last <= self.cfg.buffers.max_imu_gap,
                        "IMU timestamp gap exceeds buffers.max_imu_gap"
                    );
                }
                ensure!(
                    self.imus.len() < self.cfg.buffers.max_imu_samples,
                    "IMU buffer overflow: LiDAR absent or clock mismatch"
                );
                self.last_imu = Some(stamp);
                self.imus.push_back(imu);
                self.last_imu_arrival = Instant::now();
            }
            "pointcloud_timed" => {
                let scan = wire::scan(bytes, &self.cfg)?;
                let end = scan.stamp + scan.points.last().unwrap().offset_time as f64 * 1e-9;
                if let Some(last) = self.last_scan_start {
                    ensure!(scan.stamp > last, "LiDAR timestamps must strictly increase");
                    ensure!(
                        scan.stamp - last <= self.cfg.buffers.max_scan_gap,
                        "LiDAR timestamp gap exceeds buffers.max_scan_gap"
                    );
                }
                if let Some(last) = self.last_scan_end {
                    ensure!(
                        scan.stamp + 1e-6 >= last,
                        "overlapping LiDAR frames or invalid per-point offsets"
                    );
                }
                ensure!(
                    self.scans.len() < self.cfg.buffers.max_pending_scans,
                    "scan buffer overflow: IMU absent or clock mismatch"
                );
                self.last_scan_start = Some(scan.stamp);
                self.last_scan_end = Some(end);
                self.scans.push_back(scan);
                self.received_scans += 1;
                self.last_scan_arrival = Instant::now();
            }
            _ => return Err(anyhow!("unexpected input {id}")),
        }
        Ok(())
    }

    fn process(&mut self, node: &mut DoraNode) -> Result<()> {
        loop {
            if self.core.pending_counts().0 == 0 {
                let Some(scan) = self.scans.front() else {
                    break;
                };
                if self.last_fed_imu.is_none() {
                    let Some(first) = self.imus.front() else {
                        break;
                    };
                    if first.stamp - self.cfg.imu.time_offset_lidar_to_imu > scan.stamp + 1e-6 {
                        // Startup callbacks need not arrive on the same packet. Discard
                        // only initial scans whose beginning has no IMU coverage.
                        self.scans.pop_front();
                        eprintln!("[fast-lio2] skipped startup scan preceding first IMU sample");
                        continue;
                    }
                }
                let scan = self.scans.pop_front().unwrap();
                self.core.add_lidar_avia(&scan);
            }
            let result =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.core.run_once()))
                    .map_err(|_| anyhow!("FAST-LIO2 core failed while processing frame"))?;
            if self.core.pending_counts().0 != 0 {
                let Some(imu) = self.imus.pop_front() else {
                    break;
                };
                ensure!(
                    self.core.pending_counts().1 < self.cfg.buffers.max_imu_samples,
                    "core IMU buffer overflow: timestamps do not cover pending scan"
                );
                self.last_fed_imu = Some(imu.stamp - self.cfg.imu.time_offset_lidar_to_imu);
                self.core.add_imu(&imu);
                continue;
            }
            self.processed_scans += 1;
            if let Some(result) = result {
                ensure!(
                    result.time.is_finite()
                        && result
                            .pos
                            .iter()
                            .chain(result.vel.iter())
                            .chain(result.bg.iter())
                            .chain(result.ba.iter())
                            .chain(result.quat.iter())
                            .all(|v| v.is_finite())
                        && result.res_mean.is_finite(),
                    "nonfinite FAST-LIO2 state"
                );
                // Initialization/prediction without enough map constraints is not a valid mapping pose.
                if result.effct_feat_num < 5 {
                    self.reject_frame(node, "insufficient_matches")?;
                    continue;
                }
                let world_scan = self.core.registered_scan();
                self.map.insert(&world_scan)?;
                let state = self.core.kf.get_x();
                let pos = state.pos + state.rot * state.offset_t_l_i;
                let (roll, pitch, yaw) = (state.rot * state.offset_r_l_i).euler_angles();
                let pose = [pos.x, pos.y, pos.z, roll, pitch, yaw];
                ensure!(pose.iter().all(|v| v.is_finite()), "nonfinite LiDAR pose");
                let mut bytes = Vec::with_capacity(48);
                for value in pose {
                    bytes.extend_from_slice(&value.to_le_bytes());
                }
                send(node, "pose", bytes)?;
                send(
                    node,
                    "pointcloud",
                    wire::cloud(result.time, self.core.undistorted_scan())?,
                )?;
                self.last_output_time = result.time;
                self.accepted_scans += 1;
                let now = Instant::now();
                let previous_state = self.tracking.state(now);
                self.tracking.record_valid(result.time, now);
                self.last_rejection_reason = "matched";
                let state_changed = previous_state != TrackingState::Mapping;
                if state_changed {
                    self.status(node, None)?;
                }
                if self.accepted_scans % self.cfg.output.publish_every_n_frames == 0 {
                    self.publish_map(node)?;
                    if !state_changed {
                        self.status(node, None)?;
                    }
                }
            } else {
                // A consumed scan can also be rejected before matching (for
                // example, all points were filtered). Bootstrap is not a loss.
                self.reject_frame(node, "no_valid_result")?;
            }
            let (lidar, imu) = self.core.pending_counts();
            ensure!(
                lidar == 0 && imu <= self.cfg.buffers.max_imu_samples,
                "unexpected core buffer growth"
            );
        }
        Ok(())
    }

    fn reject_frame(&mut self, node: &mut DoraNode, reason: &'static str) -> Result<()> {
        self.tracking.record_rejected();
        if self.tracking.has_valid_pose() {
            self.last_rejection_reason = reason;
            self.status(node, None)?;
        }
        Ok(())
    }

    fn status(&self, node: &mut DoraNode, lifecycle: Option<&str>) -> Result<()> {
        let now = Instant::now();
        let tracking_state = self.tracking.state(now);
        let reason = match tracking_state {
            TrackingState::Initializing => "initializing",
            TrackingState::Mapping => "matched",
            TrackingState::TrackingLost
                if self.tracking.consecutive_failures() < self.cfg.tracking.lost_after_frames =>
            {
                "pose_timeout"
            }
            _ => self.last_rejection_reason,
        };
        send(
            node,
            "status",
            serde_json::to_vec(&serde_json::json!({
                "state":lifecycle.unwrap_or(tracking_state.as_str()),"received_scans":self.received_scans,
                "processed_scans":self.processed_scans,"accepted_scans":self.accepted_scans,
                "map_points":self.map.len(),"timestamp":self.last_output_time,
                "pose_frame":"lidar","map_frame":"world",
                "tracking_state":tracking_state.as_str(),
                "pose_valid":self.tracking.pose_valid(now),
                "consecutive_failures":self.tracking.consecutive_failures(),
                "last_valid_pose_timestamp":self.tracking.last_valid_timestamp(),
                "last_valid_pose_age_sec":self.tracking.pose_age(now),
                "reason":reason
            }))?,
        )
    }
    fn publish_map(&self, node: &mut DoraNode) -> Result<()> {
        send(
            node,
            "full_map",
            wire::cloud(self.last_output_time, &self.map.points())?,
        )
    }
    fn run(&mut self, node: &mut DoraNode, events: &mut EventStream) -> Result<bool> {
        let (mut lidar_closed, mut imu_closed) = (false, false);
        self.status(node, None)?;
        // recv() prioritizes InputClosed over buffered sensor packets. Preserve
        // arrival order so natural EOF cannot discard the final scans or errors.
        while let Some(event) = block_on(events.next()) {
            match event {
                Event::Input { id, data, .. } => {
                    if id.as_str() == "tick" {
                        // Fresh sensor packets do not imply a fresh matched pose.
                        self.status(node, None)?;
                        let timeout = self.cfg.buffers.sensor_timeout;
                        ensure!(
                            lidar_closed
                                || self.last_scan_arrival.elapsed().as_secs_f64() < timeout,
                            "LiDAR input timed out"
                        );
                        ensure!(
                            imu_closed || self.last_imu_arrival.elapsed().as_secs_f64() < timeout,
                            "IMU input timed out"
                        );
                    } else {
                        let bytes: Vec<u8> = into_vec(&data).map_err(|e| anyhow!(e.to_string()))?;
                        self.input(id.as_str(), &bytes)?;
                        self.process(node)?;
                    }
                }
                Event::InputClosed { id } => {
                    match id.as_str() {
                        "pointcloud_timed" => lidar_closed = true,
                        "imu" => imu_closed = true,
                        _ => {}
                    }
                    if lidar_closed && imu_closed {
                        self.process(node)?;
                        ensure!(
                            self.scans.is_empty() && self.core.pending_counts().0 == 0,
                            "input ended before IMU covered all LiDAR scans"
                        );
                        return Ok(false);
                    }
                }
                Event::Stop(StopCause::AllInputsClosed) => {
                    self.process(node)?;
                    ensure!(
                        self.scans.is_empty() && self.core.pending_counts().0 == 0,
                        "input ended before IMU covered all LiDAR scans"
                    );
                    return Ok(false);
                }
                Event::Stop(_) => return Ok(true),
                Event::Error(error) => return Err(anyhow!(error)),
                _ => {}
            }
        }
        ensure!(
            self.scans.is_empty() && self.core.pending_counts().0 == 0,
            "Dora event stream ended with unsynchronized scans"
        );
        Ok(false)
    }
}

fn send(node: &mut DoraNode, id: &str, bytes: Vec<u8>) -> Result<()> {
    node.send_output(id.to_string().into(), MetadataParameters::default(), bytes)
        .map_err(|error| anyhow!("publish {id}: {error}"))
}

fn main() -> Result<()> {
    let path = PathBuf::from(
        std::env::var("FAST_LIO2_CONFIG_FILE")
            .unwrap_or_else(|_| "modules/mapping/fast_lio2_dora/config/config.yaml".into()),
    );
    let cfg = Config::load(&path)?;
    let (mut node, mut events) = DoraNode::init_from_env().map_err(|e| anyhow!(e.to_string()))?;
    let mut mapping = MappingNode::new(cfg);
    let run_result = mapping.run(&mut node, &mut events);
    // Save the accumulated map even on input errors, while retaining failure exit status.
    let save_result = mapping.map.save(&mapping.cfg.output.map_path);
    if let Err(error) = &save_result {
        eprintln!("[fast-lio2] {error:#}");
    }
    if let Err(error) = &run_result {
        eprintln!("[fast-lio2] {error:#}");
    }
    let stopped = run_result?;
    save_result?;
    if !stopped {
        mapping.publish_map(&mut node)?;
        mapping.status(&mut node, Some("saved"))?;
    }
    println!(
        "[fast-lio2] saved {} points to {}; accepted {}/{} scans",
        mapping.map.len(),
        mapping.cfg.output.map_path.display(),
        mapping.accepted_scans,
        mapping.received_scans
    );
    Ok(())
}
