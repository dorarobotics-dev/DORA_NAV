use anyhow::{Context, Result, ensure};
use fast_lio::laser_mapping::LioConfig;
use fast_lio::types::{LidarType, TimeUnit};
use serde::Deserialize;
use std::path::{Path, PathBuf};

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub lidar: Lidar,
    pub imu: Imu,
    pub mapping: Mapping,
    pub extrinsics: Extrinsics,
    pub output: Output,
    pub buffers: Buffers,
    pub tracking: Tracking,
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Lidar {
    pub n_scans: usize,
    pub scan_rate: i32,
    pub blind: f64,
    pub point_filter_num: i32,
}
impl Default for Lidar {
    fn default() -> Self {
        Self {
            n_scans: 4,
            scan_rate: 10,
            blind: 0.5,
            point_filter_num: 2,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Imu {
    pub acceleration_scale: f64,
    /// Positive value is subtracted from IMU timestamps to align to LiDAR time.
    pub time_offset_lidar_to_imu: f64,
    pub gyr_cov: f64,
    pub acc_cov: f64,
    pub b_gyr_cov: f64,
    pub b_acc_cov: f64,
}
impl Default for Imu {
    fn default() -> Self {
        Self {
            acceleration_scale: 9.80665,
            time_offset_lidar_to_imu: 0.0,
            gyr_cov: 0.1,
            acc_cov: 0.1,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Mapping {
    pub filter_size_surf: f32,
    pub filter_size_map: f32,
    pub cube_len: f64,
    pub det_range: f32,
    pub max_iteration: usize,
    pub extrinsic_est_en: bool,
}
impl Default for Mapping {
    fn default() -> Self {
        Self {
            filter_size_surf: 0.3,
            filter_size_map: 0.3,
            cube_len: 300.0,
            det_range: 80.0,
            max_iteration: 4,
            extrinsic_est_en: false,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Extrinsics {
    /// p_imu = R_imu_lidar * p_lidar + t_imu_lidar.
    pub translation: [f64; 3],
    /// Row-major rotation matrix.
    pub rotation: [f64; 9],
}
impl Default for Extrinsics {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Output {
    pub map_path: PathBuf,
    pub voxel_size: f32,
    pub publish_every_n_frames: usize,
    pub max_points: usize,
}
impl Default for Output {
    fn default() -> Self {
        Self {
            map_path: "../../../../maps/fast_lio2/map.pcd".into(),
            voxel_size: 0.2,
            publish_every_n_frames: 10,
            max_points: 2_000_000,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Buffers {
    pub max_imu_samples: usize,
    pub max_pending_scans: usize,
    pub max_scan_duration: f64,
    pub max_points_per_scan: usize,
    pub max_imu_gap: f64,
    pub max_scan_gap: f64,
    pub sensor_timeout: f64,
}
impl Default for Buffers {
    fn default() -> Self {
        Self {
            max_imu_samples: 4000,
            max_pending_scans: 32,
            max_scan_duration: 0.5,
            max_points_per_scan: 200_000,
            max_imu_gap: 0.1,
            max_scan_gap: 1.0,
            sensor_timeout: 10.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Tracking {
    pub lost_after_frames: usize,
    pub pose_timeout: f64,
}
impl Default for Tracking {
    fn default() -> Self {
        Self {
            lost_after_frames: 5,
            pose_timeout: 1.0,
        }
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let mut cfg: Self = serde_yaml::from_str(
            &std::fs::read_to_string(path)
                .with_context(|| format!("read configuration {}", path.display()))?,
        )
        .context("parse FAST-LIO2 configuration")?;
        cfg.validate()?;
        if cfg.output.map_path.is_relative() {
            cfg.output.map_path = path
                .parent()
                .unwrap_or(Path::new("."))
                .join(&cfg.output.map_path);
        }
        Ok(cfg)
    }

    pub fn validate(&self) -> Result<()> {
        ensure!(
            (1..=128).contains(&self.lidar.n_scans),
            "lidar.n_scans must be 1..128"
        );
        ensure!(
            (1..=100).contains(&self.lidar.scan_rate),
            "lidar.scan_rate must be 1..100"
        );
        ensure!(
            (1..=1000).contains(&self.lidar.point_filter_num),
            "lidar.point_filter_num must be 1..1000"
        );
        positive("lidar.blind", self.lidar.blind)?;
        ensure!(
            self.tracking.lost_after_frames > 0,
            "tracking.lost_after_frames must be positive"
        );
        positive("tracking.pose_timeout", self.tracking.pose_timeout)?;
        let pose_timeout = std::time::Duration::try_from_secs_f64(self.tracking.pose_timeout)
            .context("tracking.pose_timeout is outside the supported duration range")?;
        ensure!(
            !pose_timeout.is_zero(),
            "tracking.pose_timeout must be at least one nanosecond"
        );
        for (name, value) in [
            ("imu.acceleration_scale", self.imu.acceleration_scale),
            ("imu.gyr_cov", self.imu.gyr_cov),
            ("imu.acc_cov", self.imu.acc_cov),
            ("imu.b_gyr_cov", self.imu.b_gyr_cov),
            ("imu.b_acc_cov", self.imu.b_acc_cov),
            (
                "mapping.filter_size_surf",
                self.mapping.filter_size_surf as f64,
            ),
            (
                "mapping.filter_size_map",
                self.mapping.filter_size_map as f64,
            ),
            ("mapping.cube_len", self.mapping.cube_len),
            ("mapping.det_range", self.mapping.det_range as f64),
            ("output.voxel_size", self.output.voxel_size as f64),
            ("buffers.max_scan_duration", self.buffers.max_scan_duration),
            ("buffers.max_imu_gap", self.buffers.max_imu_gap),
            ("buffers.max_scan_gap", self.buffers.max_scan_gap),
            ("buffers.sensor_timeout", self.buffers.sensor_timeout),
        ] {
            positive(name, value)?;
        }
        ensure!(
            self.imu.time_offset_lidar_to_imu.is_finite(),
            "IMU time offset must be finite"
        );
        ensure!(
            self.mapping.cube_len > 3.0 * self.mapping.det_range as f64,
            "mapping.cube_len must exceed 3 * det_range for the sliding map"
        );
        ensure!(
            (1..=20).contains(&self.mapping.max_iteration),
            "max_iteration must be 1..20"
        );
        ensure!(
            !self.output.map_path.as_os_str().is_empty(),
            "output.map_path must not be empty"
        );
        ensure!(
            self.output.publish_every_n_frames > 0
                && self.output.max_points > 0
                && self.output.max_points <= u32::MAX as usize,
            "invalid output map limits"
        );
        ensure!(
            self.buffers.max_imu_samples >= 2
                && self.buffers.max_pending_scans > 0
                && self.buffers.max_points_per_scan >= 6
                && self.buffers.max_scan_duration <= 1.0,
            "invalid input buffer limits"
        );
        ensure!(
            self.extrinsics
                .translation
                .iter()
                .chain(self.extrinsics.rotation.iter())
                .all(|v| v.is_finite()),
            "extrinsics must be finite"
        );
        let r = &self.extrinsics.rotation;
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3).map(|k| r[3 * i + k] * r[3 * j + k]).sum();
                ensure!(
                    (dot - if i == j { 1.0 } else { 0.0 }).abs() < 1e-5,
                    "extrinsics.rotation must be orthonormal"
                );
            }
        }
        let det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6])
            + r[2] * (r[3] * r[7] - r[4] * r[6]);
        ensure!(
            (det - 1.0).abs() < 1e-5,
            "extrinsics.rotation must have determinant +1"
        );
        Ok(())
    }

    pub fn core(&self) -> LioConfig {
        LioConfig {
            lidar_type: LidarType::Avia,
            feature_extract_enable: false,
            n_scans: self.lidar.n_scans,
            scan_rate: self.lidar.scan_rate,
            blind: self.lidar.blind,
            point_filter_num: self.lidar.point_filter_num,
            timestamp_unit: TimeUnit::Ns,
            filter_size_surf: self.mapping.filter_size_surf,
            filter_size_map: self.mapping.filter_size_map,
            cube_len: self.mapping.cube_len,
            det_range: self.mapping.det_range,
            max_iteration: self.mapping.max_iteration,
            gyr_cov: self.imu.gyr_cov,
            acc_cov: self.imu.acc_cov,
            b_gyr_cov: self.imu.b_gyr_cov,
            b_acc_cov: self.imu.b_acc_cov,
            extrinsic_est_en: self.mapping.extrinsic_est_en,
            time_sync_en: false,
            time_offset_lidar_to_imu: self.imu.time_offset_lidar_to_imu,
            extrinsic_t: self.extrinsics.translation,
            extrinsic_r: self.extrinsics.rotation,
            ..LioConfig::default()
        }
    }
}

fn positive(name: &str, value: f64) -> Result<()> {
    ensure!(
        value.is_finite() && value > 0.0,
        "{name} must be finite and positive"
    );
    Ok(())
}
