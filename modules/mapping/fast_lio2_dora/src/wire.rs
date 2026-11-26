use crate::config::Config;
use anyhow::{Context, Result, ensure};
use fast_lio::types::{AviaMsg, AviaPointMsg, ImuRaw, PointType};
use serde::Deserialize;

pub fn scan(data: &[u8], cfg: &Config) -> Result<AviaMsg> {
    ensure!(
        data.len() >= 24 && &data[..8] == b"DLIOPC01",
        "expected timed pointcloud DLIOPC01, not legacy XYZI"
    );
    let start_ns = u64::from_le_bytes(data[8..16].try_into()?);
    let count = u32::from_le_bytes(data[16..20].try_into()?) as usize;
    ensure!(data[20..24] == [0; 4], "unsupported timed pointcloud flags");
    ensure!(
        count >= 6 && count <= cfg.buffers.max_points_per_scan,
        "invalid point count {count}"
    );
    ensure!(
        data.len() == 24 + count * 24,
        "timed pointcloud length mismatch"
    );
    ensure!(start_ns > 0, "frame timestamp must be positive");
    let mut points = Vec::with_capacity(count);
    for record in data[24..].chunks_exact(24) {
        let f = |i| f32::from_le_bytes(record[i..i + 4].try_into().unwrap());
        let (x, y, z, intensity) = (f(0), f(4), f(8), f(12));
        ensure!(
            [x, y, z, intensity].iter().all(|v| v.is_finite()),
            "nonfinite point"
        );
        ensure!(
            [x, y, z].iter().all(|v| v.abs() <= 10_000.0),
            "point coordinate exceeds supported range"
        );
        ensure!(
            (0.0..=255.0).contains(&intensity) && intensity.fract() == 0.0,
            "invalid Livox reflectivity"
        );
        let offset_time = u32::from_le_bytes(record[16..20].try_into()?);
        let line = u16::from_le_bytes(record[20..22].try_into()?) as usize;
        ensure!(
            line < cfg.lidar.n_scans && line <= u8::MAX as usize,
            "point line exceeds lidar.n_scans"
        );
        ensure!(record[23] == 0, "unsupported point reserved field");
        ensure!(
            (offset_time as f64) * 1e-9 <= cfg.buffers.max_scan_duration,
            "scan duration exceeds limit"
        );
        points.push(AviaPointMsg {
            x,
            y,
            z,
            reflectivity: intensity as u8,
            offset_time,
            line: line as u8,
            tag: record[22] as u16,
        });
    }
    // The core determines the scan end from the final point's offset.
    points.sort_by_key(|p| p.offset_time);
    ensure!(
        points.last().unwrap().offset_time > 0,
        "scan needs real per-point time offsets"
    );
    ensure!(
        start_ns
            .checked_add(points.last().unwrap().offset_time as u64)
            .is_some(),
        "timestamp overflow"
    );
    Ok(AviaMsg {
        stamp: start_ns as f64 * 1e-9,
        points,
    })
}

#[derive(Deserialize)]
struct Header {
    timestamp: f64,
}
#[derive(Deserialize)]
struct Xyz {
    x: f64,
    y: f64,
    z: f64,
}
#[derive(Deserialize)]
struct ImuJson {
    header: Header,
    linear_acceleration: Xyz,
    angular_velocity: Xyz,
}

pub fn imu(data: &[u8], acceleration_scale: f64) -> Result<ImuRaw> {
    let msg: ImuJson = serde_json::from_slice(data).context("invalid IMU JSON")?;
    let acc = [
        msg.linear_acceleration.x * acceleration_scale,
        msg.linear_acceleration.y * acceleration_scale,
        msg.linear_acceleration.z * acceleration_scale,
    ];
    let gyr = [
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    ];
    ensure!(
        msg.header.timestamp.is_finite() && msg.header.timestamp > 0.0,
        "invalid IMU timestamp"
    );
    ensure!(
        acc.iter().all(|v| v.is_finite() && v.abs() <= 100_000.0)
            && gyr.iter().all(|v| v.is_finite() && v.abs() <= 1000.0),
        "invalid IMU measurement"
    );
    Ok(ImuRaw {
        stamp: msg.header.timestamp,
        acc,
        gyr,
    })
}

/// Existing DORA_NAV point cloud wire contract, little-endian ts/count/XYZI.
pub fn cloud(stamp: f64, points: &[PointType]) -> Result<Vec<u8>> {
    ensure!(
        stamp.is_finite() && points.len() <= u32::MAX as usize,
        "invalid output cloud"
    );
    let mut bytes = Vec::with_capacity(12 + points.len() * 16);
    bytes.extend_from_slice(&stamp.to_le_bytes());
    bytes.extend_from_slice(&(points.len() as u32).to_le_bytes());
    for p in points {
        for v in [p.x, p.y, p.z, p.intensity] {
            ensure!(v.is_finite(), "nonfinite output point");
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    Ok(bytes)
}
