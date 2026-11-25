use anyhow::{anyhow, Context, Result};
use dora_node_api::{into_vec, DoraNode, Event, EventStream, MetadataParameters};
use feetech_servo_sdk::FeetechBus;
use serde_json::json;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const ADDR_LOCK: u8 = 55;
const ADDR_MODE: u8 = 33;
const ADDR_TORQUE: u8 = 40;
const ADDR_SPEED: u8 = 46;
const ADDR_PRESENT_SPEED: u8 = 58;
const WHEEL_IDS: [u8; 3] = [9, 8, 7];

fn env_f64(name: &str, fallback: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(fallback)
}

fn env_string(name: &str, fallback: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| fallback.to_string())
}

fn parse_wheel_ids(raw: &str) -> Result<[u8; 3]> {
    let values: Vec<u8> = raw
        .split(',')
        .map(|value| value.trim().parse::<u8>())
        .collect::<std::result::Result<_, _>>()?;
    if values.len() != 3 || values.iter().any(|id| *id >= 254) {
        return Err(anyhow!(
            "WHEEL_IDS must contain three servo IDs from 0 to 253"
        ));
    }
    if values[0] == values[1] || values[0] == values[2] || values[1] == values[2] {
        return Err(anyhow!("WHEEL_IDS must contain three distinct servo IDs"));
    }
    Ok([values[0], values[1], values[2]])
}

fn parse_wheel_angles(raw: &str) -> Result<[f64; 3]> {
    let vals: Vec<f64> = raw
        .split(',')
        .map(|v| v.trim().parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()?;
    if vals.len() != 3 || vals.iter().any(|value| !value.is_finite()) {
        return Err(anyhow!(
            "WHEEL_ANGLES_DEG must contain exactly three finite values"
        ));
    }
    Ok([vals[0], vals[1], vals[2]])
}

#[derive(Clone)]
struct KiwiKinematics {
    angles_deg: [f64; 3],
    linear_ticks_per_mps: f64,
    angular_ticks_per_radps: f64,
}

impl KiwiKinematics {
    fn wheel_speeds(&self, linear: f64, lateral: f64, angular: f64) -> [f64; 3] {
        let mut out = [0.0; 3];
        for (i, angle_deg) in self.angles_deg.iter().enumerate() {
            let a = angle_deg.to_radians();
            let translational = (-a.sin() * linear + a.cos() * lateral) * self.linear_ticks_per_mps;
            let rotational = angular * self.angular_ticks_per_radps;
            out[i] = translational + rotational;
        }
        out
    }

    fn chassis_velocity(&self, wheel_speeds: [f64; 3]) -> Result<[f64; 3]> {
        let mut m = [[0.0; 3]; 3];
        for (r, angle_deg) in self.angles_deg.iter().enumerate() {
            let a = angle_deg.to_radians();
            m[r][0] = -a.sin() * self.linear_ticks_per_mps;
            m[r][1] = a.cos() * self.linear_ticks_per_mps;
            m[r][2] = self.angular_ticks_per_radps;
        }

        if m.iter().flatten().any(|value| !value.is_finite()) {
            return Err(anyhow!("wheel kinematics calibration must be finite"));
        }
        let (a, b, c) = (m[0][0], m[0][1], m[0][2]);
        let (d, e, f) = (m[1][0], m[1][1], m[1][2]);
        let (g, h, i) = (m[2][0], m[2][1], m[2][2]);
        let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if !det.is_finite() || det.abs() < 1e-12 {
            return Err(anyhow!("kinematics matrix is singular"));
        }
        let inv = [
            [
                (e * i - f * h) / det,
                (c * h - b * i) / det,
                (b * f - c * e) / det,
            ],
            [
                (f * g - d * i) / det,
                (a * i - c * g) / det,
                (c * d - a * f) / det,
            ],
            [
                (d * h - e * g) / det,
                (b * g - a * h) / det,
                (a * e - b * d) / det,
            ],
        ];
        Ok([
            inv[0][0] * wheel_speeds[0] + inv[0][1] * wheel_speeds[1] + inv[0][2] * wheel_speeds[2],
            inv[1][0] * wheel_speeds[0] + inv[1][1] * wheel_speeds[1] + inv[1][2] * wheel_speeds[2],
            inv[2][0] * wheel_speeds[0] + inv[2][1] * wheel_speeds[1] + inv[2][2] * wheel_speeds[2],
        ])
    }
}

#[derive(Default)]
struct Odometry {
    x: f64,
    y: f64,
    theta: f64,
    linear: f64,
    lateral: f64,
    angular: f64,
}

impl Odometry {
    fn set_velocity(&mut self, vx: f64, vy: f64, wz: f64) {
        self.linear = vx;
        self.lateral = vy;
        self.angular = wz;
    }

    fn update(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let c = self.theta.cos();
        let s = self.theta.sin();
        self.x += (self.linear * c - self.lateral * s) * dt;
        self.y += (self.linear * s + self.lateral * c) * dt;
        self.theta = (self.theta + self.angular * dt)
            .sin()
            .atan2((self.theta + self.angular * dt).cos());
    }
}

struct WheelPid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: [f64; 3],
    previous_error: [f64; 3],
}

impl WheelPid {
    fn from_env() -> Option<Self> {
        let enabled = std::env::var("PID_ENABLED").unwrap_or_default();
        if enabled != "1" && enabled.to_lowercase() != "true" {
            return None;
        }
        Some(Self {
            kp: env_f64("PID_KP", 0.0),
            ki: env_f64("PID_KI", 0.0),
            kd: env_f64("PID_KD", 0.0),
            integral: [0.0; 3],
            previous_error: [0.0; 3],
        })
    }

    fn apply(&mut self, target: [f64; 3], measured: [f64; 3], dt: f64) -> [f64; 3] {
        if dt <= 0.0 {
            return target;
        }
        let mut out = target;
        for i in 0..3 {
            let error = target[i] - measured[i];
            self.integral[i] = (self.integral[i] + error * dt).clamp(-3000.0, 3000.0);
            let derivative = (error - self.previous_error[i]) / dt;
            self.previous_error[i] = error;
            out[i] =
                target[i] + self.kp * error + self.ki * self.integral[i] + self.kd * derivative;
        }
        out
    }
}

struct FeetechKiwiChassis {
    bus: FeetechBus,
    wheel_ids: [u8; 3],
    kin: KiwiKinematics,
    speed_limit: f64,
    pid: Option<WheelPid>,
    last_target: [f64; 3],
}

impl FeetechKiwiChassis {
    fn new(bus: FeetechBus, kin: KiwiKinematics, speed_limit: f64, wheel_ids: [u8; 3]) -> Self {
        Self {
            bus,
            wheel_ids,
            kin,
            speed_limit,
            pid: WheelPid::from_env(),
            last_target: [0.0; 3],
        }
    }

    async fn init_motors(&mut self) -> Result<()> {
        for id in self.wheel_ids {
            self.set_wheel_speed(id, 0.0).await?;
            self.bus.write_byte(id, ADDR_LOCK, 0).await?;
            tokio::time::sleep(Duration::from_millis(1)).await;
            self.bus.write_byte(id, ADDR_MODE, 1).await?;
            tokio::time::sleep(Duration::from_millis(1)).await;
            self.bus.write_byte(id, ADDR_LOCK, 1).await?;
            tokio::time::sleep(Duration::from_millis(1)).await;
            self.bus.write_byte(id, ADDR_TORQUE, 1).await?;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
        Ok(())
    }

    async fn stop(&mut self) -> Result<()> {
        let mut first_error = None;
        for id in self.wheel_ids {
            if let Err(error) = self.set_wheel_speed(id, 0.0).await {
                if first_error.is_none() {
                    first_error = Some(error.context(format!("failed to stop servo {id}")));
                }
            }
        }
        match first_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    async fn apply_cmd_vel(
        &mut self,
        linear: f64,
        lateral: f64,
        angular: f64,
        dt: f64,
    ) -> Result<()> {
        let mut speeds = self.kin.wheel_speeds(linear, lateral, angular);
        self.last_target = speeds;
        if self.pid.is_some() {
            if let Ok(measured) = self.read_wheel_speeds().await {
                speeds = self.pid.as_mut().unwrap().apply(speeds, measured, dt);
            }
        }
        for (id, speed) in self.wheel_ids.into_iter().zip(speeds) {
            self.set_wheel_speed(id, speed).await?;
        }
        Ok(())
    }

    async fn read_wheel_speeds(&mut self) -> Result<[f64; 3]> {
        let mut speeds = [0.0; 3];
        for (i, id) in self.wheel_ids.into_iter().enumerate() {
            let raw = self.read_word_retry(id, ADDR_PRESENT_SPEED).await?;
            speeds[i] = decode_signed_speed(raw) as f64;
        }
        Ok(speeds)
    }

    async fn read_feedback_velocity(&mut self) -> Result<[f64; 3]> {
        let wheel_speeds = self.read_wheel_speeds().await?;
        self.kin.chassis_velocity(wheel_speeds)
    }

    async fn set_wheel_speed(&mut self, id: u8, speed: f64) -> Result<()> {
        let clamped = speed.clamp(-self.speed_limit, self.speed_limit);
        let magnitude = clamped.abs() as u16;
        let reg = if clamped < 0.0 {
            magnitude | 0x8000
        } else {
            magnitude
        };
        self.write_word_retry(id, ADDR_SPEED, reg).await?;
        Ok(())
    }

    async fn read_word_retry(&mut self, id: u8, addr: u8) -> Result<u16> {
        let mut last_error: Option<anyhow::Error> = None;
        for _ in 0..3 {
            match self.bus.read_word(id, addr).await {
                Ok(value) => return Ok(value),
                Err(e) => {
                    last_error = Some(anyhow!(e));
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }
        }
        Err(last_error.unwrap_or_else(|| anyhow!("read_word failed without error")))
            .with_context(|| format!("read_word retry failed: servo {id} addr {addr}"))
    }

    async fn write_word_retry(&mut self, id: u8, addr: u8, value: u16) -> Result<()> {
        let mut last_error: Option<anyhow::Error> = None;
        for _ in 0..3 {
            match self.bus.write_word(id, addr, value).await {
                Ok(()) => return Ok(()),
                Err(e) => {
                    last_error = Some(anyhow!(e));
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }
        }
        Err(last_error.unwrap_or_else(|| anyhow!("write_word failed without error")))
            .with_context(|| format!("write_word retry failed: servo {id} addr {addr}"))
    }
}

fn decode_signed_speed(raw: u16) -> i32 {
    let magnitude = (raw & 0x7fff) as i32;
    if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

fn parse_cmd_vel(data: &[u8]) -> Result<[f64; 3]> {
    let j: serde_json::Value = serde_json::from_slice(data)?;
    let linear = j.get("linear");
    let angular = j.get("angular");
    let vx = linear
        .and_then(|v| v.get("x"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let vy = linear
        .and_then(|v| v.get("y"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let wz = angular
        .and_then(|v| v.get("z"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    Ok([vx, vy, wz])
}

fn command_timed_out(last_command: Instant, now: Instant, timeout: Duration) -> bool {
    now.duration_since(last_command) > timeout
}

fn command_requires_hardware_write(previous: Option<[f64; 3]>, next: [f64; 3]) -> bool {
    previous.is_none_or(|current| {
        current
            .iter()
            .zip(next.iter())
            .any(|(old, new)| (old - new).abs() > 1e-6)
    })
}

fn send_odometry(node: &mut DoraNode, odom: &Odometry) -> Result<()> {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let half = odom.theta / 2.0;
    let msg = json!({
        "header": {"frame_id": "odom", "stamp": {"sec": now.as_secs(), "nanosec": now.subsec_nanos()}},
        "child_frame_id": "base_link",
        "pose": {"pose": {"position": {"x": odom.x, "y": odom.y, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": half.sin(), "w": half.cos()}}},
        "twist": {"twist": {"linear": {"x": odom.linear, "y": odom.lateral, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": odom.angular}}}
    }).to_string();
    node.send_output(
        "Odometry".into(),
        MetadataParameters::default(),
        msg.into_bytes(),
    )
    .map_err(|error| anyhow!(error.to_string()))
}

fn main() -> Result<()> {
    let (mut node, mut events) =
        DoraNode::init_from_env().map_err(|error| anyhow!(error.to_string()))?;
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    runtime.block_on(run(&mut node, &mut events))
}

async fn run(node: &mut DoraNode, events: &mut EventStream) -> Result<()> {
    let port = env_string("SERIAL_PORT", "/dev/ttyUSB0");
    let default_ids = WHEEL_IDS.map(|id| id.to_string()).join(",");
    let wheel_ids = parse_wheel_ids(&env_string("WHEEL_IDS", &default_ids))?;
    let baud_value = env_f64("SERIAL_BAUD", 1_000_000.0);
    if !baud_value.is_finite()
        || baud_value <= 0.0
        || baud_value > u32::MAX as f64
        || baud_value.fract() != 0.0
    {
        return Err(anyhow!("SERIAL_BAUD must be a positive integer baud rate"));
    }
    let baud = baud_value as u32;
    let speed_limit = env_f64("SPEED_LIMIT", 1500.0);
    if !speed_limit.is_finite() || speed_limit <= 0.0 || speed_limit > 32767.0 {
        return Err(anyhow!("SPEED_LIMIT must be finite and in (0, 32767]"));
    }
    let odom_source = env_string("ODOM_SOURCE", "feedback");
    if !matches!(odom_source.as_str(), "feedback" | "command" | "auto") {
        return Err(anyhow!("ODOM_SOURCE must be feedback, command or auto"));
    }
    let timeout_value = env_f64("CMD_VEL_TIMEOUT_SEC", 0.35);
    if !timeout_value.is_finite() || timeout_value <= 0.0 {
        return Err(anyhow!("CMD_VEL_TIMEOUT_SEC must be positive and finite"));
    }
    let cmd_timeout = Duration::try_from_secs_f64(timeout_value.max(0.05))
        .context("CMD_VEL_TIMEOUT_SEC is out of range")?;
    let kin = KiwiKinematics {
        angles_deg: parse_wheel_angles(&env_string("WHEEL_ANGLES_DEG", "60,180,300"))?,
        linear_ticks_per_mps: env_f64("LINEAR_TICKS_PER_MPS", 13350.0),
        angular_ticks_per_radps: env_f64("ANGULAR_TICKS_PER_RADPS", 1660.0),
    };

    kin.chassis_velocity([0.0; 3])
        .context("invalid wheel kinematics configuration")?;

    println!("Feetech Kiwi SDK Chassis DORA Node");
    println!("serial: {port} @ {baud}");
    if std::env::var("PID_ENABLED").unwrap_or_default() == "1" {
        println!("wheel PID enabled");
    }

    let bus = FeetechBus::new(&port, baud).context("failed to open Feetech bus")?;
    let mut chassis = FeetechKiwiChassis::new(bus, kin, speed_limit, wheel_ids);
    let run_result: Result<()> = async {
        chassis
            .init_motors()
            .await
            .context("failed to initialize motors")?;
        println!("motors initialized");
        let mut odom = Odometry::default();
        let mut last_odom_time = Instant::now();
        let mut last_command_time = Instant::now();
        let mut command_active = false;
        let mut last_hardware_command = None;

        while let Some(event) = events.recv_async().await {
            if let Event::Input { id, data, .. } = event {
                let data: Vec<u8> = if id.as_str() == "tick" {
                    Vec::new()
                } else {
                    into_vec(&data).map_err(|error| anyhow!(error.to_string()))?
                };

                if id.as_str() == "cmd_vel" || id.as_str() == "CmdVelTwist" {
                    match parse_cmd_vel(&data) {
                        Ok(cmd) => {
                            let now = Instant::now();
                            let dt = now.duration_since(last_command_time).as_secs_f64();
                            let applied =
                                if command_requires_hardware_write(last_hardware_command, cmd) {
                                    chassis.apply_cmd_vel(cmd[0], cmd[1], cmd[2], dt).await
                                } else {
                                    Ok(())
                                };
                            match applied {
                                Ok(()) => {
                                    last_hardware_command = Some(cmd);
                                    last_command_time = now;
                                    command_active = cmd.iter().any(|v| v.abs() > 1e-6);
                                    if odom_source == "command" || odom_source == "auto" {
                                        odom.set_velocity(cmd[0], cmd[1], cmd[2]);
                                    }
                                }
                                Err(e) => {
                                    return Err(e.context("failed to apply velocity command"));
                                }
                            }
                        }
                        Err(e) => {
                            eprintln!("failed to parse CmdVelTwist: {e}");
                        }
                    }
                } else if id.as_str() == "tick" {
                    let now = Instant::now();
                    if command_active && command_timed_out(last_command_time, now, cmd_timeout) {
                        eprintln!(
                            "CmdVelTwist timeout after {:.3}s; stopping chassis",
                            cmd_timeout.as_secs_f64()
                        );
                        chassis
                            .stop()
                            .await
                            .context("failed to stop after velocity command timeout")?;
                        last_hardware_command = Some([0.0, 0.0, 0.0]);
                        command_active = false;
                        if odom_source == "command" || odom_source == "auto" {
                            odom.set_velocity(0.0, 0.0, 0.0);
                        }
                    }
                    if odom_source == "feedback" || odom_source == "auto" {
                        match chassis.read_feedback_velocity().await {
                            Ok(v) => odom.set_velocity(v[0], v[1], v[2]),
                            Err(e) if odom_source == "feedback" => {
                                // Missing feedback must not be integrated as a stale speed.
                                odom.set_velocity(0.0, 0.0, 0.0);
                                eprintln!("feedback odom read failed: {e}");
                            }
                            Err(_) => {}
                        }
                    }
                    let dt = now.duration_since(last_odom_time).as_secs_f64();
                    last_odom_time = now;
                    odom.update(dt);
                    send_odometry(node, &odom)?;
                }
            } else if matches!(event, Event::Stop(_)) {
                break;
            } else if let Event::Error(error) = event {
                return Err(anyhow!(error));
            }
        }
        Ok(())
    }
    .await;

    let stop_result = chassis.stop().await;
    if let Err(error) = &stop_result {
        eprintln!("final chassis stop failed: {error:#}");
    }
    run_result?;
    stop_result?;
    println!("Feetech Kiwi SDK chassis node stopped");
    Ok(())
}
