import numpy as np
import pyarrow as pa
import pytest

from mujoco_mickrobot.config import load_config
from mujoco_mickrobot.contract import (
    FRAME_NAMES, HeaderSource, UnknownFrame, cloud_array, pose_array, stamp_ns,
)
from mujoco_mickrobot.messages import encode_imu, encode_odometry, encode_pointcloud
from mujoco_mickrobot.runtime import OutputSpec, SensorBinding, SimulationRuntime
from mujoco_mickrobot.sensors.imu import ImuSensor
from mujoco_mickrobot.sensors.odometry import WheelOdometry
from mujoco_mickrobot.simulator import SimulationSnapshot
from mujoco_mickrobot.vehicle import VehicleController


def test_stamp_is_exact_and_positive():
    assert stamp_ns(0.0) == 1
    assert stamp_ns(0.1) == 100_000_000
    assert stamp_ns(1e-9) == 1
    # 123.456 is 123.45600000000000306... as a double; exact arithmetic, not float scaling
    assert stamp_ns(123.456) == 123_456_000_000


def test_stamp_converts_an_epoch_time_and_keeps_adjacent_doubles_apart():
    import math
    assert stamp_ns(1729000000.5) == 1_729_000_000_500_000_000
    x, stamps = 1729000000.0, []
    for _ in range(60):
        stamps.append(stamp_ns(x))
        x = math.nextafter(x, math.inf)
    assert len(set(stamps)) == len(stamps)


def test_stamp_refuses_a_non_finite_time():
    # Sim time starts at 0 and is clamped to the first legal stamp (1 ns);
    # a time that is not a finite number never becomes a stamp.
    with pytest.raises(ValueError):
        stamp_ns(float("nan"))
    with pytest.raises(OverflowError):
        stamp_ns(float("inf"))


def test_sim_frames_map_to_dictionary_names():
    assert FRAME_NAMES["lidar_link"] == "rslidar"
    assert FRAME_NAMES["camera_link"] == "camera_front"
    assert FRAME_NAMES["imu_link"] == "imu_link"


def test_header_source_counts_per_output_and_maps_frames():
    h = HeaderSource(source="mujoco_mickrobot")
    a = h.header("cloud", "lidar_link", 0.1)
    b = h.header("cloud", "lidar_link", 0.2)
    c = h.header("imu", "imu_link", 0.2)
    assert (a["frame_id"], a["header_seq"], b["header_seq"], c["header_seq"]) == ("rslidar", 0, 1, 0)
    assert a["validity"] == "ok" and "reason" not in a and a["source"] == "mujoco_mickrobot"


def test_unknown_sim_frame_is_refused():
    with pytest.raises(UnknownFrame, match=r"'somewhere' \(output x\).*known sim frames"):
        HeaderSource(source="s").header("x", "somewhere", 0.1)


def test_strip_switch_removes_frame_id_only_on_cloud():
    h = HeaderSource(source="s", strip_frame_on=("cloud",))
    assert "frame_id" not in h.header("cloud", "lidar_link", 0.1)
    assert "frame_id" in h.header("imu", "imu_link", 0.1)


def test_cloud_array_layout():
    pts = np.array([[1, 2, 3, 4]], dtype=np.float32)
    a = cloud_array(pts)
    row = a[0].as_py()
    assert row["width"] == 1 and row["height"] == 1
    assert np.array_equal(np.frombuffer(row["points"], dtype="<f4"), [1, 2, 3, 4])


def test_pose_array_reads_xyzw():
    a = pose_array((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert a[0].as_py()["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


# --- runtime publish path with a fake adapter -------------------------------

class FakeAdapter:
    def __init__(self):
        self.sent = []  # (kind, output_id, payload, metadata)

    def poll(self, timeout_s):
        return None

    def send(self, output_id, payload, metadata):
        self.sent.append(("send", output_id, payload, metadata))

    def send_array(self, output_id, array, metadata):
        self.sent.append(("send_array", output_id, array, metadata))


class FakeSimulator:
    def __init__(self):
        self.time_s = 0.0

    def step(self, targets):
        self.time_s += 0.01

    def snapshot(self):
        return SimulationSnapshot.stationary()

    def viewer_is_running(self):
        return True

    def sync_viewer(self):
        pass

    def close(self):
        pass


def _run(strip=()):
    config = load_config(None)
    odometry = WheelOdometry(config.odometry, config.vehicle, deterministic=True, seed=1)
    imu = ImuSensor(config.imu, deterministic=True, seed=2)
    cloud = np.array([[1, 2, 3, 4]], dtype=np.float32)

    bindings = (
        SensorBinding("lidar", 10.0, lambda s, t: cloud, True, (
            OutputSpec("pointcloud", config.lidar.frame_id, encode_pointcloud),
            OutputSpec("cloud", config.lidar.frame_id, lambda t, p: (cloud_array(p), {}), typed=True),
        )),
        SensorBinding("imu", 50.0, imu.sample, False, (
            OutputSpec("imu", config.imu.frame_id, lambda t, sample: encode_imu(sample)),
        )),
        SensorBinding("odometry", 20.0, odometry.sample, False, (
            OutputSpec("odom_pose", config.odometry.frame_id,
                       lambda t, sample: (pose_array(sample.position_m, sample.orientation_xyzw),
                                          {"child_frame": FRAME_NAMES[sample.child_frame_id]}),
                       typed=True),
        )),
    )
    adapter = FakeAdapter()
    headers = HeaderSource(source="sim", strip_frame_on=strip)
    runtime = SimulationRuntime(config, FakeSimulator(), adapter, VehicleController(config.vehicle),
                                bindings, headers=headers, sleep=lambda s: None)
    runtime.run_until(0.5)
    return adapter.sent


def _by_output(sent):
    out = {}
    for kind, oid, payload, meta in sent:
        out.setdefault(oid, []).append((kind, payload, meta))
    return out


def test_runtime_publish_path_headers_and_typed_outputs():
    out = _by_output(_run())
    # legacy outputs: bytes via send, header plus the kept timestamp key
    for oid, frame in (("pointcloud", "rslidar"), ("imu", "imu_link")):
        kind, payload, meta = out[oid][0]
        assert kind == "send" and isinstance(payload, bytes)
        assert meta["frame_id"] == frame and meta["header_seq"] == 0
        assert isinstance(meta["timestamp"], float) and meta["validity"] == "ok"
        assert meta["stamp_ns"] > 0
    assert [m["header_seq"] for _, _, m in out["imu"][:3]] == [0, 1, 2]
    # typed cloud: PointCloud struct array, no extra keys beyond the header
    kind, array, meta = out["cloud"][0]
    assert kind == "send_array" and array.type == pa.struct(
        [("points", pa.large_binary()), ("width", pa.uint32()), ("height", pa.uint32())])
    assert meta["frame_id"] == "rslidar" and "timestamp" not in meta
    # odom_pose: Pose in odom, child frame mapped to the dictionary name
    kind, array, meta = out["odom_pose"][0]
    assert kind == "send_array" and meta["frame_id"] == "odom" and meta["child_frame"] == "base_footprint"
    assert set(array[0].as_py()) == {"position", "orientation"}


def test_runtime_strip_cloud_frame_only_on_cloud():
    out = _by_output(_run(strip=("cloud",)))
    assert "frame_id" not in out["cloud"][0][2]
    assert out["pointcloud"][0][2]["frame_id"] == "rslidar"
    assert out["odom_pose"][0][2]["frame_id"] == "odom"
    assert out["imu"][0][2]["frame_id"] == "imu_link"
