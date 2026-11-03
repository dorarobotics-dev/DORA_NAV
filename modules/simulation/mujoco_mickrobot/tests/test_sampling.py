"""One sample per sensor per tick, shared by every output it feeds."""
from dataclasses import replace
import json

import numpy as np
import pytest

from mujoco_mickrobot.config import load_config
from mujoco_mickrobot.contract import UnknownFrame
from mujoco_mickrobot.dora_adapter import DoraAdapter
from mujoco_mickrobot.main import _bindings
from mujoco_mickrobot.messages import decode_pointcloud
from mujoco_mickrobot.runtime import SimulationRuntime
from mujoco_mickrobot.simulator import SimulationSnapshot
from mujoco_mickrobot.vehicle import VehicleController


class Sim:
    def __init__(self):
        self.time_s = 0.0
        self.raycasts = 0

    def step(self, targets):
        self.time_s += 0.002

    def snapshot(self):
        return replace(SimulationSnapshot.stationary(), wheel_velocity_rad_s=(1.0, 1.0, 2.0, 2.0))

    def raycast_from_site(self, site, directions, max_range):
        self.raycasts += 1
        return np.full(len(directions), 5.0)

    def viewer_is_running(self):
        return True

    def sync_viewer(self):
        pass

    def close(self):
        pass


class Adapter:
    def __init__(self):
        self.sent = []

    def poll(self, timeout_s):
        return None

    def send(self, output_id, payload, metadata):
        self.sent.append((output_id, payload, metadata))

    def send_array(self, output_id, array, metadata):
        self.sent.append((output_id, array, metadata))

    def of(self, output_id):
        return [(p, m) for oid, p, m in self.sent if oid == output_id]


def _config(*, lidar=True, odometry=True, gnss=False, imu=False, **frames):
    c = load_config(None)
    c = replace(
        c,
        lidar=replace(c.lidar, enabled=lidar, frame_id=frames.get("lidar_frame", c.lidar.frame_id)),
        camera=replace(c.camera, enabled=False),
        imu=replace(c.imu, enabled=imu),
        odometry=replace(c.odometry, enabled=odometry,
                         child_frame_id=frames.get("odom_child", c.odometry.child_frame_id)),
        gnss=replace(c.gnss, enabled=gnss),
        runtime=replace(c.runtime, deterministic=False, random_seed=42),
    )
    return c


def _run(config, until=0.1, declared=None):
    sim, adapter = Sim(), Adapter()
    runtime = SimulationRuntime(config, sim, adapter, VehicleController(config.vehicle),
                                _bindings(config, sim), declared_outputs=declared,
                                sleep=lambda s: None)
    runtime.run_until(until)
    return sim, adapter


def test_lidar_is_cast_once_per_tick_and_cloud_carries_the_same_points():
    sim, adapter = _run(_config(odometry=False), until=0.35)
    legacy, typed = adapter.of("pointcloud"), adapter.of("cloud")
    assert len(legacy) == len(typed) == sim.raycasts == 4
    for (payload, _), (array, _) in zip(legacy, typed):
        _, points = decode_pointcloud(bytes(payload))
        assert np.array_equal(points, np.frombuffer(array[0].as_py()["points"], dtype="<f4").reshape(-1, 4))


# Legacy Odometry (seed 42, noisy, wheels 1,1,2,2) from base a9257f5, sampled at
# 50 Hz up to 0.1 s: [twist.linear.x, twist.angular.z, pose.x, pose.y].
BASE_ODOMETRY = [
    [0.22495418068563502, 0.2139315680693827, 0.0, 0.0],
    [0.22493208890786076, 0.21376466079651735, 0.0044986314998150525, 9.616499013705987e-06],
    [0.2247579897643279, 0.21405574176165082, 0.008993698777974438, 3.845657497522845e-05],
    [0.22475121263357326, 0.21482169349687658, 0.0134884654929469, 8.657320495197271e-05],
    [0.22480504859540848, 0.2137112153242729, 0.01798406134023981, 0.00015396702808398877],
    [0.22478843524383238, 0.2148809327457485, 0.02247899484846463, 0.00024062153971474493],
]


def test_legacy_odometry_noise_equals_base():
    _, adapter = _run(_config(lidar=False))
    got = []
    for payload, _ in adapter.of("Odometry"):
        v = json.loads(bytes(payload))
        got.append([v["twist"]["linear"]["x"], v["twist"]["angular"]["z"],
                    v["pose"]["position"]["x"], v["pose"]["position"]["y"]])
    assert got == BASE_ODOMETRY


def test_odom_pose_is_the_same_sample_as_odometry():
    _, adapter = _run(_config(lidar=False))
    legacy, typed = adapter.of("Odometry"), adapter.of("odom_pose")
    assert len(legacy) == len(typed) > 0
    for (payload, _), (array, meta) in zip(legacy, typed):
        assert json.loads(bytes(payload))["pose"]["position"] == array[0].as_py()["position"]
        assert meta["child_frame"] == "base_footprint"


def test_undeclared_outputs_are_not_produced():
    sim, adapter = _run(_config(), until=0.35, declared={"pointcloud", "Odometry"})
    assert {oid for oid, _, _ in adapter.sent} == {"pointcloud", "Odometry"}
    assert sim.raycasts == len(adapter.of("pointcloud")) == 4


def test_a_sensor_with_no_declared_output_is_not_sampled():
    sim, adapter = _run(_config(), until=0.35, declared={"odom_pose"})
    assert sim.raycasts == 0
    assert {oid for oid, _, _ in adapter.sent} == {"odom_pose"}


def test_none_declared_means_unknown_so_everything_is_produced():
    _, adapter = _run(_config(), until=0.35, declared=None)
    assert {oid for oid, _, _ in adapter.sent} == {"pointcloud", "cloud", "Odometry", "odom_pose"}


def test_unknown_sensor_frame_fails_at_startup_naming_it():
    config = _config(lidar_frame="velodyne", odometry=False)
    with pytest.raises(UnknownFrame) as err:
        SimulationRuntime(config, Sim(), Adapter(), VehicleController(config.vehicle), _bindings(config, Sim()))
    assert "velodyne" in str(err.value) and "lidar_link" in str(err.value)


def test_unknown_odometry_child_frame_fails_at_startup():
    config = _config(lidar=False, odom_child="chassis")
    with pytest.raises(UnknownFrame, match="chassis"):
        _bindings(config, Sim())


def test_gnss_is_frameless():
    _, adapter = _run(_config(lidar=False, odometry=False, gnss=True))
    (_, meta), *_ = adapter.of("gnss")
    assert meta["frame_id"] == "none"


class _Node:
    def __init__(self, config):
        self._config = config

    def node_config(self):
        if isinstance(self._config, Exception):
            raise self._config
        return self._config


def test_adapter_reads_declared_outputs_from_the_node_config():
    assert DoraAdapter(_Node({"inputs": {}, "outputs": ["cloud", "imu"]})).declared_outputs() == {"cloud", "imu"}
    assert DoraAdapter(_Node({"outputs": []})).declared_outputs() == frozenset()
    assert DoraAdapter(_Node({})).declared_outputs() is None
    assert DoraAdapter(_Node(RuntimeError("no config"))).declared_outputs() is None


class _Camera:
    made: list["_Camera"]

    def __init__(self, config, simulator):
        self.closed = 0
        _Camera.made.append(self)

    def sample(self, simulator, timestamp_s):
        raise AssertionError("not sampled in these tests")

    def close(self):
        self.closed += 1


@pytest.fixture
def camera(monkeypatch):
    from mujoco_mickrobot import main
    _Camera.made = []
    monkeypatch.setattr(main, "CameraSensor", _Camera)
    return _Camera.made


def _with_camera(config):
    return replace(config, camera=replace(config.camera, enabled=True))


def test_camera_is_not_created_when_image_is_not_declared(camera):
    config = _with_camera(_config())
    assert [b.name for b in _bindings(config, Sim(), declared={"cloud"})] == ["lidar", "odometry"]
    assert camera == []
    _bindings(config, Sim(), declared={"image"})
    _bindings(config, Sim(), declared=None)
    assert len(camera) == 2


def test_camera_is_closed_when_a_later_binding_fails(camera):
    config = _with_camera(_config(odom_child="chassis"))
    with pytest.raises(UnknownFrame, match="chassis"):
        _bindings(config, Sim())
    assert [c.closed for c in camera] == [1]


def test_camera_is_closed_when_the_runtime_refuses_a_frame(camera):
    from mujoco_mickrobot.main import _runtime
    config = _with_camera(_config(lidar_frame="velodyne"))
    with pytest.raises(UnknownFrame, match="velodyne"):
        _runtime(config, Sim(), Adapter(), declared=None)
    assert [c.closed for c in camera] == [1]


def test_close_reaches_a_sensor_whose_outputs_were_all_filtered_out(camera):
    config = _with_camera(_config(lidar=False))
    runtime = SimulationRuntime(config, Sim(), Adapter(), VehicleController(config.vehicle),
                                _bindings(config, Sim()), declared_outputs={"odom_pose"})
    assert [b.name for b in runtime.bindings] == ["odometry"]
    runtime.close()
    assert [c.closed for c in camera] == [1]
