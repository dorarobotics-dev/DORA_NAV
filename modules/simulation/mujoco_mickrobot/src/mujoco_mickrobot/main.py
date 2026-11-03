"""Command-line entry point for the MuJoCo Dora simulation node."""

from __future__ import annotations

import argparse
from collections.abc import Collection
from dataclasses import replace
import logging
import os
from pathlib import Path
import signal

from .config import SimulationConfig, load_config
from .contract import PRODUCER, HeaderSource, cloud_array, dictionary_frame, pose_array
from .dora_adapter import DoraAdapter
from .messages import encode_gnss, encode_image, encode_imu, encode_odometry, encode_pointcloud
from .model_loader import resolve_model_path
from .runtime import OutputSpec, SensorBinding, SimulationRuntime
from .sensors.camera import CameraSensor
from .sensors.gnss import GnssSensor
from .sensors.imu import ImuSensor
from .sensors.lidar import LidarSensor
from .sensors.odometry import WheelOdometry
from .simulator import MujocoSimulator
from .vehicle import VehicleController


def _close_all(bindings) -> None:
    for binding in bindings:
        if binding.close is not None:
            binding.close()


def _bindings(config: SimulationConfig, simulator: MujocoSimulator,
              declared: Collection[str] | None = None) -> tuple[SensorBinding, ...]:
    """One binding per sensor: sampled once per tick, the sample feeding each of its outputs.

    ``declared`` is the dataflow's declared outputs (None: unknown). The camera
    renderer is only created when ``image`` may be declared. If a binding
    fails (an unknown frame), the ones already made are closed first.
    """
    result: list[SensorBinding] = []
    try:
        _add_bindings(config, simulator, declared, result)
    except Exception:
        _close_all(result)
        raise
    return tuple(result)


def _add_bindings(config: SimulationConfig, simulator: MujocoSimulator,
                  declared: Collection[str] | None, result: list[SensorBinding]) -> None:
    deterministic = config.runtime.deterministic
    seed = config.runtime.random_seed
    if config.lidar.enabled:
        lidar = LidarSensor(config.lidar, simulator, deterministic=deterministic, seed=seed + 1)
        result.append(SensorBinding("lidar", config.lidar.rate_hz, lambda snapshot, timestamp: lidar.sample(simulator, timestamp), True, (
            OutputSpec("pointcloud", config.lidar.frame_id, encode_pointcloud),
            OutputSpec("cloud", config.lidar.frame_id, lambda timestamp, points: (cloud_array(points), {}), typed=True),
        )))
    if config.camera.enabled and (declared is None or "image" in declared):
        camera = CameraSensor(config.camera, simulator)
        result.append(SensorBinding("camera", config.camera.rate_hz, lambda snapshot, timestamp: camera.sample(simulator, timestamp), True, (
            OutputSpec("image", config.camera.frame_id, lambda timestamp, sample: encode_image(sample)),
        ), camera.close))
    if config.imu.enabled:
        imu = ImuSensor(config.imu, deterministic=deterministic, seed=seed + 2)
        result.append(SensorBinding("imu", config.imu.rate_hz, imu.sample, False, (
            OutputSpec("imu", config.imu.frame_id, lambda timestamp, sample: encode_imu(sample)),
        )))
    if config.odometry.enabled:
        odometry = WheelOdometry(config.odometry, config.vehicle, deterministic=deterministic, seed=seed + 3)
        child_frame = dictionary_frame(config.odometry.child_frame_id, "odometry.child_frame_id")
        result.append(SensorBinding("odometry", config.odometry.rate_hz, odometry.sample, False, (
            OutputSpec("Odometry", config.odometry.frame_id, lambda timestamp, sample: encode_odometry(sample)),
            OutputSpec("odom_pose", config.odometry.frame_id,
                       lambda timestamp, sample: (pose_array(sample.position_m, sample.orientation_xyzw),
                                                  {"child_frame": child_frame}),
                       typed=True),
        )))
    if config.gnss.enabled:
        gnss = GnssSensor(config.gnss, deterministic=deterministic, seed=seed + 4)
        result.append(SensorBinding("gnss", config.gnss.rate_hz, gnss.sample, False, (
            OutputSpec("gnss", config.gnss.frame_id, lambda timestamp, sample: encode_gnss(sample)),
        )))


def _runtime(config: SimulationConfig, simulator, adapter, *,
             declared: Collection[str] | None, headers: HeaderSource | None = None) -> SimulationRuntime:
    """The runtime over fresh bindings; the bindings are closed if it refuses them."""
    bindings = _bindings(config, simulator, declared)
    try:
        return SimulationRuntime(config, simulator, adapter, VehicleController(config.vehicle), bindings,
                                 headers=headers, declared_outputs=declared)
    except Exception:
        _close_all(bindings)
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MickRobot in MuJoCo as a Dora node")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fastest", action="store_true")
    parser.add_argument("--validate-model", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = load_config(args.config)
    if args.headless or args.fastest:
        config = replace(config, runtime=replace(config.runtime, headless=args.headless or config.runtime.headless, fastest=args.fastest or config.runtime.fastest))
    model_path = resolve_model_path()
    simulator = MujocoSimulator(config, model_path)
    if args.validate_model:
        print(f"valid model: {model_path} nq={simulator.model.nq} nv={simulator.model.nv} nu={simulator.model.nu}")
        simulator.close()
        return
    if not config.runtime.headless:
        simulator.launch_viewer()
    try:
        import dora

        adapter = DoraAdapter(dora.Node())
        headers = HeaderSource(
            source=PRODUCER,
            strip_frame_on=("cloud",) if os.environ.get("SIM_STRIP_CLOUD_FRAME") == "1" else (),
        )
        declared = adapter.declared_outputs()
        if declared is None:
            logging.getLogger(__name__).info("declared outputs unknown; producing every output")
        runtime = _runtime(config, simulator, adapter, declared=declared, headers=headers)
    except Exception:
        simulator.close()
        raise
    signal.signal(signal.SIGINT, lambda signum, frame: runtime.request_stop())
    signal.signal(signal.SIGTERM, lambda signum, frame: runtime.request_stop())
    raise SystemExit(runtime.run())


if __name__ == "__main__":
    main()
