"""Simulation/Dora orchestration and safe shutdown."""

from __future__ import annotations

from dataclasses import dataclass, replace
import logging
import time
from typing import Any, Callable, Collection, Protocol

from .config import SimulationConfig
from .contract import PRODUCER, HeaderSource, dictionary_frame
from .dora_adapter import InputEvent, StopEvent
from .scheduler import PeriodicSchedule
from .simulator import SimulationSnapshot
from .vehicle import CommandError, VehicleController, WheelTargets, parse_cmd_vel


LOGGER = logging.getLogger(__name__)


class SimulatorProtocol(Protocol):
    time_s: float

    def step(self, targets: WheelTargets) -> None: ...
    def snapshot(self) -> SimulationSnapshot: ...
    def viewer_is_running(self) -> bool: ...
    def sync_viewer(self) -> None: ...
    def close(self) -> None: ...


class AdapterProtocol(Protocol):
    def poll(self, timeout_s: float): ...
    def send(self, output_id: str, payload: bytes, metadata: dict) -> None: ...
    def send_array(self, output_id: str, array, metadata: dict) -> None: ...


@dataclass(frozen=True)
class OutputSpec:
    """One dora output fed by a sensor sample."""

    output_id: str
    frame: str  # sim frame name, mapped by contract.FRAME_NAMES
    encode: Callable[[float, Any], object]  # (timestamp, sample) -> bytes, or (pa.Array, extra_meta) when typed
    typed: bool = False


@dataclass(frozen=True)
class SensorBinding:
    """A sensor sampled once per due tick; every output encodes that one sample."""

    name: str
    rate_hz: float
    sample: Callable[[SimulationSnapshot, float], Any]
    expensive: bool
    outputs: tuple[OutputSpec, ...]
    close: Callable[[], None] | None = None


class RuntimeFailure(RuntimeError):
    """Raised after a configured number of consecutive output failures."""


class SimulationRuntime:
    def __init__(
        self,
        config: SimulationConfig,
        simulator: SimulatorProtocol,
        adapter: AdapterProtocol,
        controller: VehicleController,
        bindings: tuple[SensorBinding, ...],
        *,
        headers: HeaderSource | None = None,
        declared_outputs: Collection[str] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.simulator = simulator
        self.adapter = adapter
        self.controller = controller
        for binding in bindings:
            for output in binding.outputs:
                dictionary_frame(output.frame, f"output {output.output_id}")
        self._all_bindings = bindings
        # declared_outputs None means unknown: produce everything. Otherwise an
        # output the dataflow does not declare is not encoded, and a sensor none
        # of whose outputs is declared is not sampled at all.
        if declared_outputs is not None:
            declared = frozenset(declared_outputs)
            bindings = tuple(
                replace(binding, outputs=kept)
                for binding in bindings
                if (kept := tuple(o for o in binding.outputs if o.output_id in declared))
            )
        self.bindings = bindings
        self.headers = headers if headers is not None else HeaderSource(source=PRODUCER)
        self._schedules = {binding.name: PeriodicSchedule(binding.rate_hz) for binding in bindings}
        self._failures = {output.output_id: 0 for binding in bindings for output in binding.outputs}
        self._monotonic = monotonic
        self._sleep = sleep
        self._stop_requested = False
        self._closed = False
        self._wall_started_s = monotonic()

    def request_stop(self) -> None:
        self._stop_requested = True

    def _drain_events(self) -> None:
        while True:
            event = self.adapter.poll(0.0)
            if event is None:
                return
            if isinstance(event, StopEvent):
                self.request_stop()
                return
            if isinstance(event, InputEvent) and event.input_id == "cmd_vel":
                try:
                    command = parse_cmd_vel(event.payload)
                except CommandError as exc:
                    LOGGER.warning("ignoring invalid cmd_vel: %s", exc)
                    continue
                self.controller.apply(command, self._monotonic())

    def _publish_due(self) -> None:
        snapshot = self.simulator.snapshot()
        for binding in self.bindings:
            due = self._schedules[binding.name].due(self.simulator.time_s)
            if binding.expensive and due:
                due = (due[-1],)
            for timestamp_s in due:
                sample = binding.sample(snapshot, timestamp_s)
                for output in binding.outputs:
                    self._send(output, timestamp_s, output.encode(timestamp_s, sample))

    def _send(self, output: OutputSpec, timestamp_s: float, encoded: object) -> None:
        output_id = output.output_id
        try:
            if output.typed:
                array, extra = encoded
                self.adapter.send_array(
                    output_id, array,
                    self.headers.header(output_id, output.frame, timestamp_s, **extra))
            else:
                self.adapter.send(
                    output_id, encoded,
                    self.headers.header(output_id, output.frame, timestamp_s,
                                        timestamp=float(timestamp_s)))
        except Exception as exc:
            self._failures[output_id] += 1
            LOGGER.error("Dora output %s failed (%d/%d): %s", output_id, self._failures[output_id], self.config.runtime.output_failure_limit, exc)
            if self._failures[output_id] >= self.config.runtime.output_failure_limit:
                raise RuntimeFailure(f"Dora output {output_id} failed repeatedly") from exc
        else:
            self._failures[output_id] = 0

    def _throttle(self) -> None:
        if self.config.runtime.fastest:
            return
        target = self._wall_started_s + self.simulator.time_s / self.config.physics.realtime_factor
        delay = target - self._monotonic()
        if delay > 0:
            self._sleep(delay)

    def _iteration(self) -> None:
        self._drain_events()
        if self._stop_requested:
            return
        targets = self.controller.update(self._monotonic(), self.config.physics.timestep_s)
        self.simulator.step(targets)
        self._publish_due()
        self.simulator.sync_viewer()
        if not self.simulator.viewer_is_running():
            self.request_stop()
        self._throttle()

    def run_until(self, end_simulation_s: float) -> None:
        while self.simulator.time_s < end_simulation_s and not self._stop_requested:
            self._iteration()

    def run(self) -> int:
        result = 0
        try:
            while not self._stop_requested:
                self._iteration()
        except RuntimeFailure as exc:
            LOGGER.error("simulation runtime stopped: %s", exc)
            result = 1
        finally:
            self.close()
        return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.simulator.step(self.controller.stop())
        finally:
            for binding in self._all_bindings:
                if binding.close is not None:
                    binding.close()
            self.simulator.close()
