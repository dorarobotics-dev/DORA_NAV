"""Contract headers and typed payloads for the simulator's outputs.

The sim names frames as its MuJoCo model does; the frame dictionary rejects
some of those upstream names, so they are mapped here, once, explicitly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction

import numpy as np
import pyarrow as pa

# sim model name -> frame dictionary name.
# gnss_link has no dictionary frame; GNSS fixes are ENU positions in the world frame, so map.
FRAME_NAMES = {
    "lidar_link": "rslidar",
    "camera_link": "camera_front",
    "imu_link": "imu_link",
    "odom": "odom",
    "base_footprint": "base_footprint",
    "map": "map",
    "gnss_link": "map",
}

POINTCLOUD_TYPE = pa.struct([("points", pa.large_binary()), ("width", pa.uint32()), ("height", pa.uint32())])
_VEC3 = pa.struct([("x", pa.float64()), ("y", pa.float64()), ("z", pa.float64())])
_QUAT = pa.struct([("x", pa.float64()), ("y", pa.float64()), ("z", pa.float64()), ("w", pa.float64())])
POSE_TYPE = pa.struct([("position", _VEC3), ("orientation", _QUAT)])


def stamp_ns(seconds: float) -> int:
    """Exact nanoseconds from sim seconds (round half to even); never below 1."""
    return max(1, int(round(Fraction(seconds) * 1_000_000_000)))


@dataclass
class HeaderSource:
    source: str
    strip_frame_on: tuple[str, ...] = ()
    _seq: dict[str, int] = field(default_factory=dict)

    def header(self, output_id: str, sim_frame: str, timestamp_s: float, **extra) -> dict:
        frame = FRAME_NAMES[sim_frame]
        seq = self._seq.get(output_id, 0)
        self._seq[output_id] = seq + 1
        meta = {"frame_id": frame, "header_seq": seq, "stamp_ns": stamp_ns(timestamp_s),
                "source": self.source, "validity": "ok", **extra}
        if output_id in self.strip_frame_on:
            del meta["frame_id"]
        return meta


def cloud_array(points_xyzi) -> pa.Array:
    p = np.ascontiguousarray(points_xyzi, dtype="<f4")
    return pa.array([{"points": p.tobytes(), "width": len(p), "height": 1}], type=POINTCLOUD_TYPE)


def pose_array(position, orientation_xyzw) -> pa.Array:
    x, y, z, w = (float(c) for c in orientation_xyzw)
    px, py, pz = (float(c) for c in position)
    return pa.array([{"position": {"x": px, "y": py, "z": pz},
                      "orientation": {"x": x, "y": y, "z": z, "w": w}}], type=POSE_TYPE)
