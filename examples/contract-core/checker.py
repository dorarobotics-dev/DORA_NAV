"""Run-time assertions for the contract-core chain; writes a JSON record.

The checker receives frame_transform's output (`cloud`, `diag`) and, straight
from the sim, the raw lidar cloud (`raw`, in rslidar) and the odometry pose
(`pose`, odom <- base_footprint). Judging is the pure function `check()` over
the collected events; `main()` only reads dora events and writes the record.

EXPECT=transformed, every criterion must hold:
- Frame and header: every cloud is in odom and carries header_seq (a missing
  one is a recorded problem, not a crash); header_seq strictly increases.
- Validity: at least one `ok` cloud; otherwise only `stale`, at most 3 clouds
  and all before the first `ok` one.
- Independent recomputation, judged on EVERY cloud: the raw cloud with the same
  stamp_ns is moved to odom in plain numpy (own quaternion code, not
  dora.transform) through the static chain base_footprint -> base_link ->
  rslidar read from robots/mickrobot-sim/extrinsics.yml and a sim pose. The
  node pairs a cloud with the latest pose it holds and calls it `ok` when the
  stamps differ by at most POSE_MAX_AGE_MS. The sim sends a tick's cloud
  before that tick's pose, so the node never holds a pose stamped after the
  cloud. The candidates are therefore the poses with
  cloud stamp - POSE_MAX_AGE_MS <= pose stamp <= cloud stamp for an `ok`
  cloud, and every pose stamped <= the cloud for a `stale` one (a stale
  cloud may hold any older pose, so it is checked loosely; hence the cap on
  stale clouds above). The best candidate must reproduce every point to
  within 1e-3 m (xyz) with the intensity unchanged. A pose ignored, inverted
  or taken from the wrong time fails as soon as the robot has moved.
- Motion: over the run the pose moves at least 1.0 m in x/y from its first
  value and turns at least 0.5 rad, so the recomputation is pose-dependent.
  The driver (cmd_vel_sender.py) follows the test world's wall loop.
- Ground: the per-cloud ground error is the median |z| in odom of the lowest
  10 % of points. The run median must be <= 0.03 m (decision of 2026-11-02),
  and every cloud stamped >= 1.0 s of sim time after the first cloud must be
  <= 0.03 m: the spawn transient (the body starts at z = 0, bounces to 0.17 m
  and settles at 0.0896 m within ~0.5 s; a planar odometry pose cannot
  represent that) is excluded, nothing after it.
- Drops: counting only clouds stamped at least IN_FLIGHT_MS before the last
  raw cloud (the newest raw clouds may still be inside frame_transform when
  the checker stops; 2026-11-02 had 2 such), transformed >= raw - 3. A node
  that stops publishing early is still caught: the cut-off follows the raw
  stream, not the transformed one. A diag must have arrived, and its drops
  hold only `no_pose` (at most 3); the other labels (bad_frame, bad_header,
  malformed, bad_pose) fail the run. Other diag keys (items, bytes, proc_ns,
  ...) are ignored.
EXPECT=dropped: no cloud arrives; diag counts one bad_frame per input cloud.
"""
import json
import math
import os
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_EXTRINSICS = HERE / "../../robots/mickrobot-sim/extrinsics.yml"
GROUND_TOL_M = 0.03
SETTLE_S = 1.0
RECOMPUTE_TOL_M = 1e-3
POSE_MAX_AGE_MS = 100  # dora_frame_transform.core.POSE_MAX_AGE_MS
MIN_XY_M = 1.0
MIN_YAW_RAD = 0.5
DROP_CAP = 3  # also the cap on stale clouds
IN_FLIGHT_MS = 200
RAW_KEEP = 100  # raw clouds held for pairing (10 s at 10 Hz)


def quat_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rigid(translation, xyzw):
    m = np.eye(4)
    m[:3, :3] = quat_matrix(*xyzw)
    m[:3, 3] = translation
    return m


def static_chain(path, parent, child):
    """The pose of `child` in `parent` (maps child points to parent), composed
    from the extrinsics file's edges."""
    edges = {e["child"]: e for e in yaml.safe_load(Path(path).read_text())["transforms"]}
    m, frame = np.eye(4), child
    while frame != parent:
        if frame not in edges:
            raise ValueError(f"{path}: no edge from {frame!r} towards {parent!r}")
        e = edges[frame]
        m = rigid(e["translation_m"], e["rotation_xyzw"]) @ m
        frame = e["parent"]
    return m


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Collector:
    """Consumes events in arrival order; no I/O. Clouds are recomputed as they
    become decidable (a pose newer than the cloud's age window has arrived)."""

    def __init__(self, static):
        self.static = static
        self.raw_in = 0
        self.raw_stamps = []
        self.cloud_stamps = []
        self.raws = {}
        self.poses = []  # (stamp_ns, matrix, x, y, yaw)
        self.pending = []
        self.seqs, self.validity, self.missing = [], [], Counter()
        self.frames = Counter()
        self.ground = []  # (stamp_ns | None, err)
        self.errors, self.lags, self.unchecked = [], [], Counter()
        self.last_diag = {}

    def raw(self, meta, pts):
        self.raw_in += 1
        stamp = (meta or {}).get("stamp_ns")
        if isinstance(stamp, int):
            self.raw_stamps.append(stamp)
            self.raws[stamp] = pts
            if len(self.raws) > RAW_KEEP:
                del self.raws[min(self.raws)]

    def pose(self, meta, row):
        stamp = (meta or {}).get("stamp_ns")
        if not isinstance(stamp, int):
            return
        p, o = row["position"], row["orientation"]
        q = (o["x"], o["y"], o["z"], o["w"])
        self.poses.append((stamp, rigid((p["x"], p["y"], p["z"]), q), p["x"], p["y"], _yaw(q)))
        self._flush(stamp)

    def diag(self, d):
        self.last_diag = d

    def cloud(self, meta, pts):
        meta = meta or {}
        self.frames[meta.get("frame_id")] += 1
        for key in ("header_seq", "stamp_ns", "validity"):
            if key not in meta:
                self.missing[key] += 1
        if "header_seq" in meta:
            self.seqs.append(meta["header_seq"])
        validity = meta.get("validity")
        self.validity.append(validity)
        stamp = meta.get("stamp_ns")
        stamp = stamp if isinstance(stamp, int) else None
        z = pts[:, 2]
        low = z[z <= np.quantile(z, 0.10)] if len(z) else z
        self.ground.append((stamp, float(np.median(np.abs(low))) if len(low) else float("inf")))
        if stamp is None:
            self.unchecked["no stamp_ns"] += 1
        else:
            self.cloud_stamps.append(stamp)
            self.pending.append((stamp, validity, pts))

    def _flush(self, newest_pose_stamp=None):
        keep = []
        for item in self.pending:
            if newest_pose_stamp is not None and newest_pose_stamp <= item[0] + POSE_MAX_AGE_MS * 1_000_000:
                keep.append(item)
            else:
                self._recompute(*item)
        self.pending = keep

    def _recompute(self, stamp, validity, out):
        raw = self.raws.pop(stamp, None)
        if raw is None:
            self.unchecked["no raw cloud with its stamp_ns"] += 1
            return
        if raw.shape != out.shape:
            self.unchecked[f"point count/layout changed {raw.shape} -> {out.shape}"] += 1
            return
        window = POSE_MAX_AGE_MS * 1_000_000
        cands = [p for p in self.poses
                 if p[0] <= stamp and (validity != "ok" or stamp - p[0] <= window)]
        if not cands:
            self.unchecked["no pose the node could have used"] += 1
            return
        xyz = raw[:, :3].astype(np.float64)
        got = out[:, :3].astype(np.float64)
        best, lag = math.inf, None
        for p in cands:
            m = p[1] @ self.static
            err = float(np.max(np.linalg.norm(xyz @ m[:3, :3].T + m[:3, 3] - got, axis=1), initial=0.0))
            if err < best:
                best, lag = err, stamp - p[0]
        if raw.shape[1] > 3 and not np.array_equal(raw[:, 3:], out[:, 3:]):
            best = math.inf
        self.errors.append(best)
        self.lags.append(lag)

    def finish(self):
        self._flush()
        return self


def judge(c, expect):
    """Problems (empty when the run passes) and the record for a finished Collector."""
    problems = []
    ground = [g for _, g in c.ground]
    first = min((s for s, _ in c.ground if s is not None), default=None)
    settled = [g for s, g in c.ground if s is not None and s - first >= SETTLE_S * 1e9]
    xs = np.array([p[2] for p in c.poses]) if c.poses else np.zeros(0)
    ys = np.array([p[3] for p in c.poses]) if c.poses else np.zeros(0)
    yaws = np.unwrap([p[4] for p in c.poses]) if c.poses else np.zeros(0)
    xy = float(np.max(np.hypot(xs - xs[0], ys - ys[0]))) if c.poses else 0.0
    turn = float(np.max(np.abs(yaws - yaws[0]))) if c.poses else 0.0
    published = len(c.validity)
    drops = c.last_diag.get("drops", {}) if isinstance(c.last_diag, dict) else {}
    if c.raw_stamps:
        cut = max(c.raw_stamps) - IN_FLIGHT_MS * 1_000_000
        in_flight = sum(s > cut for s in c.raw_stamps)
        counted_raw = c.raw_in - in_flight
        counted_published = published - sum(s > cut for s in c.cloud_stamps)
    else:
        counted_raw, counted_published = c.raw_in, published

    if expect == "transformed":
        if not published:
            problems.append("no transformed clouds")
        for frame, n in c.frames.items():
            if frame != "odom":
                problems.append(f"{n} clouds in frame {frame!r}")
        for key, n in c.missing.items():
            problems.append(f"{n} clouds without {key}")
        if any(b <= a for a, b in zip(c.seqs, c.seqs[1:])):
            problems.append("header_seq not strictly increasing")
        unexpected = Counter(v for v in c.validity if v not in ("ok", "stale"))
        if unexpected:
            problems.append(f"unexpected validity {dict(unexpected)}")
        if published and "ok" not in c.validity:
            problems.append("no ok cloud")
        first_ok = c.validity.index("ok") if "ok" in c.validity else published
        late_stale = sum(v == "stale" for v in c.validity[first_ok:])
        if late_stale:
            problems.append(f"{late_stale} stale clouds after the first ok cloud")
        n_stale = c.validity.count("stale")
        if n_stale > DROP_CAP:
            problems.append(f"{n_stale} stale clouds > {DROP_CAP}")
        for why, n in c.unchecked.items():
            problems.append(f"recompute: {n} clouds not checkable: {why}")
        bad = sum(e > RECOMPUTE_TOL_M for e in c.errors)
        if bad:
            problems.append(f"recompute: {bad} of {len(c.errors)} clouds differ from the independent "
                            f"transform by more than {RECOMPUTE_TOL_M} m (max {max(c.errors):.4g} m)")
        if xy < MIN_XY_M or turn < MIN_YAW_RAD:
            problems.append(f"robot moved only {xy:.2f} m / {turn:.2f} rad "
                            f"(need {MIN_XY_M} m and {MIN_YAW_RAD} rad)")
        if ground and float(np.median(ground)) > GROUND_TOL_M:
            problems.append(f"median ground error {float(np.median(ground)):.3f} m > {GROUND_TOL_M}")
        if published and not settled:
            problems.append(f"no cloud stamped {SETTLE_S} s after the first to judge ground error on")
        if settled and max(settled) > GROUND_TOL_M:
            problems.append(f"ground error {max(settled):.3f} m > {GROUND_TOL_M} on a cloud "
                            f">= {SETTLE_S} s after the first")
        if counted_published < counted_raw - DROP_CAP:
            problems.append(f"published {counted_published} < raw_in {counted_raw} - {DROP_CAP} "
                            f"(clouds stamped within {IN_FLIGHT_MS} ms of the last raw cloud not counted)")
        if not c.last_diag:
            problems.append("no diag received")
        for label, n in drops.items():
            if label != "no_pose" and n:
                problems.append(f"diag drops {n} clouds as {label}")
        if drops.get("no_pose", 0) > DROP_CAP:
            problems.append(f"diag drops {drops['no_pose']} clouds as no_pose > {DROP_CAP}")
    elif expect == "dropped":
        if published:
            problems.append(f"{published} clouds published despite stripped frame_id")
        bad = drops.get("bad_frame", 0)
        if bad < c.last_diag.get("in", 1) or c.last_diag.get("in", 0) == 0:
            problems.append(f"diag {c.last_diag} does not count every input as bad_frame")
    else:
        problems.append(f"unknown EXPECT {expect!r}")

    errors = [e for e in c.errors if math.isfinite(e)]
    lags = [lag for lag in c.lags if lag is not None]
    return {
        "expect": expect, "raw_in": c.raw_in, "published": published,
        "drop_count": {"raw": counted_raw, "published": counted_published, "in_flight_ms": IN_FLIGHT_MS},
        "validity": dict(Counter(c.validity)),
        "recompute": {"checked": len(c.errors), "unchecked": dict(c.unchecked),
                      "max_err_m": max(c.errors, default=None) if all(map(math.isfinite, c.errors)) else "inf",
                      "max_finite_err_m": max(errors, default=None),
                      "pose_lag_ms": {"min": min(lags) / 1e6 if lags else None,
                                      "max": max(lags) / 1e6 if lags else None}},
        "motion": {"poses": len(c.poses), "xy_m": xy, "yaw_rad": turn},
        "ground_err_m": {
            "median": float(np.median(ground)) if ground else None,
            "p95": float(np.percentile(ground, 95)) if ground else None,
            "max": max(ground, default=None),
            "max_after_settle": max(settled, default=None), "settle_s": SETTLE_S},
        "last_diag": c.last_diag, "problems": problems,
    }


def check(events, expect, static):
    """Judge a run given as (input id, metadata, payload) events in arrival
    order: points arrays for raw/cloud, the pose struct as a dict, the diag dict."""
    c = Collector(static)
    for kind, meta, payload in events:
        getattr(c, kind)(*((payload,) if kind == "diag" else (meta, payload)))
    return judge(c.finish(), expect)


def points(value):
    return np.frombuffer(value[0].as_py()["points"], dtype="<f4").reshape(-1, 4)


def main():
    from dora import Node

    run_s = float(os.environ.get("RUN_SECONDS", "60"))
    expect = os.environ["EXPECT"]
    record_path = Path(os.environ["RECORD"])
    static = static_chain(os.environ.get("CHECK_EXTRINSICS", DEFAULT_EXTRINSICS), "base_footprint", "rslidar")
    c = Collector(static)
    start = time.monotonic()
    for event in Node():
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        eid, meta = event["id"], event["metadata"]
        if eid == "raw":
            c.raw(meta, points(event["value"]))
        elif eid == "pose":
            c.pose(meta, event["value"][0].as_py())
        elif eid == "diag":
            c.diag(json.loads(bytes(event["value"].to_numpy()).decode()))
        elif eid == "cloud":
            c.cloud(meta, points(event["value"]))
        if time.monotonic() - start >= run_s:
            break

    record = {"run_s": run_s, **judge(c.finish(), expect)}
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(record, indent=2) + "\n")
    problems = record["problems"]
    print("CHECKER_OK" if not problems else f"CHECKER_FAIL {problems}")
    sys.exit(0 if not problems else 1)


if __name__ == "__main__":
    main()
