"""Self-test of the live-run checker: synthetic runs fed to its judging code.

Each run mimics the sim's order of events: per 20 ms tick a pose (50 Hz) and,
every 100 ms, a raw lidar cloud sent before that tick's pose, which the
frame-transform node then places in odom with the latest pose it holds.
"""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("checker", HERE / "checker.py")
checker = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(checker)

EXTRINSICS = HERE / "../../robots/mickrobot-sim/extrinsics.yml"
STATIC = checker.static_chain(EXTRINSICS, "base_footprint", "rslidar")
RNG = np.random.default_rng(7)


def pose_at(t):
    """A drive of 2 m along a curve while turning 1 rad over 6 s."""
    s = min(t / 6.0, 1.0)
    yaw = 1.0 * s
    return (2.0 * s, 0.5 * s * s, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def matrix(position, q):
    m = np.eye(4)
    m[:3, :3] = checker.quat_matrix(*q)
    m[:3, 3] = position
    return m


def world_cloud(n=400):
    ground = np.column_stack([RNG.uniform(-5, 5, (n // 2, 2)), np.zeros(n // 2)])
    walls = np.column_stack([RNG.uniform(-5, 5, (n // 2, 2)), RNG.uniform(0.5, 2.0, n // 2)])
    return np.vstack([ground, walls])


def apply(m, xyzi):
    out = xyzi.copy()
    out[:, :3] = (xyzi[:, :3].astype(np.float64) @ m[:3, :3].T + m[:3, 3]).astype(np.float32)
    return out


def run(seconds=6.0, mutate=None, static_robot=False):
    """Events as the checker sees them. `mutate(i, pose_m, meta, raw) -> (out_points | None, meta)`."""
    events, held, seq, n_in, n_out, drops = [], None, 0, 0, 0, {}
    ticks = int(seconds * 50) + 1
    for k in range(ticks):
        t = k * 0.02
        stamp = k * 20_000_000 + 1
        if k % 5 == 0:
            n_in += 1
            p, q = pose_at(0.0 if static_robot else t)
            world = world_cloud()
            raw = np.column_stack([world, RNG.uniform(0, 1, len(world))]).astype(np.float32)
            raw = apply(np.linalg.inv(matrix(p, q) @ STATIC), raw)
            rmeta = {"frame_id": "rslidar", "header_seq": n_in - 1, "stamp_ns": stamp,
                     "source": "mujoco_mickrobot", "validity": "ok"}
            events.append(("raw", rmeta, raw))
            if held is None:
                drops["no_pose"] = drops.get("no_pose", 0) + 1
            else:
                pose_stamp, pose_m = held
                meta = {"frame_id": "odom", "header_seq": seq, "stamp_ns": stamp,
                        "source": "mujoco_mickrobot", "validity": "ok"}
                out = apply(pose_m @ STATIC, raw)
                if mutate is not None:
                    out, meta = mutate(n_in - 1, pose_m, meta, raw)
                if out is not None:
                    events.append(("cloud", meta, out))
                    seq += 1
                    n_out += 1
            if n_in % 10 == 0:
                events.append(("diag", None, {"in": n_in, "out": n_out, "drops": dict(drops)}))
        p, q = pose_at(0.0 if static_robot else t)
        pmeta = {"frame_id": "odom", "child_frame": "base_footprint", "header_seq": k,
                 "stamp_ns": stamp, "source": "mujoco_mickrobot", "validity": "ok"}
        events.append(("pose", pmeta, {"position": dict(zip("xyz", p)),
                                       "orientation": dict(zip("xyzw", q))}))
        held = (stamp, matrix(p, q))
    return events


def judge(events, expect="transformed"):
    return checker.check(events, expect, STATIC)


def test_correct_transform_passes():
    record = judge(run())
    assert record["problems"] == []
    assert record["recompute"]["max_err_m"] < 1e-4
    assert record["motion"]["xy_m"] >= 1.0 and record["motion"]["yaw_rad"] >= 0.5
    assert record["published"] == record["raw_in"] - 1


def test_pose_ignored_fails():
    record = judge(run(mutate=lambda i, pose, meta, raw: (apply(STATIC, raw), meta)))
    assert any("differ from the independent" in p for p in record["problems"]), record["problems"]


def test_pose_inverted_fails():
    record = judge(run(mutate=lambda i, pose, meta, raw: (apply(np.linalg.inv(pose) @ STATIC, raw), meta)))
    assert any("differ from the independent" in p for p in record["problems"]), record["problems"]


def test_old_pose_fails():
    old = {}

    def stale_pose(i, pose, meta, raw):
        old.setdefault(i, pose)
        return apply(old.get(i - 10, pose) @ STATIC, raw), meta
    record = judge(run(mutate=stale_pose))
    assert any("differ from the independent" in p for p in record["problems"]), record["problems"]


def test_half_the_clouds_dropped_fails():
    record = judge(run(mutate=lambda i, pose, meta, raw: (apply(pose @ STATIC, raw) if i % 2 else None, meta)))
    assert any("published" in p for p in record["problems"]), record["problems"]


def test_other_drops_in_diag_fail():
    events = run()
    events.append(("diag", None, {"in": 61, "out": 60, "drops": {"no_pose": 1, "bad_frame": 1},
                                  "items": 61, "bytes": 0, "proc_ns": 5}))
    record = judge(events)
    assert any("bad_frame" in p for p in record["problems"]), record["problems"]


def test_stale_after_first_ok_fails():
    def late_stale(i, pose, meta, raw):
        if i == 40:
            meta = {**meta, "validity": "stale", "reason": "stale_pose"}
        return apply(pose @ STATIC, raw), meta
    record = judge(run(mutate=late_stale))
    assert any("stale" in p for p in record["problems"]), record["problems"]


def test_stale_before_first_ok_passes():
    def early_stale(i, pose, meta, raw):
        if i == 1:
            meta = {**meta, "validity": "stale", "reason": "stale_pose"}
        return apply(pose @ STATIC, raw), meta
    assert judge(run(mutate=early_stale))["problems"] == []


def test_missing_header_seq_fails_without_crash():
    def no_seq(i, pose, meta, raw):
        if i == 20:
            meta = {k: v for k, v in meta.items() if k != "header_seq"}
        return apply(pose @ STATIC, raw), meta
    record = judge(run(mutate=no_seq))
    assert any("header_seq" in p for p in record["problems"]), record["problems"]


def test_robot_that_does_not_move_fails():
    record = judge(run(static_robot=True))
    assert any("moved" in p for p in record["problems"]), record["problems"]


def test_late_ground_error_fails_even_with_good_median():
    def lift(i, pose, meta, raw):
        out = apply(pose @ STATIC, raw)
        if i == 50:
            out[:, 2] += 0.2
        return out, meta
    record = judge(run(mutate=lift))
    assert record["ground_err_m"]["median"] <= 0.03
    assert any("ground" in p for p in record["problems"]), record["problems"]


def test_dropped_expectation():
    events = [e for e in run() if e[0] != "cloud" and e[0] != "diag"]
    events.append(("diag", None, {"in": 61, "out": 0, "drops": {"bad_frame": 61}}))
    assert judge(events, "dropped")["problems"] == []
    assert judge(run(), "dropped")["problems"] != []


def test_static_chain_matches_extrinsics():
    np.testing.assert_allclose(STATIC[:3, 3], [0.0, -0.02, 0.4 + 0.0896])
    np.testing.assert_allclose(STATIC[:3, :3], np.eye(3))


def test_recompute_agrees_with_dora_transform():
    """The independent numpy chain gives the node library's answer."""
    tf = pytest.importorskip("dora.transform")
    frames = tf.FrameDict.load(str(HERE / "../../robots/mickrobot-sim/frames.yml"))
    ext = tf.Extrinsics.load(str(EXTRINSICS), frames)
    p, q = (1.5, -0.3, 0.0), (0.0, 0.0, math.sin(0.35), math.cos(0.35))
    pose = tf.pose_transform([*p, *q], "odom", "base_footprint", frames)
    transform, used = tf.resolve(ext, "rslidar", "odom", pose)
    assert used
    pts = np.array([[1.0, 2.0, 3.0, 0.5], [-4.0, 0.5, -0.4, 0.1]], dtype="<f4")
    lib = np.frombuffer(transform.apply_packed(pts.tobytes(), 4), dtype="<f4").reshape(-1, 4)
    np.testing.assert_allclose(apply(matrix(p, q) @ STATIC, pts), lib, atol=1e-5)
