"""Run-time assertions for the contract-core chain; writes a JSON record.

EXPECT=transformed: every cloud is in odom with increasing seq and validity ok
once a pose has arrived, and ground points sit at z = 0 +/- 0.03 m.
EXPECT=dropped: no cloud arrives; diag counts one bad_frame per input cloud.
"""
import json
import os
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
from dora import Node

RUN_S = float(os.environ.get("RUN_SECONDS", "60"))
EXPECT = os.environ["EXPECT"]
RECORD = Path(os.environ["RECORD"])
GROUND_TOL_M = 0.03


def points(value):
    return np.frombuffer(value[0].as_py()["points"], dtype="<f4").reshape(-1, 4)


def main():
    start = time.monotonic()
    seqs, validity, ground_err, raw_in, last_diag = [], Counter(), [], 0, {}
    problems = []
    for event in Node():
        if event["type"] != "INPUT":
            continue
        if event["id"] == "raw":
            raw_in += 1
        elif event["id"] == "diag":
            last_diag = json.loads(bytes(event["value"].to_numpy()).decode())
        elif event["id"] == "cloud":
            m = event["metadata"]
            if m.get("frame_id") != "odom":
                problems.append(f"cloud frame {m.get('frame_id')!r}")
            seqs.append(m["header_seq"])
            validity[m["validity"]] += 1
            p = points(event["value"])
            low = p[p[:, 2] <= np.quantile(p[:, 2], 0.10)]
            ground_err.append(float(np.median(np.abs(low[:, 2]))))
        if time.monotonic() - start >= RUN_S:
            break

    if EXPECT == "transformed":
        if not seqs:
            problems.append("no transformed clouds")
        if any(b <= a for a, b in zip(seqs, seqs[1:])):
            problems.append("seq not strictly increasing")
        if seqs and max(ground_err) > GROUND_TOL_M:
            problems.append(f"ground error {max(ground_err):.3f} m > {GROUND_TOL_M}")
        if set(validity) - {"ok", "stale"}:
            problems.append(f"unexpected validity {dict(validity)}")
        if validity.get("stale", 0) > 2:  # only before the first pose settles
            problems.append(f"{validity['stale']} stale clouds")
    elif EXPECT == "dropped":
        if seqs:
            problems.append(f"{len(seqs)} clouds published despite stripped frame_id")
        bad = last_diag.get("drops", {}).get("bad_frame", 0)
        if bad < last_diag.get("in", 1) or last_diag.get("in", 0) == 0:
            problems.append(f"diag {last_diag} does not count every input as bad_frame")

    record = {"expect": EXPECT, "run_s": RUN_S, "raw_in": raw_in, "published": len(seqs),
              "validity": dict(validity), "ground_err_m": {
                  "max": max(ground_err, default=None),
                  "median": float(np.median(ground_err)) if ground_err else None},
              "last_diag": last_diag, "problems": problems}
    RECORD.parent.mkdir(parents=True, exist_ok=True)
    RECORD.write_text(json.dumps(record, indent=2) + "\n")
    print("CHECKER_OK" if not problems else f"CHECKER_FAIL {problems}")
    sys.exit(0 if not problems else 1)


if __name__ == "__main__":
    main()
