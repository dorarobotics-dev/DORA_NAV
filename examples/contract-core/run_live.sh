#!/usr/bin/env bash
# Live contract-core runs: good (60 s), stripped cloud frame (20 s), C header.
#
# Isolation: every run uses `dora run`, which in dora 1.1.0 runs the dataflow
# on an in-process daemon with NO coordinator (binaries/cli/src/command/run.rs).
# It never connects to or binds a coordinator port, its zenoh session listens on
# an ephemeral loopback port, and multicast scouting is off because no node here
# is dynamic (binaries/daemon/src/lib.rs, run_dataflow_with). The only fixed port
# it could bind is the dynamic-node listener, which is only opened for dynamic
# nodes; it is pinned to 6114 anyway so it can never contend with another
# daemon's default 53291. No `dora up/start/stop/destroy` is used, so a
# coordinator already running on the default port 6013 is never contacted.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
# Overridable: DORA_ROOT (the dora checkout), DORA (its CLI), VENV (python env
# with dora and the dora-frame-transform console script, put first on PATH).
DORA_ROOT=${DORA_ROOT:-/home/demo/Public/github_dorarobotics_dev/dora}
DORA=${DORA:-$DORA_ROOT/target/release/dora}
VENV=${VENV:-/home/demo/Public/github_dimos/.venv-exp}
CHEADER=$DORA_ROOT/examples/c-header

unset DORA_COORDINATOR_ADDR DORA_COORDINATOR_PORT CONDA_PREFIX
export DORA_DAEMON_LOCAL_LISTEN_PORT=6114
export VIRTUAL_ENV=$VENV
export PATH=$VENV/bin:$PATH
export PYTHONPATH=

"$DORA" --version
cd "$here"
# Always regenerated (~2 s), so a changed URDF or converter is never run stale.
PYTHON=$VENV/bin/python ./make_model.sh
mkdir -p records

stripped=.stripped.yml
failed=()
RUN_PID=

# Stops the running `dora run` the way Ctrl-C would. SIGINT is what dora run
# handles (its ctrlc handler stops the dataflow and the daemon stops each node,
# which runs in its own process group); SIGTERM would kill dora run outright
# and orphan the nodes. `timeout` forwards the INT and SIGKILLs dora run 20 s
# later if it has not exited; the group kill below is the last resort.
stop_run() {
  [[ -n $RUN_PID ]] || return 0
  kill -INT "$RUN_PID" 2>/dev/null || return 0
  for _ in $(seq 30); do kill -0 "$RUN_PID" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- "-$RUN_PID" 2>/dev/null || true
}
trap 'stop_run; rm -f "$here/$stripped"' EXIT
trap 'echo "interrupted; stopping dora run" >&2; exit 130' INT TERM

# Runs one dataflow, shows its output while writing it to a log, and records a
# failure (without stopping) unless the log contains the marker.
# `timeout` is a backstop for a run that ignores --stop-after: SIGINT 60 s
# after it, SIGKILL 20 s after that. It runs in the background in its own
# process group (so a terminal Ctrl-C reaches only this script), and the INT/TERM
# trap above stops it: nothing is left running after Ctrl-C.
run_one() { # name, log, marker, dir, dataflow, stop-after (seconds)
  local name=$1 log=$2 marker=$3
  : >"$log"
  (cd "$4" && exec timeout -s INT -k 20 $(( $6 + 60 )) "$DORA" run "$5" --stop-after "$6s") >"$log" 2>&1 &
  RUN_PID=$!
  tail -n +1 -f --pid="$RUN_PID" "$log" &
  local tail_pid=$!
  wait "$RUN_PID" || true
  RUN_PID=
  wait "$tail_pid" 2>/dev/null || true
  if grep -q "$marker" "$log"; then echo "PASS $name"; else echo "FAIL $name"; failed+=("$name"); fi
}

# 1. Good chain: 60 s of checking; stop-after leaves room for MuJoCo start-up.
run_one good records/contract-core.log CHECKER_OK "$here" good.yml 100

# 2. Sim strips frame_id from the cloud: frame-transform must drop every input.
# Kept next to good.yml so its relative paths still resolve.
sed -e 's/EXPECT: transformed/EXPECT: dropped/' \
    -e 's#RECORD: records/contract-core.json#RECORD: records/contract-core-stripped.json#' \
    -e 's/^\( *\)MUJOCO_GL: egl$/&\n\1SIM_STRIP_CLOUD_FRAME: "1"/' \
    -e 's/RUN_SECONDS: "60"/RUN_SECONDS: "20"/' good.yml > "$stripped"
grep -q 'SIM_STRIP_CLOUD_FRAME: "1"' "$stripped"
run_one stripped records/contract-core-stripped.log CHECKER_OK "$here" "$stripped" 60

# 3. C node sending a header through dora_send_output_with_params.
[[ -x $CHEADER/build/c_node ]] || "$CHEADER/build.sh"
run_one c-header "$here/records/c-header.log" C_HEADER_OK "$CHEADER" dataflow.yml 30

if (( ${#failed[@]} )); then echo "LIVE_RUNS_FAILED: ${failed[*]}"; exit 1; fi
echo "LIVE_RUNS_OK"
