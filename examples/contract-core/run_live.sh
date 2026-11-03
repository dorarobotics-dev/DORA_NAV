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
DORA_ROOT=${DORA_ROOT:-/home/demo/Public/github_dimos/dora}
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
trap 'rm -f "$here/$stripped"' EXIT
failed=()

# Runs one dataflow, tees its output to a log, and records a failure (without
# stopping) unless the log contains the marker. `timeout` is a backstop for a
# run that ignores --stop-after: TERM 60 s after it, KILL 15 s after that.
run_one() { # name, log, marker, dir, dataflow, stop-after (seconds)
  local name=$1 log=$2 marker=$3
  (cd "$4" && timeout -k 15 $(( $6 + 60 )) "$DORA" run "$5" --stop-after "$6s") 2>&1 | tee "$log" || true
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
