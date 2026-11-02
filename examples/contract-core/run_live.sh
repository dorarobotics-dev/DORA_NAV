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
DORA=${DORA:-/home/demo/Public/github_dimos/dora/target/release/dora}
VENV=/home/demo/Public/github_dimos/.venv-exp
CHEADER=/home/demo/Public/github_dimos/dora/examples/c-header

unset DORA_COORDINATOR_ADDR DORA_COORDINATOR_PORT CONDA_PREFIX
export DORA_DAEMON_LOCAL_LISTEN_PORT=6114
export VIRTUAL_ENV=$VENV
export PATH=$VENV/bin:$PATH
export PYTHONPATH=

"$DORA" --version
cd "$here"
[[ -f models/mickrobot.xml ]] || ./make_model.sh
mkdir -p records

stripped=.stripped.yml
trap 'rm -f "$here/$stripped"' EXIT

# 1. Good chain: 60 s of checking; stop-after leaves room for MuJoCo start-up.
"$DORA" run good.yml --stop-after 100s 2>&1 | tee records/contract-core.log
grep -q CHECKER_OK records/contract-core.log

# 2. Sim strips frame_id from the cloud: frame-transform must drop every input.
# Kept next to good.yml so its relative paths still resolve.
sed -e 's/EXPECT: transformed/EXPECT: dropped/' \
    -e 's#RECORD: records/contract-core.json#RECORD: records/contract-core-stripped.json#' \
    -e 's/^\( *\)MUJOCO_GL: egl$/&\n\1SIM_STRIP_CLOUD_FRAME: "1"/' \
    -e 's/RUN_SECONDS: "60"/RUN_SECONDS: "20"/' good.yml > "$stripped"
grep -q 'SIM_STRIP_CLOUD_FRAME: "1"' "$stripped"
"$DORA" run "$stripped" --stop-after 60s 2>&1 | tee records/contract-core-stripped.log
grep -q CHECKER_OK records/contract-core-stripped.log

# 3. C node sending a header through dora_send_output_with_params.
[[ -x $CHEADER/build/c_node ]] || "$CHEADER/build.sh"
(cd "$CHEADER" && "$DORA" run dataflow.yml --stop-after 30s) 2>&1 | tee records/c-header.log
grep -q C_HEADER_OK records/c-header.log
echo "LIVE_RUNS_OK"
