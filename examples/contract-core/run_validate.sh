#!/usr/bin/env bash
# Accept good.yml; refuse each bad-*.yml with its reason.
set -uo pipefail
here=$(cd "$(dirname "$0")" && pwd)
# Overridable: DORA_ROOT (the dora checkout), DORA (its CLI), VENV (holds the
# dora-frame-transform console script the dataflows name by bare command).
DORA_ROOT=${DORA_ROOT:-/home/demo/Public/github_dorarobotics_dev/dora}
dora=${DORA:-$DORA_ROOT/target/release/dora}
VENV=${VENV:-/home/demo/Public/github_dimos/.venv-exp}
export PATH=$VENV/bin:$PATH
fail=0
expect() { # file, expected-exit (0|1), grep pattern
  out=$("$dora" validate "$here/$1" 2>&1); code=$?
  if [[ $2 == 0 && $code != 0 ]] || [[ $2 == 1 && $code == 0 ]] || ! grep -q -- "$3" <<<"$out"; then
    echo "FAIL $1 (exit $code)"; echo "$out" | sed 's/^/    /'; fail=1
  else echo "ok   $1"; fi
}
expect good.yml 0 "Frames OK."
expect bad-frame-mismatch.yml 1 "requires frame 'base_link'"
expect bad-unknown-frame.yml 1 "use 'rslidar'"
expect bad-missing-calibration.yml 1 "never assumed to be identity"
expect bad-two-dynamic-edges.yml 1 "crosses 2 dynamic edges"
exit $fail
