#!/usr/bin/env bash
# Generate a loadable MuJoCo model into models/ (the tracked mickrobot.xml has
# absolute /home/mickrobot paths). good.yml points the sim at it via MUJOCO_MODEL_PATH.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
py=${PYTHON:-${VENV:-/home/demo/Public/github_dimos/.venv-exp}/bin/python}
mod="$here/../../modules/simulation/mujoco_mickrobot"
mkdir -p "$here/models"
"$py" - "$mod" "$here/models/mickrobot.xml" <<'PY'
import sys
from pathlib import Path
mod, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
sys.path.insert(0, str(mod / "scripts"))
from urdf_converter import convert_urdf
import mujoco
p = convert_urdf(mod / "models/mickrobot_3d.urdf", mod / "models/mujoco_extensions.xml",
                 mod / "models/test_world.xml", out)
m = mujoco.MjModel.from_xml_path(str(p))
print(f"model={p} nq={m.nq} nv={m.nv} nu={m.nu}")
PY
