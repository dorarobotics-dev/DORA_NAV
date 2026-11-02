"""Load the finished MuJoCo model without invoking offline conversion."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

import mujoco


MODULE_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = MODULE_ROOT / "models/mickrobot.xml"
MODEL_PATH_ENV = "MUJOCO_MODEL_PATH"
BUILD_INSTRUCTION = "python3 scripts/build_model.py"


def resolve_model_path(environ: Mapping[str, str] | None = None) -> Path:
    """The model to load: MUJOCO_MODEL_PATH if set and non-empty, else MODEL_PATH."""
    env = os.environ if environ is None else environ
    override = env.get(MODEL_PATH_ENV, "").strip()
    return Path(override) if override else MODEL_PATH


class ModelLoadError(RuntimeError):
    """Raised when the offline-generated model is unavailable or invalid."""


def load_generated_model(path: Path = MODEL_PATH) -> mujoco.MjModel:
    candidate = Path(path)
    if not candidate.is_file():
        raise ModelLoadError(f"MuJoCo model is missing: {candidate}; run `{BUILD_INSTRUCTION}` first")
    try:
        return mujoco.MjModel.from_xml_path(str(candidate))
    except Exception as exc:
        raise ModelLoadError(f"MuJoCo model is invalid: {candidate}; rerun `{BUILD_INSTRUCTION}`: {exc}") from exc
