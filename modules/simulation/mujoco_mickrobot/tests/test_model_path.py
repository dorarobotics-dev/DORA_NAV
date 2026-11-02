from pathlib import Path

from mujoco_mickrobot.model_loader import MODEL_PATH, load_generated_model, resolve_model_path

from .conftest import build_test_model


def test_default_is_the_tracked_model():
    assert resolve_model_path({}) == MODEL_PATH
    assert resolve_model_path({"MUJOCO_MODEL_PATH": "  "}) == MODEL_PATH


def test_env_selects_a_generated_model(tmp_path):
    built = build_test_model(tmp_path)
    chosen = resolve_model_path({"MUJOCO_MODEL_PATH": str(built)})
    assert chosen == Path(built)
    assert load_generated_model(chosen).nq > 0
