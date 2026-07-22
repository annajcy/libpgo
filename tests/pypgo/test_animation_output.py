import json
from pathlib import Path

from pypgo.animation import abc


class _FakeAnimationLoader:
    saved_to = None

    def load(self, config_path):
        self.config_path = config_path

    def save_abc(self, output_folder):
        type(self).saved_to = Path(output_folder)


def test_dump_animation_defaults_to_cwd_output(tmp_path, monkeypatch):
    config_path = tmp_path / "animation.json"
    config_path.write_text(json.dumps({"meshes": []}))
    monkeypatch.setattr(abc, "AnimationLoader", _FakeAnimationLoader)
    monkeypatch.chdir(tmp_path)

    abc.dump_animation(config_path)

    assert _FakeAnimationLoader.saved_to == Path("output")


def test_dump_animation_config_output_is_config_relative(tmp_path, monkeypatch):
    config_path = tmp_path / "animation.json"
    config_path.write_text(json.dumps({"meshes": [], "output-folder": "renders"}))
    monkeypatch.setattr(abc, "AnimationLoader", _FakeAnimationLoader)

    abc.dump_animation(config_path)

    assert _FakeAnimationLoader.saved_to == tmp_path / "renders"
