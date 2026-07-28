import json
from pathlib import Path

import numpy as np

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


def test_abc_writer_forwards_frame_rate(tmp_path, monkeypatch):
    captured = {}

    def fake_dump_abc(
        filename, name, rest_positions, displacements, triangles, fps
    ):
        captured.update(
            filename=filename,
            name=name,
            num_frames=len(displacements),
            fps=fps,
        )

    monkeypatch.setattr(abc, "has_animation_io", lambda: True)
    monkeypatch.setattr(abc._core, "dump_abc", fake_dump_abc)

    abc.AbcWriter.dump(
        tmp_path / "animation.abc",
        "box",
        rest_positions=np.zeros(9),
        triangles=np.array([[0, 1, 2]]),
        displacements=[np.zeros(9), np.ones(9)],
        fps=500.0,
    )

    assert captured == {
        "filename": str(tmp_path / "animation.abc"),
        "name": "box",
        "num_frames": 2,
        "fps": 500.0,
    }
