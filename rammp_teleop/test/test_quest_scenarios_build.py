"""Every scripted scenario builds and has the shape MockPoseSource expects."""

import numpy as np
import pytest

from rammp_teleop.quest import scenarios
from rammp_teleop.quest.pose_source import MockPoseSource


@pytest.mark.parametrize("name", sorted(scenarios.SCENARIOS))
def test_scenario_builds_and_replays(name):
    frames = scenarios.get(name)
    assert len(frames) > 10
    src = MockPoseSource(script=frames)
    pose, btn = src.read()
    assert pose.shape == (4, 4)
    assert np.isclose(np.linalg.det(pose[:3, :3]), 1.0)
    assert isinstance(btn.grip, bool)
    assert 0.0 <= btn.trigger <= 1.0


def test_unknown_scenario_raises():
    with pytest.raises(KeyError):
        scenarios.get("does-not-exist")
