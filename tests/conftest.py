import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT / "go1_ros2_ws/src/go1_mock"),
    str(ROOT / "phanthymotus-driver/interview/go1_mcp"),
]


@pytest.fixture(scope="session")
def urdf_path():
    return ROOT / "go1_ros2_ws/src/go1_description/urdf/go1_model.urdf"
