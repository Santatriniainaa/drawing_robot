"""Shared robot dimensions loaded from the installed geometry configuration."""

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


_CONFIG_PATH = (
    Path(get_package_share_directory("drawbot_control"))
    / "config"
    / "drawbot_geometry.yaml"
)
GEOMETRY: dict[str, Any] = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))

PEN_CONTACT = (
    GEOMETRY["base"]["thickness"] + GEOMETRY["pcb"]["thickness"]
    - (
        GEOMETRY["base"]["thickness"]
        + GEOMETRY["rail_x"]["height"]
        + GEOMETRY["beam_y"]["height"]
        + GEOMETRY["gantry_raise"]
        - GEOMETRY["pen"]["length"]
        - GEOMETRY["pen"]["tip_radius"]
    )
)
PEN_DOWN_POSITION = PEN_CONTACT - GEOMETRY["drawing"]["pen_compression"]
