"""
task_VTG/motion/retreat.py
==========================
M3：抬起离开 / 回安全位（调用 B2 move_tcp / recover_home）。
"""

from __future__ import annotations

from typing import Any

import numpy as np


def lift_clear(afc: Any, robot, arm_joints, ee_link, orn, width, z: float) -> None:
    """保持握力宽度，抬到安全高。"""
    cur = afc.get_current_tcp(robot, ee_link)
    high = cur.copy()
    high[2] = max(float(z), float(cur[2]) + 0.02)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, high, orn, width,
            steps=160, allow_via=True,
        )
    except RuntimeError as e:
        print(f"  [抬清] {e}")


def go_home(afc: Any, robot, arm_joints, ee_link, home_pos, home_orn) -> None:
    """回安全位。"""
    afc.recover_home(robot, arm_joints, ee_link, home_pos, home_orn)
