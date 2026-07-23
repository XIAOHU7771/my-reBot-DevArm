"""
simulation2/utils 公共接口
==========================
为三个实验脚本提供：
  - robot  : URDF 加载与关节/link 索引
  - gripper: 夹爪开合、接触力与关节反力读数
  - ik     : TCP 逆运动学与平滑移动
  - scene  : 可选默认桌面场景

本包只做 re-export，不改变各子模块逻辑。
"""

from .robot import load_robot, URDF_PATH
from .gripper import (
    init_gripper,
    set_gripper,
    set_gripper_positions,
    balance_gripper_forces,
    width_to_joint_positions,
    open_gripper,
    close_gripper,
    get_gripper_contact_forces,
    get_gripper_joint_reaction_forces,
    get_gripper_force_readings,
    OPEN,
    CLOSE,
    CLOSE_CENTER,
)
from .ik import init_ik, solve_ik, move_tcp, get_current_tcp, TCP_OFFSET
from .scene import load_scene

__all__ = [
    "load_robot",
    "URDF_PATH",
    "init_gripper",
    "set_gripper",
    "set_gripper_positions",
    "balance_gripper_forces",
    "width_to_joint_positions",
    "open_gripper",
    "close_gripper",
    "get_gripper_contact_forces",
    "get_gripper_joint_reaction_forces",
    "get_gripper_force_readings",
    "OPEN",
    "CLOSE",
    "CLOSE_CENTER",
    "init_ik",
    "solve_ik",
    "move_tcp",
    "get_current_tcp",
    "TCP_OFFSET",
    "load_scene",
]
