"""
task_VTG/config.py
==================
M3/M4：放置区、固定对照列表、随机摆放范围。
"""

from __future__ import annotations

from typing import Any

# 桌面半高（与 B2 cube_small 一致）
CUBE_HALF = 0.025

# 安全/预抓取高度（米）
HOME_POS = (0.28, 0.0, 0.22)
PRE_CLEAR = 0.14
TRANSPORT_Z = 0.20
PLACE_Z = 0.06  # 放置下降目标 EE/物体附近高度（由 bridge 再钳位）

# 放置判定：最终 XY 距区中心 < 此值视为进区
ZONE_ACCEPT_M = 0.08

# 易碎区 A / 重物区 B（拉开距离，避免相撞）
ZONE_A_SOFT = (0.22, 0.28)   # soft → A
ZONE_B_HARD = (0.22, -0.28)  # hard → B

# M4 随机摆放（工作台可达区；避开放置区）
SPAWN_X_RANGE = (0.34, 0.42)
SPAWN_Y_RANGE = (-0.14, 0.14)
SPAWN_MIN_SEP = 0.10          # 两物最小间距
SPAWN_ZONE_CLEAR = 0.12       # 距放置区中心最小距离

# 视觉颜色外观（便于 HSV；物理仍用 B2 iron/sponge）
COLOR_RGBA = {
    "red": [0.85, 0.10, 0.10, 1.0],
    "yellow": [0.95, 0.85, 0.20, 1.0],
}

# 颜色 → 物体元数据（M4 检测标签映射）
COLOR_META: dict[str, dict[str, Any]] = {
    "red": {
        "id": "hard_red",
        "label": "IronBlock",
        "kind": "hard",
        "soft": False,
        "mass": 0.55,
        "color": "red",
    },
    "yellow": {
        "id": "soft_yellow",
        "label": "Sponge",
        "kind": "soft",
        "soft": True,
        "mass": 0.04,
        "color": "yellow",
    },
}

# 固定待抓列表（M3 对照；M4 默认由视觉生成）
OBJECTS: list[dict[str, Any]] = [
    {
        "id": "hard_red",
        "label": "IronBlock",
        "xy": (0.38, -0.10),
        "kind": "hard",
        "soft": False,
        "mass": 0.55,
    },
    {
        "id": "soft_yellow",
        "label": "Sponge",
        "xy": (0.38, 0.10),
        "kind": "soft",
        "soft": True,
        "mass": 0.04,
    },
]


# M6 重跟踪
RETRACK_XY_THRESH_M = 0.02     # XY 变化超过此值才更新目标
RETRACK_MISS_LIMIT = 3         # 连续丢检次数上限
RETRACK_NUDGE_M = 0.04         # demo 强制平移量（米）


def zone_xy_for_kind(kind: str) -> tuple[float, float]:
    """soft → ZONE_A，hard → ZONE_B。"""
    if kind == "soft":
        return ZONE_A_SOFT
    return ZONE_B_HARD


def object_pos3(obj: dict[str, Any]) -> list[float]:
    """物体桌面位姿 [x,y,z]（抓取高度固定 CUBE_HALF）。"""
    xy = obj["xy"]
    return [float(xy[0]), float(xy[1]), float(CUBE_HALF)]
