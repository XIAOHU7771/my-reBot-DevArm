"""
task_VTG/motion/approach.py
===========================
M3：侧向预热；M6：分段接近 + 可选重跟踪。
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np
import pybullet as p

from config import CUBE_HALF, RETRACK_MISS_LIMIT, RETRACK_XY_THRESH_M


def side_warmup(
    afc: Any,
    robot,
    arm_joints,
    ee_link,
    obj_pos,
    orn,
    home_pos,
) -> None:
    """先到目标同侧高位，降低从 home 直达时的对准漂移。"""
    side = np.asarray(home_pos, dtype=float).copy()
    obj_pos = np.asarray(obj_pos, dtype=float)
    side[0] = 0.30
    side[1] = 0.55 * float(obj_pos[1])
    side[2] = max(float(home_pos[2]), float(obj_pos[2]) + float(afc.PRE_CLEAR) + 0.02)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, side, orn, afc.OPEN,
            steps=160, allow_via=True,
        )
    except RuntimeError as e:
        print(f"  [侧向预热] {e}")


def _pin_at_real(afc: Any, body) -> np.ndarray:
    """钉在物体真实 XY（不把物体挪到估计点）。"""
    pos = np.array(p.getBasePositionAndOrientation(body)[0], dtype=float)
    pos[2] = float(CUBE_HALF)
    afc.pin_body(body, pos, afc.TABLE_ORN)
    return pos


def approach_with_retrack(
    afc: Any,
    robot,
    arm_joints,
    ee_link,
    body,
    obj_pos,
    orn,
    off_local,
    *,
    color: str = "red",
    camera_cfg: Any = None,
    enable_retrack: bool = False,
    nudge_fn: Callable[[int], None] | None = None,
    thresh_m: float = RETRACK_XY_THRESH_M,
) -> tuple[np.ndarray, int]:
    """
    分段接近；可选在预位后 nudge + 视觉重跟踪更新目标。

    返回 (最终 grasp_pos, retrack 触发次数)。
    """
    from tracking.retrack import grasp_xy_from_goal, log_retrack, maybe_retrack
    from vision.camera import CameraConfig

    if camera_cfg is None:
        camera_cfg = CameraConfig()

    goal = np.asarray(obj_pos, dtype=float).copy()
    goal[2] = float(CUBE_HALF)
    retrack_count = 0
    miss_streak = 0

    afc.set_robot_obj_collision(robot, body, enable=False)
    _pin_at_real(afc, body)

    def do_retrack(tag: str) -> None:
        nonlocal goal, retrack_count, miss_streak
        if not enable_retrack:
            return
        rr = maybe_retrack(
            color,
            (float(goal[0]), float(goal[1])),
            camera_cfg,
            thresh_m=thresh_m,
        )
        print(f"  --- retrack@{tag} ---")
        log_retrack(rr)
        if rr.reason == "detect_miss":
            miss_streak += 1
            return
        miss_streak = 0
        if rr.triggered and rr.xy_new is not None:
            goal = grasp_xy_from_goal(rr.xy_new)
            retrack_count += 1

    # --- 预抓取高位 ---
    orn = afc.grasp_orn_for_pos(goal)
    ee_g = afc.ee_for_mid(goal, orn, off_local)
    pre = ee_g.copy()
    pre[2] = goal[2] + float(afc.PRE_CLEAR)
    print(f"  预抓取（目标 {np.round(goal, 3)}）...")
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, pre, orn, afc.OPEN,
            steps=200, allow_via=True,
        )
    except RuntimeError as e:
        print(f"  [预抓取] {e}")

    if nudge_fn is not None:
        print("  [NUDGE] 平移物体以触发重跟踪...")
        nudge_fn(body)
        _pin_at_real(afc, body)

    do_retrack("pre")
    orn = afc.grasp_orn_for_pos(goal)

    if enable_retrack and miss_streak >= RETRACK_MISS_LIMIT:
        print(f"  [RETRACK] 连续丢检>={RETRACK_MISS_LIMIT}，抬高再搜")
        cur = afc.get_current_tcp(robot, ee_link)
        high = cur.copy()
        high[2] = max(float(cur[2]) + 0.08, float(afc.PRE_CLEAR) + 0.05)
        try:
            afc.move_tcp(
                robot, arm_joints, ee_link, high, orn, afc.OPEN,
                steps=100, allow_via=False,
            )
        except RuntimeError:
            pass
        miss_streak = 0
        do_retrack("rescue")
        orn = afc.grasp_orn_for_pos(goal)

    # --- 中段 ---
    ee_g = afc.ee_for_mid(goal, orn, off_local)
    mid = ee_g.copy()
    mid[2] = goal[2] + 0.5 * float(afc.PRE_CLEAR)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, mid, orn, afc.OPEN,
            steps=160, allow_via=False,
        )
    except RuntimeError as e:
        print(f"  [中段] {e}")

    do_retrack("mid")
    orn = afc.grasp_orn_for_pos(goal)

    # --- 下降到抓取高 ---
    ee_g = afc.ee_for_mid(goal, orn, off_local)
    print("  下降并对准指心...")
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, ee_g, orn, afc.OPEN,
            steps=220, allow_via=False,
        )
    except RuntimeError as e:
        print(f"  [下降] {e}")
        down = afc.get_current_tcp(robot, ee_link).copy()
        down[2] = max(float(ee_g[2]), float(afc.EE_Z_MIN))
        try:
            afc.move_tcp(
                robot, arm_joints, ee_link, down, orn, afc.OPEN,
                steps=160, allow_via=False,
            )
        except RuntimeError as e2:
            print(f"  [下降回退] {e2}")

    for it in range(8):
        fing, _, _ = afc.get_finger_mid(robot)
        err = goal - fing
        if float(np.linalg.norm(err[:2])) < 0.007:
            break
        corr = afc.ee_for_mid(goal, orn, off_local)
        corr[0] += err[0]
        corr[1] += err[1]
        corr[2] = max(float(ee_g[2]), float(afc.EE_Z_MIN))
        try:
            afc.move_tcp(
                robot, arm_joints, ee_link, corr, orn, afc.OPEN,
                steps=100 + 10 * it, allow_via=False,
            )
        except RuntimeError as e:
            print(f"  [对准{it}] 跳过: {e}")
            break

    fing, left, right = afc.get_finger_mid(robot)
    span = float(np.linalg.norm(right - left))
    mid_err_xy = float(np.linalg.norm((fing - goal)[:2]))
    _pin_at_real(afc, body)
    print(
        f"  指心={np.round(fing, 4)}  span={span:.3f}m  "
        f"mid_err_xy={mid_err_xy*1000:.1f}mm  retrack_n={retrack_count}"
    )
    return goal.copy(), retrack_count
