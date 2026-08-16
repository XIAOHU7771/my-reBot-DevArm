"""
task_VTG/grasp/adaptive_bridge.py
=================================
M3：薄封装调用 task_B2 自适应力控（不重写 PID）。

提供：
  - import_b2_afc()
  - grasp_and_lift(...)  接近→力控闭合→抬升（不在原位松开）
  - place_held_object(...) 运到放置区→下降→张开
"""

from __future__ import annotations

import importlib.util
import os
import sys
import time
import traceback
from typing import Any

import numpy as np
import pybullet as p

from config import PLACE_Z, TRANSPORT_Z, zone_xy_for_kind
from motion.approach import approach_with_retrack, side_warmup
from motion.retreat import lift_clear


def import_b2_afc():
    """动态加载 task_B2/2.adaptive_force_control_grasp.py。"""
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    b2 = os.path.join(repo, "task_B2")
    if b2 not in sys.path:
        sys.path.insert(0, b2)
    path = os.path.join(b2, "2.adaptive_force_control_grasp.py")
    spec = importlib.util.spec_from_file_location("afc_b2_vtg", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 B2 力控模块: {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def grasp_and_lift(
    afc: Any,
    robot,
    arm_joints,
    ee_link,
    body,
    obj_pos,
    off_local,
    label: str,
    home_pos,
    *,
    soft_object: bool = False,
    mass: float = 0.15,
    save_force_log: bool = True,
    enable_retrack: bool = False,
    retrack_color: str = "red",
    camera_cfg: Any = None,
    nudge_fn: Any = None,
) -> dict:
    """
    接近 → 解冻 → 力控闭合 → 抬升。
    enable_retrack=True 时用分段接近 + 视觉重跟踪（M6）。
    """
    print(f"\n{'=' * 60}")
    print(f"  [BRIDGE] 抓取+抬升: {label}  retrack={enable_retrack}")
    print(f"{'=' * 60}")

    obj_pos = np.asarray(obj_pos, dtype=float).copy()
    obj_pos[2] = afc.CUBE_HALF
    obj_orn = afc.TABLE_ORN
    orn = afc.grasp_orn_for_pos(obj_pos)
    home_orn = p.getQuaternionFromEuler(afc.HOME_EULER)
    retrack_n = 0

    # 钉在真实位置；抓取目标用 obj_pos（可被 retrack 更新）
    real = np.array(p.getBasePositionAndOrientation(body)[0], dtype=float)
    real[2] = afc.CUBE_HALF
    afc.pin_body(body, real, obj_orn)
    afc.open_gripper(robot)
    afc.settle(12)

    side_warmup(afc, robot, arm_joints, ee_link, obj_pos, orn, home_pos)

    if enable_retrack or nudge_fn is not None:
        try:
            obj_pos, retrack_n = approach_with_retrack(
                afc, robot, arm_joints, ee_link, body, obj_pos, orn, off_local,
                color=retrack_color,
                camera_cfg=camera_cfg,
                enable_retrack=enable_retrack,
                nudge_fn=nudge_fn,
            )
            orn = afc.grasp_orn_for_pos(obj_pos)
        except RuntimeError as e:
            print(f"  [APPROACH/RETRACK] 失败: {e}")
            return _fail_result(label, soft_object, retrack_n=retrack_n)
    else:
        try:
            afc.approach_object(robot, arm_joints, ee_link, obj_pos, orn, off_local, body)
        except RuntimeError as e:
            print(f"  [APPROACH] 失败: {e}")
            return _fail_result(label, soft_object)

        mid, _, _ = afc.get_finger_mid(robot)
        if float(np.linalg.norm((mid - obj_pos)[:2])) > 0.015:
            print("  [对准] mid_err_xy 偏大，回位后二次接近...")
            afc.recover_home(robot, arm_joints, ee_link, home_pos, home_orn)
            side_warmup(afc, robot, arm_joints, ee_link, obj_pos, orn, home_pos)
            try:
                afc.approach_object(robot, arm_joints, ee_link, obj_pos, orn, off_local, body)
            except RuntimeError as e:
                print(f"  [APPROACH] 二次失败: {e}")
                return _fail_result(label, soft_object)

    # 解冻：用物体真实位姿
    real = np.array(p.getBasePositionAndOrientation(body)[0], dtype=float)
    real[2] = afc.CUBE_HALF
    afc.unfreeze_body(body, real, obj_orn, mass, soft=soft_object)
    afc.settle(6)

    try:
        width, info = afc.force_controlled_grasp(robot, arm_joints, body, label)
    except Exception as e:
        print(f"  [FORCE_GRASP] 异常: {e}")
        traceback.print_exc()
        return _fail_result(label, soft_object, retrack_n=retrack_n)

    if save_force_log:
        try:
            info["recorder"].save(f"vtg_m3_{label}")
        except Exception as e:
            print(f"  [日志] 保存跳过: {e}")

    # 抬升前保压（与 B2 grasp_one 同逻辑）
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    for _ in range(40):
        afc.drive_arm(robot, arm_joints, hold_arm, force=afc.HOLD_ARM_FORCE, pg=0.75)
        f = afc.read_force(robot, body)
        if info["f_target"] > 1.0:
            if f["total"] < 0.85 * info["f_target"]:
                width = max(afc.CLOSE_CENTER, width - 4e-5)
            elif f["total"] > info["f_target"] + 15.0:
                width = min(afc.OPEN, width + 2e-5)
        afc.set_gripper(robot, width, force=afc.GRIP_MOTOR_HOLD, keep_center=True)
        p.stepSimulation()
        time.sleep(afc.SIM_DT)
    info["final_force"] = afc.read_force_smooth(robot, body, n=5)["total"]

    if not soft_object:
        for link in (afc.LEFT_LINK, afc.RIGHT_LINK):
            p.changeDynamics(
                robot, link,
                lateralFriction=3.5, spinningFriction=0.25, rollingFriction=0.02,
            )

    print("\n=== [LIFT] 抬升验证 ===")
    try:
        lift = afc.lift_with_force_hold(
            robot, arm_joints, ee_link, body, orn, width, info["f_target"],
            force_control=True,
        )
    except RuntimeError as e:
        print(f"  [LIFT] 失败: {e}")
        lift = {"lifted": 0.0, "min_force": 0.0, "final_z": 0.0, "width": width}

    lifted_ok = lift["lifted"] > 0.06
    if soft_object or ("软" in str(info.get("kind", ""))):
        success = lifted_ok and (not info["crushed"])
    else:
        success = lifted_ok and info["final_force"] >= 0.4 * max(info["f_target"], 1.0)

    print(
        f"  lifted={lift['lifted']:.3f}m  success={success}  "
        f"kind={info.get('kind')} crushed={info['crushed']} retrack_n={retrack_n}"
    )

    if not success:
        _release_in_place(afc, robot, arm_joints, ee_link, body, real, orn, lift["width"])
        return {
            "label": label,
            "success": False,
            "kind": info.get("kind", "未知"),
            "f_target": info["f_target"],
            "lifted": lift["lifted"],
            "crushed": info["crushed"],
            "final_force": info["final_force"],
            "width": lift["width"],
            "orn": orn,
            "held": False,
            "retrack_n": retrack_n,
        }

    return {
        "label": label,
        "success": True,
        "kind": info.get("kind", "未知"),
        "f_target": info["f_target"],
        "lifted": lift["lifted"],
        "crushed": info["crushed"],
        "final_force": info["final_force"],
        "width": lift["width"],
        "orn": orn,
        "held": True,
        "retrack_n": retrack_n,
    }


def place_held_object(
    afc: Any,
    robot,
    arm_joints,
    ee_link,
    body,
    kind: str,
    orn,
    width: float,
) -> dict:
    """
    TRANSPORT → PLACE：抬到安全高 → 横移到区上方 → 下降 → 张开。
    """
    zone = zone_xy_for_kind(kind)
    print(f"\n=== [TRANSPORT/PLACE] kind={kind} zone={zone} ===")

    lift_clear(afc, robot, arm_joints, ee_link, orn, width, TRANSPORT_Z)

    cur = afc.get_current_tcp(robot, ee_link)
    above = cur.copy()
    above[0] = float(zone[0])
    above[1] = float(zone[1])
    above[2] = float(TRANSPORT_Z)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, above, orn, width,
            steps=220, allow_via=True,
        )
    except RuntimeError as e:
        print(f"  [TRANSPORT] {e}")
        return {"placed": False, "reason": str(e)}

    # 放置下降：用指心附近高度
    down = above.copy()
    down[2] = max(float(PLACE_Z), float(afc.EE_Z_MIN) + 0.02)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, down, orn, width,
            steps=200, allow_via=False,
        )
    except RuntimeError as e:
        print(f"  [PLACE 下降] {e}")

    for w in np.linspace(width, afc.OPEN, 70):
        afc.set_gripper(robot, float(w), force=300, keep_center=True)
        p.stepSimulation()
        time.sleep(afc.SIM_DT)

    afc.open_gripper(robot)
    afc.settle(15)

    # 抬起离开
    cur = afc.get_current_tcp(robot, ee_link)
    up = cur.copy()
    up[2] = float(TRANSPORT_Z)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, up, orn, afc.OPEN,
            steps=140, allow_via=False,
        )
    except RuntimeError as e:
        print(f"  [RETREAT] {e}")

    for link in (afc.LEFT_LINK, afc.RIGHT_LINK):
        p.changeDynamics(
            robot, link,
            lateralFriction=2.0, spinningFriction=0.1, rollingFriction=0.01,
        )

    pos, _ = p.getBasePositionAndOrientation(body)
    # 钉在落点附近，避免弹飞
    place_pos = [float(pos[0]), float(pos[1]), float(afc.CUBE_HALF)]
    afc.pin_body(body, place_pos, afc.TABLE_ORN)
    afc.set_robot_obj_collision(robot, body, enable=True)

    return {"placed": True, "final_pos": place_pos, "zone_xy": zone}


def _fail_result(label: str, soft_object: bool, retrack_n: int = 0) -> dict:
    return {
        "label": label,
        "success": False,
        "kind": "软/易碎" if soft_object else "未知",
        "f_target": 0.0,
        "lifted": 0.0,
        "crushed": False,
        "final_force": 0.0,
        "width": 0.08,
        "orn": None,
        "held": False,
        "retrack_n": retrack_n,
    }


def _release_in_place(afc, robot, arm_joints, ee_link, body, obj_pos, orn, width) -> None:
    """抓取失败时松开并钉回原位。"""
    try:
        for w in np.linspace(width, afc.OPEN, 60):
            afc.set_gripper(robot, float(w), force=300, keep_center=True)
            p.stepSimulation()
            time.sleep(afc.SIM_DT)
    except Exception:
        pass
    afc.open_gripper(robot)
    afc.pin_body(body, obj_pos, afc.TABLE_ORN)
    afc.set_robot_obj_collision(robot, body, enable=True)
