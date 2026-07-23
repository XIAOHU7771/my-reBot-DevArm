"""
1.Force_Sensor_Simulation.py
============================================================
虚拟力传感器部署（Tactile / Force Sensor Simulation）

功能：
  1. 在夹爪指内侧部署 PyBullet 力/触觉接口：
     - getContactPoints → 接触点法向力（正压力 Normal Force）
     - enableJointForceTorqueSensor + getJointState → 关节反作用力
  2. 闭合抓取过程实时打印正压力
  3. 按仿真时间记录时序，导出 CSV + 曲线图

运行（在 simulation2 目录下）:
  python 1.Force_Sensor_Simulation.py
  python 1.Force_Sensor_Simulation.py --direct --no-show
  python 1.Force_Sensor_Simulation.py --cube 0.4 0.2 0.04 --mass 0.15 --friction 2.5 --target-force 12

GUI 模式会并排打开 PyBullet 仿真窗 + 力曲线实时窗（横轴为仿真时间）。
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
import traceback

import numpy as np
import pybullet as p
import pybullet_data

from utils.robot import load_robot
from utils.gripper import (
    init_gripper,
    LEFT_JOINT,
    RIGHT_JOINT,
    LEFT_LINK,
    RIGHT_LINK,
    LEFT_UPPER,
    RIGHT_UPPER,
    CLOSE,
    set_gripper,
    set_gripper_positions,
    balance_gripper_forces,
    width_to_joint_positions,
    open_gripper,
    get_gripper_force_readings,
    OPEN,
    CLOSE_CENTER,
)
from utils.ik import (
    init_ik,
    solve_ik,
    get_current_tcp,
    LOWER_LIMITS,
    UPPER_LIMITS,
)

# -------------------- 仿真参数（可由 CLI 覆盖）--------------------
SIM_DT = 1.0 / 240.0
HOME_EULER = [0.0, 0.0, 0.0]
FORCE_THRESHOLD = 120.0
TARGET_GRASP_FORCE = 12.0
TARGET_EACH_FORCE = 6.0
CONTACT_THRESHOLD = 1.5
FORCE_BALANCE_DEADBAND = 2.0
GRIPPER_OPEN_WIDTH = OPEN
CLOSE_STEP = 0.00012
SETTLE_STEPS = 40
ALIGN_TOL = 0.008
MAX_ALIGN_ITERS = 4
IK_OK_ERR = 0.020
TRACK_OK_ERR = 0.012
EE_Z_MIN = 0.010
PRE_GRASP_CLEARANCE = 0.14
AXIS_ALIGN_MIN = 0.96
CUBE_DRIFT_ABORT = 0.028
GRIP_MOTOR_SOFT = 120
GRIP_MOTOR_FIRM = 550
FORCE_SMOOTH_STEPS = 4
MAX_REGRIP = 3
BALANCE_GAIN_BASE = 0.00015
SLIP_LIN_VEL = 0.08
SLIP_ANG_VEL = 1.5
LIFT_BALANCE_EVERY = 8
HOLD_SETTLE_STEPS = 18   # 插值到位后短稳定，避免长时间停顿感
SEARCH_IK_ITERS = 250    # 姿态搜索用快速 IK，不做雅可比精调

ARM_FORCE = 2000
ARM_POSITION_GAIN = 0.35
ARM_VELOCITY_GAIN = 1.0
ARM_POSITION_GAIN_FINE = 0.55   # 末端精定位增益
JOINT_DAMPING = 0.12
LINK_LINEAR_DAMPING = 0.04
LINK_ANGULAR_DAMPING = 0.04


def apply_runtime_config(args):
    """用命令行参数覆盖模块级抓取/力控阈值。"""
    global TARGET_GRASP_FORCE, TARGET_EACH_FORCE, FORCE_THRESHOLD
    TARGET_GRASP_FORCE = float(args.target_force)
    TARGET_EACH_FORCE = 0.5 * TARGET_GRASP_FORCE
    FORCE_THRESHOLD = max(FORCE_THRESHOLD, TARGET_GRASP_FORCE * 8.0)


# =====================================================================
# 力/触觉传感器
# =====================================================================

def read_normal_forces(robot, cube=None):
    """读取夹爪指内侧正压力（contact normalForce）。"""
    left_all = p.getContactPoints(bodyA=robot, linkIndexA=LEFT_LINK)
    right_all = p.getContactPoints(bodyA=robot, linkIndexA=RIGHT_LINK)
    left_n = sum(pt[9] for pt in left_all)
    right_n = sum(pt[9] for pt in right_all)
    out = {"left": left_n, "right": right_n, "total": left_n + right_n}

    if cube is not None:
        left_c = p.getContactPoints(bodyA=robot, linkIndexA=LEFT_LINK, bodyB=cube)
        right_c = p.getContactPoints(bodyA=robot, linkIndexA=RIGHT_LINK, bodyB=cube)
        out["cube_left"] = sum(pt[9] for pt in left_c)
        out["cube_right"] = sum(pt[9] for pt in right_c)
        out["cube_total"] = out["cube_left"] + out["cube_right"]
        out["has_cube_contact"] = len(left_c) > 0 or len(right_c) > 0
        out["both_fingers_contact"] = len(left_c) > 0 and len(right_c) > 0
    return out


def settle_contacts(n=FORCE_SMOOTH_STEPS):
    """力读数前多步仿真，让接触约束收敛、抑制尖峰噪声。"""
    for _ in range(max(1, int(n))):
        p.stepSimulation()


def read_normal_forces_smoothed(robot, cube=None, steps=FORCE_SMOOTH_STEPS):
    """多步仿真后取接触力（可选均值）。"""
    samples = []
    for _ in range(max(1, int(steps))):
        p.stepSimulation()
        samples.append(read_normal_forces(robot, cube))
    keys_num = ["left", "right", "total"]
    if cube is not None:
        keys_num += ["cube_left", "cube_right", "cube_total"]
    out = dict(samples[-1])
    for k in keys_num:
        out[k] = float(np.mean([s.get(k, 0.0) for s in samples]))
    if cube is not None:
        out["has_cube_contact"] = any(s.get("has_cube_contact", False) for s in samples)
        out["both_fingers_contact"] = any(
            s.get("both_fingers_contact", False) for s in samples[-max(1, len(samples) // 2):]
        )
    return out


def read_joint_reaction(robot):
    """读取关节力传感器反作用力（需 enableJointForceTorqueSensor）。"""
    left_js = p.getJointState(robot, LEFT_JOINT)
    right_js = p.getJointState(robot, RIGHT_JOINT)
    left_f = np.array(left_js[2][:3])
    right_f = np.array(right_js[2][:3])
    return {
        "left_pos": left_js[0],
        "right_pos": right_js[0],
        "left_mag": float(np.linalg.norm(left_f)),
        "right_mag": float(np.linalg.norm(right_f)),
        "total_mag": float(np.linalg.norm(left_f) + np.linalg.norm(right_f)),
    }


def read_cube_state(cube):
    """Cube 位姿与速度。"""
    pos, orn = p.getBasePositionAndOrientation(cube)
    lin, ang = p.getBaseVelocity(cube)
    pos = np.asarray(pos, dtype=float)
    orn = np.asarray(orn, dtype=float)
    lin = np.asarray(lin, dtype=float)
    ang = np.asarray(ang, dtype=float)
    return {
        "cube_x": float(pos[0]),
        "cube_y": float(pos[1]),
        "cube_z": float(pos[2]),
        "cube_qx": float(orn[0]),
        "cube_qy": float(orn[1]),
        "cube_qz": float(orn[2]),
        "cube_qw": float(orn[3]),
        "cube_vx": float(lin[0]),
        "cube_vy": float(lin[1]),
        "cube_vz": float(lin[2]),
        "cube_wx": float(ang[0]),
        "cube_wy": float(ang[1]),
        "cube_wz": float(ang[2]),
        "cube_lin_speed": float(np.linalg.norm(lin)),
        "cube_ang_speed": float(np.linalg.norm(ang)),
    }


def adaptive_balance_gain(imbalance, base=BALANCE_GAIN_BASE):
    """力差越大增益略增，并钳位，避免过冲松指。"""
    scale = 1.0 + 0.12 * abs(float(imbalance))
    return float(np.clip(base * scale, base * 0.5, base * 2.5))


# =====================================================================
# 运动 / 场景
# =====================================================================

def configure_physics(robot, arm_joints):
    """高精度求解器 + 关节/连杆阻尼，抑制冲击与数值颤振。"""
    # 不同 PyBullet 版本关键字略有差异，逐项尝试
    base = dict(
        fixedTimeStep=SIM_DT,
        numSolverIterations=150,
        numSubSteps=4,
        enableConeFriction=1,
    )
    optional = [
        dict(erp=0.2, contactERP=0.2, frictionERP=0.2),
        dict(cfm=1e-5),
        dict(enableFileCaching=0),
    ]
    params = dict(base)
    for block in optional:
        trial = dict(params)
        trial.update(block)
        try:
            p.setPhysicsEngineParameter(**trial)
            params = trial
        except TypeError:
            continue
    try:
        p.setPhysicsEngineParameter(**params)
    except TypeError:
        p.setPhysicsEngineParameter(
            fixedTimeStep=SIM_DT,
            numSolverIterations=150,
            numSubSteps=4,
        )

    for link in list(arm_joints) + [LEFT_LINK, RIGHT_LINK]:
        try:
            p.changeDynamics(
                robot, link,
                jointDamping=JOINT_DAMPING,
                linearDamping=LINK_LINEAR_DAMPING,
                angularDamping=LINK_ANGULAR_DAMPING,
            )
        except Exception:
            p.changeDynamics(robot, link, jointDamping=JOINT_DAMPING)

    print(
        f"[物理] solverIter=150 subSteps=4 jointDamping={JOINT_DAMPING} "
        f"lin/angDamp={LINK_LINEAR_DAMPING}/{LINK_ANGULAR_DAMPING}"
    )


def settle(n=SETTLE_STEPS):
    """推进仿真 n 步，用于加载或动作后的短暂稳定。"""
    for _ in range(n):
        p.stepSimulation()
        time.sleep(SIM_DT)


def _smoothstep5(t: float) -> float:
    """
    5 次多项式平滑插值（C2）：端点速度、加速度均为 0，
    比 3 阶 smoothstep 更弱冲击，减轻力尖峰。
    s(t) = 6t^5 - 15t^4 + 10t^3
    """
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def clamp_arm_angles(angles):
    """臂关节角度限位钳位。"""
    a = np.asarray(angles, dtype=float).copy()
    lo = np.asarray(LOWER_LIMITS, dtype=float)
    hi = np.asarray(UPPER_LIMITS, dtype=float)
    return np.clip(a, lo, hi)


def clamp_gripper_positions(left_pos, right_pos):
    """夹爪棱柱关节行程钳位。"""
    return (
        float(np.clip(left_pos, CLOSE, LEFT_UPPER)),
        float(np.clip(right_pos, CLOSE, RIGHT_UPPER)),
    )


def validate_ik_solution(robot, arm_joints, ee_link, angles, tcp_target,
                         restore_angles=None, left_pos=None, right_pos=None):
    """
    IK 解合法性：有限值、关节限位钳位、应用后 TCP 误差检查。
    返回 (clamped_angles, tcp_error_m)
    """
    angles = np.asarray(angles, dtype=float)
    if not np.all(np.isfinite(angles)):
        raise RuntimeError("IK 解含 NaN/Inf，非法")

    clamped = clamp_arm_angles(angles)
    if np.max(np.abs(clamped - angles)) > 1e-4:
        print(
            f"  [IK] 关节限位钳位 Δq_max="
            f"{np.max(np.abs(clamped - angles)):.4f} rad"
        )

    for j, ang in zip(arm_joints, clamped):
        p.resetJointState(robot, j, float(ang))
    if left_pos is not None and right_pos is not None:
        lp, rp = clamp_gripper_positions(left_pos, right_pos)
        p.resetJointState(robot, LEFT_JOINT, lp)
        p.resetJointState(robot, RIGHT_JOINT, rp)

    tcp = get_current_tcp(robot, ee_link)
    err = float(np.linalg.norm(tcp - np.asarray(tcp_target, dtype=float)))

    if restore_angles is not None:
        for j, ang in zip(arm_joints, restore_angles):
            p.resetJointState(robot, j, float(ang))
        if left_pos is not None and right_pos is not None:
            lp, rp = clamp_gripper_positions(left_pos, right_pos)
            p.resetJointState(robot, LEFT_JOINT, lp)
            p.resetJointState(robot, RIGHT_JOINT, rp)

    return clamped, err


def _drive_arm(robot, arm_joints, angles, force=ARM_FORCE,
               position_gain=None, velocity_gain=None):
    """对臂关节下发位置电机指令（可调 force / positionGain / velocityGain）。"""
    angles = clamp_arm_angles(angles)
    pg = ARM_POSITION_GAIN if position_gain is None else position_gain
    vg = ARM_VELOCITY_GAIN if velocity_gain is None else velocity_gain
    for j, ang in zip(arm_joints, angles):
        p.setJointMotorControl2(
            robot, j, p.POSITION_CONTROL,
            targetPosition=float(ang),
            force=force,
            positionGain=pg,
            velocityGain=vg,
        )


def _solve_ik_hidden(robot, arm_joints, ee_link, tcp_target, start_angles,
                     target_orn, left_pos, right_pos, quiet=False,
                     max_err=None):
    """关闭渲染下求 IK，校验合法性后恢复起点。返回 (goal, ik_err, tcp_target)。"""
    max_err = IK_OK_ERR * 2.0 if max_err is None else max_err
    left_pos, right_pos = clamp_gripper_positions(left_pos, right_pos)
    tcp_target = np.asarray(tcp_target, dtype=float).copy()
    tcp_target[2] = max(float(tcp_target[2]), EE_Z_MIN)

    try:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    except Exception:
        pass

    raw_goal = solve_ik(
        robot, arm_joints, ee_link, tcp_target,
        start_angles, target_orn=target_orn, quiet=quiet,
    )
    goal, ik_err = validate_ik_solution(
        robot, arm_joints, ee_link, raw_goal, tcp_target,
        restore_angles=start_angles, left_pos=left_pos, right_pos=right_pos,
    )

    for j, ang in zip(arm_joints, start_angles):
        p.resetJointState(robot, j, float(ang))
    p.resetJointState(robot, LEFT_JOINT, float(left_pos))
    p.resetJointState(robot, RIGHT_JOINT, float(right_pos))
    set_gripper_positions(robot, left_pos, right_pos, force=500)

    try:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    except Exception:
        pass

    if ik_err > max_err:
        print(f"  [IK警告] 解后 TCP 误差 {ik_err * 1000:.1f} mm > {max_err * 1000:.0f} mm")
    return np.asarray(goal, dtype=float), float(ik_err), tcp_target


def _interpolate_joints(robot, arm_joints, start, goal, left_pos, right_pos,
                        steps, fine_tail=True):
    """5 次多项式关节插值；末段提高增益改善跟踪。"""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    joint_travel = float(np.linalg.norm(goal - start))
    steps = int(max(steps, joint_travel / 0.0028))
    fine_start = int(steps * 0.72) if fine_tail else steps + 1

    for i in range(1, steps + 1):
        s = _smoothstep5(i / steps)
        q = start + s * (goal - start)
        if i >= fine_start:
            _drive_arm(
                robot, arm_joints, q,
                position_gain=ARM_POSITION_GAIN_FINE, velocity_gain=1.2,
            )
        else:
            _drive_arm(robot, arm_joints, q)
        set_gripper_positions(robot, left_pos, right_pos, force=500)
        p.stepSimulation()
        time.sleep(SIM_DT)

    for _ in range(HOLD_SETTLE_STEPS):
        _drive_arm(
            robot, arm_joints, goal,
            position_gain=ARM_POSITION_GAIN_FINE, velocity_gain=1.2,
        )
        set_gripper_positions(robot, left_pos, right_pos, force=500)
        p.stepSimulation()
        time.sleep(SIM_DT)
    return steps


def move_tcp(robot, arm_joints, ee_link, tcp_target, gripper_width,
             target_orn, steps=280, allow_via=True, max_corr=2):
    """
    平滑移动到目标 TCP：
      - 大落差时先经上方途经点，再 quintic 下降
      - 末段提高位置增益
      - 跟踪误差过大则短程纠正
    """
    tcp_target = np.asarray(tcp_target, dtype=float).copy()
    tcp_target[2] = max(float(tcp_target[2]), EE_Z_MIN)
    left_pos, right_pos = width_to_joint_positions(gripper_width, keep_center=True)
    left_pos, right_pos = clamp_gripper_positions(left_pos, right_pos)

    cur = get_current_tcp(robot, ee_link)
    if allow_via and (cur[2] - tcp_target[2]) > 0.05:
        via = tcp_target.copy()
        via[2] = max(tcp_target[2] + 0.10, cur[2] - 0.02)
        if via[2] > tcp_target[2] + 0.04:
            print(f"  [途经] Z={via[2]:.3f} → 再下降到 {tcp_target[2]:.3f}")
            move_tcp(
                robot, arm_joints, ee_link, via, gripper_width, target_orn,
                steps=max(160, steps // 2), allow_via=False, max_corr=1,
            )

    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    goal, ik_err, tcp_target = _solve_ik_hidden(
        robot, arm_joints, ee_link, tcp_target, start, target_orn,
        left_pos, right_pos,
    )
    if ik_err > IK_OK_ERR * 3.0:
        raise RuntimeError(
            f"IK 解不可用: TCP 误差 {ik_err * 1000:.1f} mm，目标 {np.round(tcp_target, 4)}"
        )

    used = _interpolate_joints(
        robot, arm_joints, start, goal, left_pos, right_pos, steps,
    )
    final = get_current_tcp(robot, ee_link)
    err = float(np.linalg.norm(final - tcp_target))

    for c in range(max_corr):
        if err <= TRACK_OK_ERR:
            break
        print(f"  [纠正{c+1}] 跟踪误差 {err*1000:.1f} mm，短程重试...")
        start = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
        goal, ik_err, tcp_target = _solve_ik_hidden(
            robot, arm_joints, ee_link, tcp_target, start, target_orn,
            left_pos, right_pos, quiet=True,
        )
        if ik_err > IK_OK_ERR * 3.0:
            break
        used += _interpolate_joints(
            robot, arm_joints, start, goal, left_pos, right_pos,
            steps=max(120, int(err / 0.002)),
        )
        final = get_current_tcp(robot, ee_link)
        err = float(np.linalg.norm(final - tcp_target))

    print(f"  TCP -> {np.round(final, 4)}, 误差 {err * 1000:.2f} mm  (steps={used})")
    if err > TRACK_OK_ERR * 2.0:
        print(f"  [警告] TCP 跟踪偏差 {err*1000:.1f} mm，交由对齐阶段补偿")
    return final


def recover_arm_home(robot, arm_joints, ee_link, home_orn):
    """候选失败后回到安全高位；电机失败则 IK 复位。"""
    home = np.array([0.30, 0.0, 0.22], dtype=float)
    try:
        move_tcp(
            robot, arm_joints, ee_link, home,
            GRIPPER_OPEN_WIDTH, home_orn, steps=180, allow_via=False, max_corr=2,
        )
        return
    except Exception as e:
        print(f"  [恢复] 平滑回位失败({e})，使用 IK 复位")
    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    left_pos, right_pos = width_to_joint_positions(GRIPPER_OPEN_WIDTH, keep_center=True)
    try:
        goal, _, _ = _solve_ik_hidden(
            robot, arm_joints, ee_link, home, start, home_orn,
            left_pos, right_pos, quiet=True,
        )
        for j, ang in zip(arm_joints, goal):
            p.resetJointState(robot, j, float(ang))
        set_gripper_positions(robot, left_pos, right_pos, force=500)
        for _ in range(30):
            _drive_arm(robot, arm_joints, goal, position_gain=ARM_POSITION_GAIN_FINE)
            set_gripper_positions(robot, left_pos, right_pos, force=500)
            p.stepSimulation()
            time.sleep(SIM_DT)
    except Exception as e:
        print(f"  [恢复] IK 复位也失败: {e}")


def get_finger_geometry(robot):
    """返回左右指世界坐标、指心、开口轴单位向量与开口跨度。"""
    # 与 EE 一致使用 worldLinkFramePosition[4]，避免 COM[0] 与连杆原点偏差
    left = np.array(p.getLinkState(robot, LEFT_LINK)[4], dtype=float)
    right = np.array(p.getLinkState(robot, RIGHT_LINK)[4], dtype=float)
    mid = (left + right) / 2.0
    axis = right - left
    span = float(np.linalg.norm(axis))
    axis = axis / (span + 1e-9)
    return left, right, mid, axis, span


def calibrate_finger_offset_local(robot, ee_link):
    """标定指心相对 EE 的局部坐标系偏移，供后续 mid→EE 目标换算。"""
    ee_pos = np.array(p.getLinkState(robot, ee_link)[4], dtype=float)
    ee_orn = p.getLinkState(robot, ee_link)[5]
    _, _, mid, _, _ = get_finger_geometry(robot)
    off_world = mid - ee_pos
    inv_orn = p.invertTransform([0.0, 0.0, 0.0], ee_orn)[1]
    off_local = np.array(p.rotateVector(inv_orn, off_world.tolist()), dtype=float)
    print(f"  指心相对 EE 偏移(局部): {np.round(off_local, 4)}")
    print(f"  指心相对 EE 偏移(世界): {np.round(off_world, 4)}")
    return off_local


def ee_target_for_finger_mid(mid_world, orn, off_local):
    """由期望指心世界坐标与姿态，换算 EE/TCP 目标位置。"""
    off_world = np.array(p.rotateVector(orn, np.asarray(off_local, dtype=float).tolist()))
    ee = np.asarray(mid_world, dtype=float) - off_world
    ee[2] = max(float(ee[2]), EE_Z_MIN)
    return ee


def cube_between_fingers(cube_pos, left, right, mid, axis, margin=0.018):
    """沿开口轴物体在两指之间，且垂直开口方向的偏移不过大。"""
    cube_pos = np.asarray(cube_pos, dtype=float)
    t_l = float(np.dot(left - mid, axis))
    t_r = float(np.dot(right - mid, axis))
    t_c = float(np.dot(cube_pos - mid, axis))
    if t_l * t_r >= 0:
        return False
    if abs(t_c) > margin:
        return False
    radial = (cube_pos - mid) - t_c * axis
    return float(np.linalg.norm(radial)) <= 0.022


def _grasp_yaw_candidates(cube_pos):
    """按物体 XY 生成 yaw（覆盖 y=0、x≈0 等特殊位置）。"""
    x, y = float(cube_pos[0]), float(cube_pos[1])
    phi = float(np.arctan2(y, x)) if (abs(x) + abs(y)) > 1e-6 else 0.0
    prefer_open_x = abs(y) >= abs(x)
    if prefer_open_x:
        yaws = [
            np.pi / 2, -np.pi / 2,
            phi + np.pi / 2, phi - np.pi / 2,
            0.0, np.pi, phi, 0.4, -0.4,
        ]
    else:
        yaws = [
            0.0, 0.4, -0.4,
            phi, np.pi / 2, -np.pi / 2,
            phi + np.pi / 4, phi - np.pi / 4, np.pi,
        ]
    uniq = []
    for yaw in yaws:
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        if not any(abs(yaw - u) < 0.05 for u in uniq):
            uniq.append(float(yaw))
    return uniq


def _priority_euler_pairs(cube_pos):
    """按可达性启发式排序 (pitch, yaw)，优先搜索最可能成功的姿态。"""
    mid = np.asarray(cube_pos, dtype=float)
    cx, cy = abs(float(mid[0])), abs(float(mid[1]))
    prefer_open_x = cy >= cx
    yaws = _grasp_yaw_candidates(mid)
    if prefer_open_x:
        pitches = [0.08, 0.15, 0.0, 0.25, 0.35]
    else:
        pitches = [0.0, 0.08, 0.15, 0.25]
    pairs = []
    for pitch in pitches:
        for yaw in yaws:
            yaw_to_pi2 = min(
                abs(abs(float(yaw)) - 0.5 * np.pi),
                abs(abs(float(yaw)) - 1.5 * np.pi),
            )
            if prefer_open_x:
                score = 0.45 * yaw_to_pi2 + 0.12 * abs(float(pitch) - 0.08)
            else:
                score = 0.35 * abs(float(yaw)) + 0.12 * abs(float(pitch))
            if cx < 0.10 and cy > 0.15:
                score += 0.55 * yaw_to_pi2
            pairs.append((score, float(pitch), float(yaw)))
    pairs.sort(key=lambda t: t[0])
    return [(p, y) for _, p, y in pairs]


def solve_ik_fast(robot, arm_joints, ee_link, tcp_target, current_arm_angles,
                  target_orn, max_iters=SEARCH_IK_ITERS, refine_iters=40):
    """
    姿态搜索专用：DLS 初值 + 少量雅可比精调（比正式 solve_ik 快数倍）。
    返回 (angles, tcp_error_m)。
    """
    from utils.ik import refine_joints, TCP_OFFSET

    tcp_target = np.asarray(tcp_target, dtype=float)
    left_cur = float(p.getJointState(robot, LEFT_JOINT)[0])
    right_cur = float(p.getJointState(robot, RIGHT_JOINT)[0])
    all_current = list(np.asarray(current_arm_angles, dtype=float)) + [left_cur, right_cur]
    full_lower = list(LOWER_LIMITS) + [0.0, 0.0]
    full_upper = list(UPPER_LIMITS) + [LEFT_UPPER, RIGHT_UPPER]
    offset_world = np.array(p.rotateVector(target_orn, TCP_OFFSET.tolist()), dtype=float)
    ee_target = tcp_target - offset_world

    result = p.calculateInverseKinematics(
        robot,
        ee_link,
        ee_target.tolist(),
        targetOrientation=target_orn,
        lowerLimits=full_lower,
        upperLimits=full_upper,
        jointRanges=[u - l for u, l in zip(full_upper, full_lower)],
        restPoses=all_current,
        currentPositions=all_current,
        maxNumIterations=int(max_iters),
        solver=p.IK_DLS,
    )
    angles = clamp_arm_angles([result[j] for j in arm_joints])
    for j, ang in zip(arm_joints, angles):
        p.resetJointState(robot, j, float(ang))
    tcp = get_current_tcp(robot, ee_link)
    err0 = float(np.linalg.norm(tcp - tcp_target))
    if err0 < 0.006:
        return angles, err0

    angles, _ = refine_joints(
        robot, arm_joints, ee_link, tcp_target, target_orn, angles,
        max_iter=int(refine_iters), tolerance=1e-4, lr=0.8, quiet=True,
    )
    angles = clamp_arm_angles(angles)
    for j, ang in zip(arm_joints, angles):
        p.resetJointState(robot, j, float(ang))
    tcp = get_current_tcp(robot, ee_link)
    err = float(np.linalg.norm(tcp - tcp_target))
    if err > max(err0 * 2.0, 0.012):
        return clamp_arm_angles([result[j] for j in arm_joints]), err0
    return angles, err


def select_grasp_orientation(robot, arm_joints, ee_link, cube_pos, off_local,
                             top_k=4, cube=None):
    """
    快速搜索可达侧夹姿态：优先序 + 轻量 IK + 早停。
    不推进仿真、不移动物体，避免可见停顿与 cube 闪烁。
    """
    mid_tgt = np.asarray(cube_pos, dtype=float).copy()
    pairs = _priority_euler_pairs(mid_tgt)

    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    left_pos = float(p.getJointState(robot, LEFT_JOINT)[0])
    right_pos = float(p.getJointState(robot, RIGHT_JOINT)[0])
    left_pos, right_pos = clamp_gripper_positions(left_pos, right_pos)

    try:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    except Exception:
        pass

    t0 = time.perf_counter()
    cands = []
    tried = 0
    for pitch, yaw in pairs:
        tried += 1
        euler = [0.0, float(pitch), float(yaw)]
        orn = p.getQuaternionFromEuler(euler)
        ee_tgt = ee_target_for_finger_mid(mid_tgt, orn, off_local)

        for j, ang in zip(arm_joints, start):
            p.resetJointState(robot, j, float(ang))
        p.resetJointState(robot, LEFT_JOINT, left_pos)
        p.resetJointState(robot, RIGHT_JOINT, right_pos)

        # 只求 grasp 位姿（省掉 pre 二次精调）
        ang_g, ik_err = solve_ik_fast(
            robot, arm_joints, ee_link, ee_tgt, start, orn,
            max_iters=SEARCH_IK_ITERS, refine_iters=36,
        )
        if ik_err > IK_OK_ERR:
            continue

        left, right, mid, axis, span = get_finger_geometry(robot)
        mid_err = float(np.linalg.norm(mid - mid_tgt))
        horiz = abs(axis[2]) < 0.10
        between = cube_between_fingers(mid_tgt, left, right, mid, axis)
        ax0, ax1 = abs(float(axis[0])), abs(float(axis[1]))
        axis_dom = max(ax0, ax1)
        axis_sec = min(ax0, ax1)

        if span < 0.08 or mid_err > 0.024:
            continue
        if axis_dom < 0.96 or axis_sec > 0.28:
            continue
        if not horiz or not between:
            continue

        cost = 4.0 * ik_err + 3.0 * mid_err
        cost += 0.18 * abs(float(pitch))
        cost += 0.12 * axis_sec
        cost -= 0.04 * axis_dom
        cx, cy = abs(float(mid_tgt[0])), abs(float(mid_tgt[1]))
        prefer_open_x = cy >= cx
        yaw_to_pi2 = min(
            abs(abs(float(yaw)) - 0.5 * np.pi),
            abs(abs(float(yaw)) - 1.5 * np.pi),
        )
        if prefer_open_x:
            cost += 0.45 * yaw_to_pi2
            cost += 0.12 * abs(float(pitch) - 0.08)
            cost -= 0.10 * ax0
        else:
            cost += 0.35 * abs(float(yaw))
            cost += 0.12 * abs(float(pitch))
            cost -= 0.10 * ax1

        cands.append({
            "cost": cost,
            "euler": euler,
            "orn": orn,
            "ee_tgt": ee_tgt.copy(),
            "ik_err": ik_err,
            "mid_err": mid_err,
            "axis": axis.copy(),
            "span": span,
            "horiz": horiz,
            "between": between,
        })
        if len(cands) >= top_k:
            break

    for j, ang in zip(arm_joints, start):
        p.resetJointState(robot, j, float(ang))
    p.resetJointState(robot, LEFT_JOINT, left_pos)
    p.resetJointState(robot, RIGHT_JOINT, right_pos)
    set_gripper_positions(robot, left_pos, right_pos, force=500)

    try:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    except Exception:
        pass

    dt_ms = (time.perf_counter() - t0) * 1000.0
    if not cands:
        raise RuntimeError(
            "未找到可用抓取姿态（请检查物体是否在工作空间内，如 x≈0 时 |y| 是否过大）"
        )

    cands.sort(key=lambda c: c["cost"])
    uniq = []
    for c in cands:
        if any(
            abs(c["euler"][1] - u["euler"][1]) < 0.05
            and abs(c["euler"][2] - u["euler"][2]) < 0.08
            for u in uniq
        ):
            continue
        uniq.append(c)
        if len(uniq) >= top_k:
            break

    print(f"  找到 {len(uniq)} 个候选（试了 {tried} 组, {dt_ms:.0f} ms）")
    for i, c in enumerate(uniq):
        print(
            f"    [{i}] euler={np.round(c['euler'], 3)} "
            f"IK={c['ik_err']*1000:.1f}mm mid={c['mid_err']*1000:.1f}mm "
            f"axis={np.round(c['axis'], 2)}"
        )
    return uniq


def snap_finger_mid(robot, arm_joints, ee_link, mid_tgt, target_orn,
                    gripper_width=GRIPPER_OPEN_WIDTH, max_iters=2):
    """
    用短程 quintic 插值把指心拉到 mid_tgt（无 reset 跳变）。
    接近阶段应已关闭整机-Cube 碰撞，避免推开物体。
    """
    mid_tgt = np.asarray(mid_tgt, dtype=float)
    left_pos, right_pos = width_to_joint_positions(gripper_width, keep_center=True)
    last_err = 1.0
    for k in range(max_iters):
        _, _, mid, _, _ = get_finger_geometry(robot)
        err = mid_tgt - mid
        last_err = float(np.linalg.norm(err))
        if last_err <= ALIGN_TOL:
            break
        ee_now = get_current_tcp(robot, ee_link)
        ee_tgt = ee_now + err
        ee_tgt[2] = max(float(ee_tgt[2]), EE_Z_MIN)
        start = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
        goal, ik_err, ee_tgt = _solve_ik_hidden(
            robot, arm_joints, ee_link, ee_tgt, start, target_orn,
            left_pos, right_pos, quiet=True, max_err=IK_OK_ERR * 3.0,
        )
        if ik_err > IK_OK_ERR * 3.0:
            print(f"  [指心复位] 迭代{k} IK差 {ik_err*1000:.1f}mm，停止")
            break
        steps = max(60, int(last_err / 0.0015))
        _interpolate_joints(
            robot, arm_joints, start, goal, left_pos, right_pos, steps,
            fine_tail=True,
        )
        _, _, mid, _, _ = get_finger_geometry(robot)
        last_err = float(np.linalg.norm(mid_tgt - mid))
        print(f"  [指心复位] 迭代{k} mid_err={last_err*1000:.1f}mm")
        if last_err > 0.05 and k >= 1:
            break
    _, _, mid, _, _ = get_finger_geometry(robot)
    return mid, float(np.linalg.norm(mid_tgt - mid))


def align_finger_mid_to_cube(robot, arm_joints, ee_link, cube_pos, target_orn,
                             gripper_width=GRIPPER_OPEN_WIDTH):
    """迭代微调臂位，使指心对准立方体中心（侧夹对准）。"""
    mid, err_n = snap_finger_mid(
        robot, arm_joints, ee_link, cube_pos, target_orn, gripper_width,
    )
    if err_n <= ALIGN_TOL:
        return mid, err_n

    for i in range(MAX_ALIGN_ITERS):
        _, _, mid, _, _ = get_finger_geometry(robot)
        err = np.asarray(cube_pos, dtype=float) - mid
        err_n = float(np.linalg.norm(err))
        print(f"  对齐[{i}] 指心误差={err_n*1000:.1f}mm  mid={np.round(mid, 4)}")
        if err_n <= ALIGN_TOL:
            return mid, err_n
        ee_now = get_current_tcp(robot, ee_link)
        tgt = ee_now + err
        tgt[2] = max(float(tgt[2]), EE_Z_MIN)
        try:
            move_tcp(
                robot, arm_joints, ee_link, tgt,
                gripper_width, target_orn, steps=100, allow_via=False, max_corr=1,
            )
        except RuntimeError as e:
            print(f"  [对齐] 电机微调失败({e})，改用短程插值")
            mid, err_n = snap_finger_mid(
                robot, arm_joints, ee_link, cube_pos, target_orn, gripper_width,
                max_iters=2,
            )
            return mid, err_n
    _, _, mid, _, _ = get_finger_geometry(robot)
    return mid, float(np.linalg.norm(np.asarray(cube_pos, dtype=float) - mid))


def verify_cube_in_grasp(robot, cube, max_mid_err=0.025):
    """抬升前确认物体仍在两指之间且未大幅漂移。"""
    cp = np.array(p.getBasePositionAndOrientation(cube)[0], dtype=float)
    left, right, mid, axis, span = get_finger_geometry(robot)
    mid_err = float(np.linalg.norm(cp - mid))
    between = cube_between_fingers(cp, left, right, mid, axis)
    return between and mid_err <= max_mid_err and span > 0.03, {
        "cube": cp, "mid": mid, "mid_err": mid_err, "between": between, "axis": axis,
    }


def stash_body(body, stash_pos=(2.0, 2.0, 1.0)):
    """接近时移走物体，避免碰撞阻挡跟踪。"""
    pos, orn = p.getBasePositionAndOrientation(body)
    p.resetBasePositionAndOrientation(body, stash_pos, orn)
    p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
    return np.array(pos, dtype=float), orn


def restore_body(body, pos, orn, mass, friction):
    """将物体复位到给定位姿，清零速度并恢复质量/摩擦。"""
    p.resetBasePositionAndOrientation(
        body, np.asarray(pos, dtype=float).tolist(), orn,
    )
    p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
    p.changeDynamics(
        body, -1,
        mass=float(mass),
        lateralFriction=float(friction),
        spinningFriction=0.1,
        rollingFriction=0.01,
    )


def set_robot_cube_collision(robot, cube, enable=True):
    """开关整机与 Cube 的碰撞。"""
    flag = 1 if enable else 0
    n = p.getNumJoints(robot)
    for link in range(-1, n):
        p.setCollisionFilterPair(robot, cube, link, -1, flag)


def set_finger_only_cube_collision(robot, cube):
    """仅左右指与 Cube 碰撞，避免手掌/臂杆把物体撞飞。"""
    n = p.getNumJoints(robot)
    for link in range(-1, n):
        p.setCollisionFilterPair(robot, cube, link, -1, 0)
    p.setCollisionFilterPair(robot, cube, LEFT_LINK, -1, 1)
    p.setCollisionFilterPair(robot, cube, RIGHT_LINK, -1, 1)


def load_cube(position, mass=0.15, friction=2.0):
    """加载可配置质量/摩擦的桌面立方体测试物体。"""
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    cube = p.loadURDF("cube_small.urdf", basePosition=position)
    p.changeDynamics(
        cube, -1,
        mass=mass,
        lateralFriction=float(friction),
        spinningFriction=0.1,
        rollingFriction=0.01,
        restitution=0.0,
        linearDamping=0.04,
        angularDamping=0.04,
    )
    p.changeVisualShape(cube, -1, rgbaColor=[0.2, 0.6, 0.2, 1])
    return cube


def detect_slip(cube, cube_z0, contact_total, lift_progress_ok,
                lin_thresh=SLIP_LIN_VEL, ang_thresh=SLIP_ANG_VEL):
    """
    滑落判定：接触力不足 + 高度未跟上，或水平/转动速度异常大。
    lift_progress_ok: 物体是否已明显抬离桌面。
    """
    st = read_cube_state(cube)
    height_ok = (st["cube_z"] - cube_z0) >= 0.03
    force_lost = contact_total < CONTACT_THRESHOLD
    # 抬升中水平速度过大且高度未跟上 → 滑出
    lateral = float(np.hypot(st["cube_vx"], st["cube_vy"]))
    spinning = st["cube_ang_speed"] > ang_thresh
    sliding = lateral > lin_thresh and not height_ok
    falling = force_lost and not height_ok and not lift_progress_ok
    return falling or sliding or (spinning and force_lost), st


# =====================================================================
# 记录与绘图
# =====================================================================

class LiveForcePlotter:
    """
    与 PyBullet GUI 同步的实时力曲线窗口。
    横坐标为仿真时间 (s)；在 GUI 模式下与仿真并排刷新。
    """

    def __init__(self, enabled=True, update_every=6, window_xy=(960, 40),
                 figsize=(9.5, 9.0)):
        """初始化实时力曲线窗口与数据缓冲。"""
        self.enabled = bool(enabled)
        self.update_every = max(1, int(update_every))
        self.window_xy = window_xy
        self.figsize = figsize
        self._fig = None
        self._axes = None
        self._lines = {}
        self._event_lines = []
        self._drawn_events = 0
        self._last_n = 0
        self._title = None
        if self.enabled:
            self._setup()

    def _setup(self):
        import matplotlib.pyplot as plt

        plt.ion()
        self._fig, self._axes = plt.subplots(
            4, 1, figsize=self.figsize, sharex=True,
            gridspec_kw={"height_ratios": [1.15, 1.0, 0.9, 0.85]},
        )
        try:
            self._fig.canvas.manager.set_window_title("力传感器实时曲线")
        except Exception:
            pass
        self._place_window()

        ax0, ax1, ax2, ax3 = self._axes
        self._lines["c_left"], = ax0.plot([], [], color="C0", lw=1.3, label="左指正压力")
        self._lines["c_right"], = ax0.plot([], [], color="C3", lw=1.3, label="右指正压力")
        self._lines["c_total"], = ax0.plot([], [], color="C2", lw=2.0, label="总正压力")
        ax0.axhline(FORCE_THRESHOLD, color="orange", ls="--", lw=1.0,
                    label=f"上限 {FORCE_THRESHOLD:.0f}N")
        ax0.axhline(TARGET_GRASP_FORCE, color="C2", ls=":", lw=1.0, alpha=0.8,
                    label=f"目标 {TARGET_GRASP_FORCE:.0f}N")
        ax0.set_ylabel("正压力 (N)")
        ax0.set_title("夹爪指内侧正压力（实时）")
        ax0.legend(loc="upper left", fontsize=8, ncol=2)
        ax0.grid(True, alpha=0.3)

        self._lines["r_left"], = ax1.plot([], [], color="C0", lw=1.2, label="左关节反力")
        self._lines["r_right"], = ax1.plot([], [], color="C3", lw=1.2, label="右关节反力")
        self._lines["r_total"], = ax1.plot([], [], color="C4", lw=1.8, label="反力合计")
        ax1.set_ylabel("反力 (N)")
        ax1.set_title("关节力传感器")
        ax1.legend(loc="upper left", fontsize=8, ncol=2)
        ax1.grid(True, alpha=0.3)

        self._lines["cube_f"], = ax2.plot([], [], color="darkgreen", lw=2.0, label="Cube 正压力")
        ax2.axhline(CONTACT_THRESHOLD, color="gray", ls="--", lw=1.0,
                    label=f"接触阈值 {CONTACT_THRESHOLD:.1f}N")
        ax2.set_ylabel("Cube 力 (N)")
        ax2.set_title("物体表面正压力")
        ax2.legend(loc="upper left", fontsize=8)
        ax2.grid(True, alpha=0.3)

        self._lines["width"], = ax3.plot([], [], color="purple", lw=1.5, label="夹爪宽度")
        self._lines["cube_z"], = ax3.plot([], [], color="C1", lw=1.3, label="Cube Z")
        ax3.set_xlabel("时间 (s)")
        ax3.set_ylabel("宽度 / 高度 (m)")
        ax3.set_title("夹爪行程与 Cube 高度")
        ax3.legend(loc="upper right", fontsize=8)
        ax3.grid(True, alpha=0.3)

        self._fig.tight_layout()
        self._fig.canvas.draw_idle()
        try:
            self._fig.canvas.flush_events()
        except Exception:
            pass
        plt.pause(0.05)
        print(f"[曲线] 实时窗口已打开（与仿真 GUI 同步，横轴=时间）")

    def _place_window(self):
        """尽量把曲线窗口放到 PyBullet GUI 右侧，避免遮挡。"""
        if self._fig is None:
            return
        x, y = self.window_xy
        try:
            mng = self._fig.canvas.manager
            # TkAgg
            if hasattr(mng, "window"):
                win = mng.window
                if hasattr(win, "wm_geometry"):
                    win.wm_geometry(f"+{x}+{y}")
                    return
                if hasattr(win, "setGeometry"):
                    w = int(self.figsize[0] * 100)
                    h = int(self.figsize[1] * 100)
                    win.setGeometry(x, y, w, h)
                    return
            # Qt
            if hasattr(mng, "window") and hasattr(mng.window(), "move"):
                mng.window().move(x, y)
        except Exception:
            pass

    def update(self, recorder, force=False):
        """追加一帧力数据并刷新曲线显示。"""
        if not self.enabled or self._fig is None:
            return
        n = len(recorder.records)
        if n == 0:
            return
        if (not force) and (n - self._last_n) < self.update_every:
            return

        times = [r["time"] for r in recorder.records]
        self._lines["c_left"].set_data(times, [r["contact_left"] for r in recorder.records])
        self._lines["c_right"].set_data(times, [r["contact_right"] for r in recorder.records])
        self._lines["c_total"].set_data(times, [r["contact_total"] for r in recorder.records])
        self._lines["r_left"].set_data(times, [r["reaction_left"] for r in recorder.records])
        self._lines["r_right"].set_data(times, [r["reaction_right"] for r in recorder.records])
        self._lines["r_total"].set_data(times, [r["reaction_total"] for r in recorder.records])
        self._lines["cube_f"].set_data(times, [r["cube_total"] for r in recorder.records])
        self._lines["width"].set_data(times, [r["gripper_width"] for r in recorder.records])
        self._lines["cube_z"].set_data(times, [r.get("cube_z", 0.0) for r in recorder.records])

        t_now = times[-1]
        phase = recorder.records[-1].get("phase", "")
        self._axes[0].set_title(f"夹爪指内侧正压力（实时）  t={t_now:.2f}s  [{phase}]")

        # 新事件竖线
        while self._drawn_events < len(recorder.events):
            t_ev, label = recorder.events[self._drawn_events]
            for ax in self._axes:
                ln = ax.axvline(t_ev, color="k", ls=":", alpha=0.45, lw=1.0)
                self._event_lines.append(ln)
            self._axes[3].text(
                t_ev, self._axes[3].get_ylim()[0], label,
                rotation=90, va="bottom", ha="right", fontsize=7, alpha=0.8,
            )
            self._drawn_events += 1

        for ax in self._axes:
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=True)
            # 横轴始终以时间为准，留一点右边距
            if times:
                ax.set_xlim(0.0, max(times[-1] * 1.05, 0.5))

        self._axes[3].set_xlabel("时间 (s)")
        try:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            pass
        self._last_n = n

    def finalize(self, recorder=None, hold=False):
        """仿真结束后做一次完整刷新。"""
        if not self.enabled or self._fig is None:
            return
        if recorder is not None:
            self.update(recorder, force=True)
        try:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            pass
        if hold:
            import matplotlib.pyplot as plt
            print("[曲线] 实时窗口保持打开（可与仿真对照）；关闭后继续保存最终图...")
            try:
                plt.show(block=False)
            except Exception:
                pass

    def close(self):
        """关闭实时绘图窗口。"""
        if self._fig is None:
            return
        import matplotlib.pyplot as plt
        try:
            plt.close(self._fig)
        except Exception:
            pass
        self._fig = None


class ForceRecorder:
    """按仿真时间记录力/宽度等时序，并导出 CSV 与静态图。"""

    CSV_FIELDS = [
        "step", "time", "phase", "gripper_width",
        "joint_pos_left", "joint_pos_right",
        "contact_left", "contact_right", "contact_total",
        "cube_left", "cube_right", "cube_total",
        "has_cube_contact", "both_fingers_contact", "force_imbalance",
        "reaction_left", "reaction_right", "reaction_total",
        "cube_x", "cube_y", "cube_z",
        "cube_qx", "cube_qy", "cube_qz", "cube_qw",
        "cube_vx", "cube_vy", "cube_vz",
        "cube_wx", "cube_wy", "cube_wz",
        "cube_lin_speed", "cube_ang_speed",
    ]

    def __init__(self, sim_dt=SIM_DT, live_plotter=None):
        """创建空记录表，绑定仿真步长。"""
        self.sim_dt = sim_dt
        self.sim_step = 0
        self.records = []
        self.events = []
        self.live_plotter = live_plotter

    @property
    def t(self):
        """当前已记录的仿真时间 (s)。"""
        return self.sim_step * self.sim_dt

    def mark(self, label):
        """打相位/事件标记，便于事后对齐曲线。"""
        self.events.append((self.t, label))
        print(f"[事件] t={self.t:.3f}s  {label}")
        if self.live_plotter is not None:
            self.live_plotter.update(self, force=True)

    def log(self, robot, cube, phase="closing", smooth=True):
        """追加一行采样（力、宽度、相位、物体状态等）。"""
        if smooth:
            contact = read_normal_forces_smoothed(robot, cube, FORCE_SMOOTH_STEPS)
        else:
            settle_contacts(FORCE_SMOOTH_STEPS)
            contact = read_normal_forces(robot, cube)
        reaction = read_joint_reaction(robot)
        bundled = get_gripper_force_readings(robot)
        cube_st = read_cube_state(cube)
        width = reaction["right_pos"]

        row = {
            "step": self.sim_step,
            "time": self.t,
            "phase": phase,
            "gripper_width": width,
            "joint_pos_left": reaction["left_pos"],
            "joint_pos_right": reaction["right_pos"],
            "contact_left": contact["left"],
            "contact_right": contact["right"],
            "contact_total": contact["total"],
            "cube_left": contact.get("cube_left", 0.0),
            "cube_right": contact.get("cube_right", 0.0),
            "cube_total": contact.get("cube_total", 0.0),
            "has_cube_contact": contact.get("has_cube_contact", False),
            "both_fingers_contact": contact.get("both_fingers_contact", False),
            "force_imbalance": abs(
                contact.get("cube_left", 0.0) - contact.get("cube_right", 0.0)
            ),
            "reaction_left": reaction["left_mag"],
            "reaction_right": reaction["right_mag"],
            "reaction_total": reaction["total_mag"],
            "utils_contact_total": bundled["contact"]["total"],
            "utils_reaction_total": bundled["reaction"]["total_magnitude"],
        }
        row.update(cube_st)
        self.records.append(row)
        self.sim_step += 1
        if self.live_plotter is not None:
            self.live_plotter.update(self)
        return self.records[-1]

    def save_csv(self, path):
        """将记录写入 force_data/*.csv。"""
        fields = self.CSV_FIELDS
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            for r in self.records:
                row = {k: r.get(k, "") for k in fields}
                row["has_cube_contact"] = int(bool(row["has_cube_contact"]))
                row["both_fingers_contact"] = int(bool(row["both_fingers_contact"]))
                w.writerow(row)
        print(f"[保存] CSV -> {path}")

    def plot(self, path, show=True):
        """绘制并保存力时序曲线图。"""
        import matplotlib.pyplot as plt

        if not self.records:
            print("[警告] 无数据可绘制")
            return

        times = [r["time"] for r in self.records]
        fig, axes = plt.subplots(
            5, 1, figsize=(11, 12), sharex=True,
            gridspec_kw={"height_ratios": [1.15, 1.0, 1.0, 0.9, 0.85]},
        )
        try:
            fig.canvas.manager.set_window_title("力传感器曲线（最终）")
        except Exception:
            pass

        ax = axes[0]
        ax.plot(times, [r["contact_left"] for r in self.records],
                label="左指正压力", color="C0", lw=1.2)
        ax.plot(times, [r["contact_right"] for r in self.records],
                label="右指正压力", color="C3", lw=1.2)
        ax.plot(times, [r["contact_total"] for r in self.records],
                label="总正压力", color="C2", lw=2)
        ax.axhline(FORCE_THRESHOLD, color="orange", ls="--",
                   label=f"上限 {FORCE_THRESHOLD}N")
        ax.axhline(TARGET_GRASP_FORCE, color="C2", ls=":",
                   label=f"目标 {TARGET_GRASP_FORCE}N")
        ax.set_ylabel("正压力 (N)")
        ax.set_title("虚拟触觉传感器 — 夹爪指内侧正压力")
        ax.legend(loc="upper left", fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(times, [r["reaction_left"] for r in self.records],
                label="左关节反力", color="C0", lw=1.2)
        ax.plot(times, [r["reaction_right"] for r in self.records],
                label="右关节反力", color="C3", lw=1.2)
        ax.plot(times, [r["reaction_total"] for r in self.records],
                label="反力合计", color="C4", lw=2)
        ax.set_ylabel("反力 (N)")
        ax.set_title("关节力传感器 — getJointState reactionForces")
        ax.legend(loc="upper left", fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)

        ax = axes[2]
        cube_f = [r["cube_total"] for r in self.records]
        ax.plot(times, cube_f, label="Cube 正压力", color="darkgreen", lw=2)
        ax.axhline(CONTACT_THRESHOLD, color="gray", ls="--",
                   label=f"接触阈值 {CONTACT_THRESHOLD}N")
        ax.fill_between(times, 0, cube_f, alpha=0.2, color="green")
        ax.set_ylabel("Cube 力 (N)")
        ax.set_title("物体表面正压力（抓取判定）")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[3]
        ax.plot(times, [r.get("cube_z", 0) for r in self.records],
                label="Cube Z", color="C1", lw=1.5)
        ax.plot(times, [r.get("cube_lin_speed", 0) for r in self.records],
                label="|v|", color="C5", lw=1.0)
        ax.set_ylabel("高度 / 速度")
        ax.set_title("Cube 高度与线速度")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[4]
        width = [r["gripper_width"] for r in self.records]
        ax.plot(times, width, label="夹爪宽度", color="purple", lw=1.5)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("宽度 (m)")
        ax.set_title("夹爪开合行程")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        if times:
            for a in axes:
                a.set_xlim(times[0], times[-1] if times[-1] > times[0] else times[0] + 0.5)

        for t_ev, label in self.events:
            for a in axes:
                a.axvline(t_ev, color="k", ls=":", alpha=0.4)
            y0 = min(width) if width else 0
            axes[4].text(t_ev, y0, label, rotation=90,
                         va="bottom", ha="right", fontsize=8)

        fig.tight_layout()
        fig.savefig(path, dpi=150)
        print(f"[绘图] PNG -> {path}")
        if show:
            # 若已有实时窗口，最终图另开；阻塞直到用户关闭
            plt.show(block=True)
        else:
            plt.close(fig)


# =====================================================================
# 主流程
# =====================================================================

def parse_args():
    """解析命令行：GUI/DIRECT、目标力、立方体位姿等。"""
    parser = argparse.ArgumentParser(description="虚拟力传感器仿真")
    parser.add_argument("--direct", action="store_true", help="无 GUI")
    parser.add_argument("--cube", nargs=3, type=float, default=[0.35, 0.0, 0.04],
                        metavar=("X", "Y", "Z"), help="Cube 初始位置")
    parser.add_argument("--mass", type=float, default=0.15, help="Cube 质量 (kg)")
    parser.add_argument("--friction", type=float, default=2.5,
                        help="Cube / 指面侧向摩擦系数")
    parser.add_argument("--target-force", type=float, default=12.0,
                        help="目标总正压力 (N)")
    parser.add_argument("--no-show", action="store_true", help="不弹窗显示曲线")
    return parser.parse_args()


def _save_outputs(recorder, show=True):
    """汇总保存本轮实验的 CSV 与图像。"""
    if recorder is None:
        return None
    out_dir = os.path.join(os.path.dirname(__file__), "force_data")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(out_dir, f"gripper_force_{stamp}")
    print("\n=== 保存力传感器数据 ===")
    try:
        recorder.save_csv(base + ".csv")
    except Exception as e:
        print(f"[保存] CSV 失败: {e}")
    try:
        recorder.plot(base + ".png", show=show)
    except Exception as e:
        print(f"[保存] PNG 失败: {e}")
    return base


def run_grasp_pipeline(args, recorder_out=None, live_plotter=None, enable_live_plot=False):
    """
    主抓取流程；异常由 main 捕获并落盘。
    recorder_out: 可选 list，创建后立即 append recorder，供异常路径取用。
    live_plotter / enable_live_plot: 与仿真 GUI 同步的实时曲线窗口。
    """
    apply_runtime_config(args)

    mode = p.DIRECT if args.direct else p.GUI
    if mode == p.GUI:
        # 仿真窗口靠左，右侧留给实时曲线
        p.connect(mode, options="--width=920 --height=720")
    else:
        p.connect(mode)
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    if enable_live_plot and live_plotter is None:
        live_plotter = LiveForcePlotter(
            enabled=True, update_every=6, window_xy=(960, 40),
        )

    robot_data = load_robot()
    robot = robot_data["robot"]
    arm_joints = robot_data["arm_joints"]
    ee_link = robot_data["ee_link"]
    init_gripper(robot_data)
    init_ik(robot_data)
    configure_physics(robot, arm_joints)

    finger_mu = float(args.friction)
    for link in (LEFT_LINK, RIGHT_LINK):
        p.changeDynamics(
            robot, link,
            lateralFriction=finger_mu,
            spinningFriction=0.2,
            rollingFriction=0.01,
            jointDamping=JOINT_DAMPING,
            linearDamping=LINK_LINEAR_DAMPING,
            angularDamping=LINK_ANGULAR_DAMPING,
        )

    print("\n========== 虚拟力传感器已部署 ==========")
    print(f"  左指 link={LEFT_LINK}, joint={LEFT_JOINT}")
    print(f"  右指 link={RIGHT_LINK}, joint={RIGHT_JOINT}")
    print(f"  末端 ee_link={ee_link} (gripper_end)")
    print(f"  目标力={TARGET_GRASP_FORCE:.1f}N  Cube质量={args.mass}kg  摩擦={args.friction}")
    print("========================================\n")

    home_orn = p.getQuaternionFromEuler(HOME_EULER)
    cube_pos = np.array(args.cube, dtype=float)
    print(f"Cube 位置: {cube_pos}")
    cube = load_cube(cube_pos.tolist(), mass=args.mass, friction=args.friction)

    print("[初始化] 场景稳定...")
    settle(40)

    cube_pos, _ = p.getBasePositionAndOrientation(cube)
    cube_pos = np.array(cube_pos)
    cube_z0 = float(cube_pos[2])
    print(f"Cube 稳定后: {np.round(cube_pos, 4)}")

    recorder = ForceRecorder(sim_dt=SIM_DT, live_plotter=live_plotter)
    if recorder_out is not None:
        recorder_out.append(recorder)

    print("\n=== 阶段 0: 打开夹爪并校准 ===")
    open_gripper(robot)
    settle(12)
    move_tcp(robot, arm_joints, ee_link,
             [0.30, 0.0, 0.20], GRIPPER_OPEN_WIDTH, home_orn, steps=160)
    off_local = calibrate_finger_offset_local(robot, ee_link)

    print("\n=== 阶段 0.5: 搜索可达抓取姿态（快速）===")
    candidates = select_grasp_orientation(
        robot, arm_joints, ee_link, cube_pos, off_local, top_k=4, cube=cube,
    )
    cube_pos = np.array(p.getBasePositionAndOrientation(cube)[0], dtype=float)
    cube_z0 = float(cube_pos[2])
    cube_orn0 = p.getBasePositionAndOrientation(cube)[1]
    print(f"Cube 搜索后位置: {np.round(cube_pos, 4)}")

    target_orn = None
    ee_grasp = None
    approach_ok = False

    # 接近全程保持 Cube 可见：只关碰撞，不暂存到远处
    set_robot_cube_collision(robot, cube, enable=False)
    p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])

    for ci, grasp in enumerate(candidates):
        print(
            f"\n=== 尝试候选[{ci}] euler={np.round(grasp['euler'], 3)} ==="
        )
        target_orn = grasp["orn"]
        try:
            # 固定 Cube，避免接近时被碰飞；始终可见
            p.resetBasePositionAndOrientation(
                cube, cube_pos.tolist(), cube_orn0,
            )
            p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])
            set_robot_cube_collision(robot, cube, enable=False)

            ee_grasp = ee_target_for_finger_mid(cube_pos, target_orn, off_local)
            pre = ee_grasp.copy()
            pre[2] = cube_pos[2] + PRE_GRASP_CLEARANCE

            if ci == 0:
                print("  预抓取...")
                move_tcp(
                    robot, arm_joints, ee_link, pre,
                    GRIPPER_OPEN_WIDTH, target_orn, steps=200, allow_via=True,
                )
            else:
                # 失败候选：抬高后直接去下一预抓点，避免回 home 停顿
                print("  切换候选（抬高后连续接近）...")
                cur = get_current_tcp(robot, ee_link)
                via = pre.copy()
                via[2] = max(float(pre[2]), float(cur[2]) + 0.04, cube_pos[2] + 0.12)
                move_tcp(
                    robot, arm_joints, ee_link, via,
                    GRIPPER_OPEN_WIDTH, target_orn, steps=120, allow_via=False,
                    max_corr=1,
                )
                move_tcp(
                    robot, arm_joints, ee_link, pre,
                    GRIPPER_OPEN_WIDTH, target_orn, steps=140, allow_via=False,
                    max_corr=1,
                )

            print("  下降对齐...")
            move_tcp(
                robot, arm_joints, ee_link, ee_grasp,
                GRIPPER_OPEN_WIDTH, target_orn, steps=220, allow_via=True, max_corr=2,
            )

            print("  指心对齐...")
            finger_mid, mid_err = align_finger_mid_to_cube(
                robot, arm_joints, ee_link, cube_pos, target_orn,
                gripper_width=GRIPPER_OPEN_WIDTH,
            )
            # 对齐后把 Cube 钉回原位（碰撞关闭期间可能被微扰）
            p.resetBasePositionAndOrientation(
                cube, cube_pos.tolist(), cube_orn0,
            )
            p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])

            left_p, right_p, finger_mid, axis, span = get_finger_geometry(robot)
            between = cube_between_fingers(
                cube_pos, left_p, right_p, finger_mid, axis,
            )
            axis_align = max(abs(float(axis[0])), abs(float(axis[1])))
            axis_sec = min(abs(float(axis[0])), abs(float(axis[1])))
            tcp_now = get_current_tcp(robot, ee_link)
            tcp_err = float(np.linalg.norm(tcp_now - ee_grasp))
            print(f"  左指={np.round(left_p, 4)}")
            print(f"  右指={np.round(right_p, 4)}")
            print(f"  指心={np.round(finger_mid, 4)}  误差={mid_err*1000:.1f}mm")
            print(f"  目标Cube={np.round(cube_pos, 4)}  TCP误差={tcp_err*1000:.1f}mm")
            print(
                f"  开口轴={np.round(axis, 2)}  span={span:.3f}m  "
                f"夹住={between}  主轴={axis_align:.2f} 副轴={axis_sec:.2f}"
            )
            if (between and mid_err <= 0.018 and span >= 0.10
                    and axis_align >= 0.95 and axis_sec <= 0.30
                    and tcp_err <= 0.035):
                # 仅指尖与 Cube 碰撞，Cube 始终在原位可见
                set_finger_only_cube_collision(robot, cube)
                settle(8)
                cube_now = np.array(
                    p.getBasePositionAndOrientation(cube)[0], dtype=float,
                )
                print(f"  就绪 Cube={np.round(cube_now, 4)}  指心={np.round(finger_mid, 4)}")
                approach_ok = True
                recorder.mark(f"候选[{ci}]到位")
                break
            print(f"  [跳过] 候选[{ci}] 到位几何不合格，试下一个")
        except RuntimeError as e:
            print(f"  [跳过] 候选[{ci}] 失败: {e}")
            set_robot_cube_collision(robot, cube, enable=False)
            p.resetBasePositionAndOrientation(
                cube, cube_pos.tolist(), cube_orn0,
            )
            p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])
            continue

    if not approach_ok:
        set_robot_cube_collision(robot, cube, enable=True)
        raise RuntimeError(
            "所有抓取姿态候选在电机跟踪后均无法可靠包住 Cube"
        )

    ee_grasp = get_current_tcp(robot, ee_link)
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    current_width = GRIPPER_OPEN_WIDTH

    # ---- 闭合 + 记力 ----
    print("\n=== 阶段 3: 闭合夹爪（全开到位后软夹）===")
    recorder.mark("开始闭合")
    left_cmd, right_cmd = width_to_joint_positions(current_width, keep_center=True)
    left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
    contact_detected = False
    both_contact = False
    force_ready = False
    ready_count = 0
    balanced_hold = 0
    motor_force = GRIP_MOTOR_SOFT
    balance_steps = 0
    regrip_count = 0
    drift_realign_count = 0
    last = None

    while True:
        _drive_arm(
            robot, arm_joints, hold_arm,
            force=3200,
            position_gain=0.75, velocity_gain=1.4,
        )
        if not force_ready:
            if ready_count == 0:
                step = CLOSE_STEP * (0.25 if (last and last["has_cube_contact"]
                                              and not last["both_fingers_contact"])
                                    else 0.7)
                current_width = max(CLOSE_CENTER, current_width - step)
            left_cmd, right_cmd = set_gripper(
                robot, current_width, force=motor_force, keep_center=True
            )
        else:
            # 阶段 B：自适应增益力均衡；力塌陷则补夹紧重试
            if last is not None and last["cube_total"] < TARGET_GRASP_FORCE * 0.45:
                if regrip_count < MAX_REGRIP:
                    regrip_count += 1
                    recorder.mark(f"力塌陷，补夹紧#{regrip_count}")
                    print(
                        f"  >> 补夹紧: Σ={last['cube_total']:.1f}N → "
                        f"对称收紧重试 ({regrip_count}/{MAX_REGRIP})"
                    )
                    current_width = max(
                        CLOSE_CENTER,
                        min(current_width, last["gripper_width"]) - CLOSE_STEP * 10,
                    )
                    left_cmd, right_cmd = set_gripper(
                        robot, current_width, force=motor_force + 100, keep_center=True
                    )
                    force_ready = False
                    ready_count = 0
                    balanced_hold = 0
                    balance_steps = 0
                    motor_force = min(600, motor_force + 50)
                    for _ in range(6):
                        _drive_arm(
                            robot, arm_joints, hold_arm,
                            position_gain=ARM_POSITION_GAIN_FINE,
                        )
                        p.stepSimulation()
                        time.sleep(SIM_DT)
                        last = recorder.log(robot, cube, phase="closing")
                    continue
                recorder.mark("补夹紧次数用尽，对称保持")
                left_cmd, right_cmd = width_to_joint_positions(
                    max(CLOSE_CENTER, current_width), keep_center=True
                )
                left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
                set_gripper_positions(robot, left_cmd, right_cmd, force=motor_force)
                break

            balance_steps += 1
            gain = adaptive_balance_gain(
                last["force_imbalance"] if last else 0.0,
                base=BALANCE_GAIN_BASE,
            )
            left_cmd, right_cmd = balance_gripper_forces(
                robot, left_cmd, right_cmd,
                left_force=last["cube_left"],
                right_force=last["cube_right"],
                close_step=0.0,
                balance_gain=gain,
                target_each=None,
                force=motor_force,
            )
            left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
            current_width = right_cmd

        for _ in range(2):
            _drive_arm(
                robot, arm_joints, hold_arm,
                position_gain=ARM_POSITION_GAIN_FINE,
            )
            p.stepSimulation()
            time.sleep(SIM_DT)
        last = recorder.log(robot, cube, phase="closing", smooth=True)

        if recorder.sim_step % 24 == 0:
            status = ("双指接触" if last["both_fingers_contact"]
                      else ("单指接触" if last["has_cube_contact"] else "未接触"))
            print(
                f"  t={last['time']:.3f}s | 宽={last['gripper_width']:.4f}m | "
                f"Cube L/R={last['cube_left']:.1f}/{last['cube_right']:.1f}N "
                f"Σ={last['cube_total']:.1f}N |Δ|={last['force_imbalance']:.1f}N | {status}"
            )

        # 闭合过程中物体被挤飞 / 大幅漂移 → 停收、重新对准当前物体位置
        cube_now = np.array([
            last["cube_x"], last["cube_y"], last["cube_z"]
        ], dtype=float)
        _, _, mid_now, _, _ = get_finger_geometry(robot)
        drift = float(np.linalg.norm(cube_now[:2] - mid_now[:2]))
        if (drift > CUBE_DRIFT_ABORT and last["has_cube_contact"]
                and drift_realign_count < 2):
            drift_realign_count += 1
            recorder.mark(f"物体漂移，重新对准#{drift_realign_count}")
            print(f"  >> Cube 相对指心漂移 {drift*1000:.1f} mm，重新对准后继续")
            set_gripper(robot, min(OPEN, current_width + 0.015), force=300, keep_center=True)
            for _ in range(30):
                _drive_arm(robot, arm_joints, hold_arm, position_gain=ARM_POSITION_GAIN_FINE)
                p.stepSimulation()
                time.sleep(SIM_DT)
            cube_pos = cube_now.copy()
            align_finger_mid_to_cube(
                robot, arm_joints, ee_link, cube_pos, target_orn,
            )
            hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
            ee_grasp = get_current_tcp(robot, ee_link)
            force_ready = False
            ready_count = 0
            current_width = max(current_width, CLOSE_CENTER + 0.02)
            continue

        if last["has_cube_contact"] and not contact_detected:
            contact_detected = True
            recorder.mark("首次接触")

        if last["both_fingers_contact"] and not both_contact:
            both_contact = True
            recorder.mark("双指接触")

        # 需双指接触且物体仍对准指心，才计入稳定；力过大立即保持防过冲
        _, _, mid_chk, _, _ = get_finger_geometry(robot)
        cube_xy_err = float(np.linalg.norm(
            np.array([last["cube_x"], last["cube_y"]]) - mid_chk[:2]
        ))
        aligned_ok = cube_xy_err < 0.020
        strong_grip = (
            last["both_fingers_contact"]
            and last["cube_left"] > CONTACT_THRESHOLD
            and last["cube_right"] > CONTACT_THRESHOLD
            and last["cube_total"] >= TARGET_GRASP_FORCE
            and aligned_ok
        )
        if (not force_ready) and strong_grip:
            ready_count += 1
            if last["cube_total"] >= TARGET_GRASP_FORCE * 1.8:
                ready_count = max(ready_count, 12)
        elif not force_ready:
            ready_count = 0

        if (not force_ready) and ready_count >= 12:
            force_ready = True
            left_cmd = float(last["joint_pos_left"])
            right_cmd = float(last["joint_pos_right"])
            left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
            motor_force = GRIP_MOTOR_FIRM
            if last["force_imbalance"] < FORCE_BALANCE_DEADBAND * 1.5:
                recorder.mark("目标力稳定(跳过均衡)")
                print(
                    f"  >> 左右已均衡: L={last['cube_left']:.1f}N "
                    f"R={last['cube_right']:.1f}N |Δ|={last['force_imbalance']:.1f}N "
                    f"对准误差={cube_xy_err*1000:.1f}mm"
                )
                break
            recorder.mark("目标力稳定，开始力均衡")

        # 双指接触后从软夹切换到稳夹
        if (not force_ready) and last["both_fingers_contact"]:
            motor_force = max(motor_force, 280)

        if force_ready:
            if (last["force_imbalance"] < FORCE_BALANCE_DEADBAND
                    and last["cube_total"] >= TARGET_GRASP_FORCE * 0.75):
                balanced_hold += 1
            else:
                balanced_hold = 0

            if balanced_hold >= 12:
                recorder.mark("夹持稳定(力均衡)")
                print(
                    f"  >> 左右均衡: L={last['cube_left']:.1f}N "
                    f"R={last['cube_right']:.1f}N |Δ|={last['force_imbalance']:.1f}N"
                )
                break

            if balance_steps >= 80:
                recorder.mark("力均衡结束")
                print(
                    f"  >> 结束均衡: L={last['cube_left']:.1f}N "
                    f"R={last['cube_right']:.1f}N |Δ|={last['force_imbalance']:.1f}N"
                )
                break

        if last["cube_total"] > FORCE_THRESHOLD:
            recorder.mark("力超限停止")
            left_cmd = float(last["joint_pos_left"])
            right_cmd = float(last["joint_pos_right"])
            left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
            break

        if (not force_ready) and current_width <= CLOSE_CENTER:
            recorder.mark("对称行程用尽")
            left_cmd = float(last["joint_pos_left"])
            right_cmd = float(last["joint_pos_right"])
            left_cmd, right_cmd = clamp_gripper_positions(left_cmd, right_cmd)
            break

    if not contact_detected:
        raise RuntimeError("闭合结束但未检测到与 Cube 的接触")

    hold_left, hold_right = clamp_gripper_positions(left_cmd, right_cmd)
    print(
        f"\n保持夹爪: left={hold_left:.4f}m right={hold_right:.4f}m "
        f"(L/R力={last['cube_left']:.1f}/{last['cube_right']:.1f}N)"
    )

    ok_grasp, ginfo = verify_cube_in_grasp(robot, cube)
    print(
        f"  抬升前校验: mid_err={ginfo['mid_err']*1000:.1f}mm "
        f"夹住={ginfo['between']} cube={np.round(ginfo['cube'], 4)}"
    )
    if not ok_grasp:
        recorder.mark("抬升前几何不合格，补对准")
        set_gripper(robot, min(OPEN, hold_right + 0.02), force=300, keep_center=True)
        settle(40)
        cube_pos = ginfo["cube"].copy()
        align_finger_mid_to_cube(
            robot, arm_joints, ee_link, cube_pos, target_orn,
        )
        # 短程再夹紧
        w = max(CLOSE_CENTER, hold_right)
        hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
        for _ in range(80):
            w = max(CLOSE_CENTER, w - CLOSE_STEP)
            _drive_arm(robot, arm_joints, hold_arm, position_gain=ARM_POSITION_GAIN_FINE)
            set_gripper(robot, w, force=450, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
            fr = read_normal_forces(robot, cube)
            if (fr.get("both_fingers_contact")
                    and fr.get("cube_total", 0) >= TARGET_GRASP_FORCE * 0.8):
                break
        hold_left = float(p.getJointState(robot, LEFT_JOINT)[0])
        hold_right = float(p.getJointState(robot, RIGHT_JOINT)[0])
        hold_left, hold_right = clamp_gripper_positions(hold_left, hold_right)
        ok_grasp, ginfo = verify_cube_in_grasp(robot, cube)
        print(
            f"  补对准后: mid_err={ginfo['mid_err']*1000:.1f}mm 夹住={ginfo['between']}"
        )
        if not ok_grasp:
            raise RuntimeError("抬升前物体已不在夹爪有效包络内")
        ee_grasp = get_current_tcp(robot, ee_link)
        cube_pos = ginfo["cube"].copy()
        cube_z0 = float(cube_pos[2])

    # ---- 提升 ----
    print("\n=== 阶段 4: 提升物体 ===")
    recorder.mark("开始提升")
    lift_target = ee_grasp.copy()
    lift_target[2] = cube_pos[2] + 0.16

    hold_force = 1000
    set_gripper_positions(robot, hold_left, hold_right, force=hold_force)
    settle(60)

    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    goal, lift_ik_err, lift_target = _solve_ik_hidden(
        robot, arm_joints, ee_link, lift_target, start, target_orn,
        hold_left, hold_right,
    )
    if lift_ik_err > IK_OK_ERR * 3.0:
        raise RuntimeError(f"提升 IK 失败: {lift_ik_err * 1000:.1f} mm")
    set_gripper_positions(robot, hold_left, hold_right, force=hold_force)

    joint_travel = float(np.linalg.norm(goal - start))
    lift_steps = int(max(420, joint_travel / 0.0028))

    slip = False
    min_cube_force = float("inf")
    for i in range(1, lift_steps + 1):
        s = _smoothstep5(i / lift_steps)
        q = start + s * (goal - start)
        pg = ARM_POSITION_GAIN_FINE if i > lift_steps * 0.7 else ARM_POSITION_GAIN
        _drive_arm(robot, arm_joints, q, position_gain=pg)

        # 抬升中微型力均衡闭环
        if i % LIFT_BALANCE_EVERY == 0 and last is not None:
            if last["cube_total"] < TARGET_GRASP_FORCE * 0.6:
                # 力偏弱：对称微收紧
                hold_left = max(CLOSE, hold_left - CLOSE_STEP * 2)
                hold_right = max(CLOSE_CENTER, hold_right - CLOSE_STEP * 2)
                hold_left, hold_right = clamp_gripper_positions(hold_left, hold_right)
            elif last["force_imbalance"] > FORCE_BALANCE_DEADBAND:
                gain = adaptive_balance_gain(
                    last["force_imbalance"], base=BALANCE_GAIN_BASE * 0.6
                )
                hold_left, hold_right = balance_gripper_forces(
                    robot, hold_left, hold_right,
                    left_force=last["cube_left"],
                    right_force=last["cube_right"],
                    close_step=0.0,
                    balance_gain=gain,
                    target_each=None,
                    force=hold_force,
                )
                hold_left, hold_right = clamp_gripper_positions(hold_left, hold_right)

        set_gripper_positions(robot, hold_left, hold_right, force=hold_force)
        p.stepSimulation()
        time.sleep(SIM_DT)

        last = recorder.log(robot, cube, phase="lifting", smooth=True)
        min_cube_force = min(min_cube_force, last["cube_total"])

        lift_progress_ok = (last["cube_z"] - cube_z0) > 0.04
        if i > 100 and not slip:
            is_slip, _ = detect_slip(
                cube, cube_z0, last["cube_total"], lift_progress_ok,
            )
            if is_slip:
                slip = True
                recorder.mark("疑似滑落")

    recorder.mark("提升完成")
    cube_now, _ = p.getBasePositionAndOrientation(cube)
    lifted = cube_now[2] - cube_pos[2]
    still = read_normal_forces(robot, cube)["has_cube_contact"]
    print(f"\n[结果] 提升高度={lifted:.4f}m, 过程最小Cube力={min_cube_force:.2f}N")
    if recorder.records:
        closing = [
            r for r in recorder.records
            if r["phase"] == "closing" and r["both_fingers_contact"]
        ]
        if closing:
            imb = np.mean([r["force_imbalance"] for r in closing[-30:]])
            print(f"[结果] 闭合末段平均左右力差 |Δ|={imb:.2f}N")

    success = lifted > 0.05
    if slip and success:
        print("[提示] 过程中曾报滑落迹象，但物体最终已抬离桌面")
    if lifted > 0.05 and not still:
        print("[提示] 末态接触力读数偏弱，但物体已抬离桌面")
    print("[结果] 抓取成功" if success else "[结果] 抓取未完全成功（正压力时序已完整记录）")

    closing = [r for r in recorder.records if r["phase"] == "closing"]
    peak = max((r["contact_total"] for r in closing), default=0.0)
    if peak < CONTACT_THRESHOLD:
        raise RuntimeError("闭合阶段未记录到有效正压力")

    print(f"\n[完成] 正压力峰值={peak:.1f}N，记录点数={len(recorder.records)}")
    if not success:
        print("[提示] 提升未完全成功，但不影响力传感器部署验收")
    print("[完成] 虚拟力传感器仿真结束")
    return recorder, success


def main():
    """任务1主流程：部署虚拟力传感 → 侧夹闭合 → 记录正压力时序。"""
    args = parse_args()

    use_gui = not args.direct
    show_plot = not args.no_show and use_gui

    if args.direct or args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    else:
        # 交互后端，便于与 PyBullet GUI 并排实时刷新
        import matplotlib
        try:
            matplotlib.use("TkAgg")
        except Exception:
            pass
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    recorder_box = []
    live = None
    connected = False
    exit_code = 0

    try:
        connected = True
        recorder, _success = run_grasp_pipeline(
            args,
            recorder_out=recorder_box,
            enable_live_plot=show_plot,
        )
        live = getattr(recorder, "live_plotter", None)
        if live is not None:
            live.finalize(recorder, hold=False)
            # 关闭实时窗后再弹出最终汇总图，避免叠窗
            live.close()
            live = None
            recorder.live_plotter = None
        _save_outputs(recorder, show=show_plot)
    except Exception as e:
        exit_code = 1
        print(f"\n[异常] {type(e).__name__}: {e}")
        traceback.print_exc()
        print("[异常] 尝试保存已有力数据后退出...")
        recorder = recorder_box[0] if recorder_box else None
        live = getattr(recorder, "live_plotter", None) if recorder else None
        if live is not None and recorder is not None:
            live.finalize(recorder, hold=False)
            live.close()
            live = None
        _save_outputs(recorder, show=show_plot)
    finally:
        if live is not None:
            try:
                live.close()
            except Exception:
                pass
        if connected:
            try:
                p.disconnect()
            except Exception:
                pass

    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
