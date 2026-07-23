"""
2.adaptive_force_control_grasp.py
============================================================
自适应力控抓取（Force-controlled Grasping）

场景：
  - 铁块 IronBlock：重、硬、高摩擦（刚体），固定于 [0.38, -0.10, 0.025]
  - 海绵 Sponge：轻、易压缩，固定于 [0.38, +0.10, 0.025]
  - 加载后立即冻结贴桌（水平姿态），等待抓取前位置/形态不变
  - 两物体互不挪动；夹爪去找物体（绝不把物体滑到 y=0）

力控开关（二选一改代码，或用命令行）：
  ENABLE_FORCE_CONTROL = True   # 开启：接触后轻柔停下，安全抓起重物与易碎品
  ENABLE_FORCE_CONTROL = False  # 关闭：夹爪无脑闭合，易碎品被捏扁（变色+压矮）/ 可能报错

功能流程（力控开启时）:
  1) 软闭合至接触
  2) 微探闭合，估计接触刚度 k ≈ ΔF / Δw
  3) 按刚度选择安全抓紧力 F_safe（硬物高、软物低）
  4) PID 力伺服：达到 F_safe 后维持恒力
  5) 抬升验证：铁块不掉；海绵不超限“捏扁”

运行（在 simulation2 目录）:
  python 2.adaptive_force_control_grasp.py
  python 2.adaptive_force_control_grasp.py --no-force-control

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
    OPEN,
    CLOSE_CENTER,
    set_gripper,
    set_gripper_positions,
    width_to_joint_positions,
    open_gripper,
    get_gripper_contact_forces,
)
from utils.ik import init_ik, solve_ik, get_current_tcp, LOWER_LIMITS, UPPER_LIMITS

# =====================================================================
# 【力控开关】改这里即可：True=开启力控，False=关闭（无脑闭合）
# =====================================================================
ENABLE_FORCE_CONTROL = True

# -------------------- 仿真 / 控制参数 --------------------
SIM_DT = 1.0 / 240.0
HOME_EULER = [0.0, 0.0, 0.0]
# 抓取轻微低头，避免 +Y 低位贴地折叠 IK
GRASP_PITCH = 0.08
NATURAL_ARM_SEED = np.array([0.0, 0.55, 0.95, -0.70, 0.25, 0.0], dtype=float)
EE_Z_MIN = 0.018
PRE_CLEAR = 0.14

# 接触与安全力（自适应上下界）
F_CONTACT = 1.5          # 判定接触
F_SOFT_SAFE = 10.0        # 软物体安全抓紧力
F_HARD_SAFE = 36.0        # 硬/重物体安全抓紧力
F_ABS_MAX = 70.0          # 绝对上限（防过载）
K_HARD = 2500.0           # N/m，刚度阈值（软接触刚度更低）
CRUSH_WIDTH = 0.014       # 软物允许的接触后额外压缩量 (m)；力控开启时保护阈值
BLIND_CRUSH_WIDTH = 0.006  # 无力控时开始“捏扁”动画的压缩量
BLIND_CRUSH_SHOW = 0.020   # 完全捏扁展示目标压缩量（更大 → 过程更明显）
PROBE_DW = 0.006           # 探刚度时额外闭合量

# PID（宽度增量控制：正输出 → 继续闭合）
PID_KP = 9.0e-5
PID_KI = 1.8e-5
PID_KD = 3.0e-5
PID_OUT_MAX = 2.8e-4      # 每步最大闭合增量

GRIP_MOTOR = 160
GRIP_MOTOR_APPROACH = 60
GRIP_MOTOR_HOLD = 420
GRIP_MOTOR_BLIND = 900    # 无脑闭合大力矩
ARM_FORCE = 2200
HOLD_ARM_FORCE = 3200
HOLD_SETTLE = 28          # 插值到位后短稳定，避免停顿感
JOINT_STEP = 0.0022       # 关节插值步长（越小越平滑）


# =====================================================================
# 物体
# =====================================================================

# cube_small.urdf 边长 5cm；贴地放置时中心高度 = 半边长
CUBE_SIZE = 0.05
CUBE_HALF = 0.5 * CUBE_SIZE
TABLE_ORN = [0.0, 0.0, 0.0, 1.0]  # 水平摆放，禁止倾斜


def table_pose(xy, z=None):
    """桌面摆放位姿：XY 给定，Z 贴地，姿态固定水平。"""
    xy = np.asarray(xy, dtype=float).reshape(-1)
    pos = [float(xy[0]), float(xy[1]), float(CUBE_HALF if z is None else z)]
    return pos, list(TABLE_ORN)


def pin_body(body, pos, orn=None):
    """
    冻结物体：mass=0 + 清零速度。
    加载后、等待抓取前用此保持“位置和形态不变”。
    """
    if orn is None:
        orn = TABLE_ORN
    p.resetBasePositionAndOrientation(body, list(pos), list(orn))
    p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
    p.changeDynamics(body, -1, mass=0.0)
    return np.array(pos, dtype=float), tuple(orn)


def load_iron_block(position, mass=0.55, friction=3.2, pin=True):
    """重且硬的刚体（铁块）。默认加载后冻结，避免落地弹跳。"""
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    pos, orn = table_pose(position[:2] if len(position) >= 2 else position,
                          position[2] if len(position) > 2 else None)
    # 若调用方给了完整 3D 且 z 合理，仍强制贴地半高，避免悬空下落
    pos[2] = CUBE_HALF
    body = p.loadURDF(
        "cube_small.urdf",
        basePosition=pos,
        baseOrientation=orn,
        useFixedBase=False,
    )
    p.changeDynamics(
        body, -1,
        mass=float(mass),
        lateralFriction=float(friction),
        spinningFriction=0.2,
        rollingFriction=0.02,
        restitution=0.0,
        linearDamping=0.2,
        angularDamping=0.2,
    )
    p.changeVisualShape(body, -1, rgbaColor=[0.35, 0.35, 0.38, 1.0])
    if pin:
        pin_body(body, pos, orn)
    print(f"  [IronBlock] pos={np.round(pos, 3)} mass={mass}kg μ={friction} pinned={pin}")
    return body


def load_sponge(position, mass=0.04, friction=2.2, pin=True):
    """
    轻且易变形物体（海绵）。
    用较低 contactStiffness 模拟可压缩接触；加载后默认冻结防弹跳。
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    pos, orn = table_pose(position[:2], position[2] if len(position) > 2 else None)
    pos[2] = CUBE_HALF
    body = p.loadURDF(
        "cube_small.urdf",
        basePosition=pos,
        baseOrientation=orn,
        useFixedBase=False,
    )
    p.changeDynamics(
        body, -1,
        mass=float(mass),
        lateralFriction=float(friction),
        spinningFriction=0.05,
        rollingFriction=0.005,
        restitution=0.0,
        linearDamping=0.25,
        angularDamping=0.25,
        contactStiffness=450.0,
        contactDamping=20.0,
    )
    p.changeVisualShape(body, -1, rgbaColor=[0.95, 0.75, 0.35, 1.0])
    if pin:
        pin_body(body, pos, orn)
    print(f"  [Sponge] pos={np.round(pos, 3)} mass={mass}kg μ={friction} pinned={pin}")
    return body


def restore_iron_dynamics(body, mass=0.55, friction=3.2):
    """恢复铁块刚体动力学参数（质量、摩擦、阻尼）。"""
    p.changeDynamics(
        body, -1,
        mass=float(mass),
        lateralFriction=float(friction),
        spinningFriction=0.2,
        rollingFriction=0.02,
        restitution=0.0,
        linearDamping=0.05,
        angularDamping=0.05,
    )


def restore_sponge_dynamics(body, mass=0.04, friction=2.2):
    """恢复海绵接触动力学（较低接触刚度，模拟可压缩）。"""
    p.changeDynamics(
        body, -1,
        mass=float(mass),
        lateralFriction=float(friction),
        spinningFriction=0.05,
        rollingFriction=0.005,
        restitution=0.0,
        linearDamping=0.1,
        angularDamping=0.1,
        contactStiffness=450.0,
        contactDamping=20.0,
    )


# =====================================================================
# 基础工具
# =====================================================================

def settle(n=40):
    """推进仿真 n 步等待稳定。"""
    for _ in range(n):
        p.stepSimulation()
        time.sleep(SIM_DT)


def _smoothstep5(t: float) -> float:
    """5 次多项式平滑插值系数 s(t)，端点速度/加速度为 0。"""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def clamp_arm(angles):
    """将臂关节角钳位到 URDF 限位。"""
    a = np.asarray(angles, dtype=float)
    return np.clip(a, LOWER_LIMITS, UPPER_LIMITS)


def drive_arm(robot, arm_joints, angles, force=ARM_FORCE, pg=0.4, vg=1.0):
    """臂关节位置控制下发。"""
    for j, ang in zip(arm_joints, clamp_arm(angles)):
        p.setJointMotorControl2(
            robot, j, p.POSITION_CONTROL,
            targetPosition=float(ang), force=force,
            positionGain=pg, velocityGain=vg,
        )


def read_force(robot, body=None):
    """读取夹爪正压力；若给 body 则优先统计与该物体的接触力。"""
    if body is None:
        c = get_gripper_contact_forces(robot)
        return {
            "left": c["left"], "right": c["right"], "total": c["total"],
            "avg": 0.5 * (c["left"] + c["right"]),
        }
    left = p.getContactPoints(bodyA=robot, linkIndexA=LEFT_LINK, bodyB=body)
    right = p.getContactPoints(bodyA=robot, linkIndexA=RIGHT_LINK, bodyB=body)
    lf = sum(pt[9] for pt in left)
    rf = sum(pt[9] for pt in right)
    return {"left": lf, "right": rf, "total": lf + rf, "avg": 0.5 * (lf + rf)}


def read_force_smooth(robot, body=None, n=4):
    """对正压力做 n 步平均，降低接触力噪声。"""
    acc = {"left": 0.0, "right": 0.0, "total": 0.0, "avg": 0.0}
    for _ in range(n):
        f = read_force(robot, body)
        for k in acc:
            acc[k] += f[k]
        p.stepSimulation()
        time.sleep(SIM_DT)
    return {k: v / n for k, v in acc.items()}


def get_finger_mid(robot):
    """返回指心及左右指世界坐标。"""
    left = np.array(p.getLinkState(robot, LEFT_LINK)[4], dtype=float)
    right = np.array(p.getLinkState(robot, RIGHT_LINK)[4], dtype=float)
    return (left + right) * 0.5, left, right


def calibrate_off_local(robot, ee_link):
    """标定指心相对 EE 的局部偏移。"""
    # 先对称张开，避免左右指不对称污染局部偏移
    set_gripper(robot, OPEN, force=400, keep_center=True)
    for _ in range(20):
        p.stepSimulation()
        time.sleep(SIM_DT)
    ee = np.array(p.getLinkState(robot, ee_link)[4], dtype=float)
    orn = p.getLinkState(robot, ee_link)[5]
    mid, left, right = get_finger_mid(robot)
    inv = p.invertTransform([0, 0, 0], orn)[1]
    off = np.array(p.rotateVector(inv, (mid - ee).tolist()), dtype=float)
    # 开口沿 Y 时左右应对称；抑制标定噪声在横向的偏置
    if abs(off[1]) < 0.025:
        off[1] = 0.0
    print(f"  指心相对 EE 局部偏移: {np.round(off, 4)}")
    return off


def ee_for_mid(mid, orn, off_local):
    """由指心目标换算 EE 目标位姿。"""
    off_w = np.array(p.rotateVector(orn, np.asarray(off_local).tolist()))
    ee = np.asarray(mid, dtype=float) - off_w
    ee[2] = max(float(ee[2]), EE_Z_MIN)
    return ee


def grasp_orn_for_pos(pos):
    """
    固定位侧夹：开口沿世界 Y（yaw≈0），加一点 pitch 让腕部自然朝桌面。
    纯 identity 在 +Y 低位常解出贴地折叠姿态。
    """
    _ = pos
    return p.getQuaternionFromEuler([0.0, float(GRASP_PITCH), 0.0])


def set_robot_obj_collision(robot, obj, enable=True):
    """开关整臂与物体的碰撞过滤。"""
    flag = 1 if enable else 0
    for link in range(-1, p.getNumJoints(robot)):
        p.setCollisionFilterPair(robot, obj, link, -1, flag)


def set_finger_only_collision(robot, obj):
    """仅保留左右指与物体碰撞，接近阶段减少误碰。"""
    for link in range(-1, p.getNumJoints(robot)):
        p.setCollisionFilterPair(robot, obj, link, -1, 0)
    p.setCollisionFilterPair(robot, obj, LEFT_LINK, -1, 1)
    p.setCollisionFilterPair(robot, obj, RIGHT_LINK, -1, 1)


def interpolate_joints(robot, arm_joints, start, goal, width, steps):
    """5 次多项式关节插值，全程电机跟踪，无 reset 跳变。"""
    lp, rp = width_to_joint_positions(width, keep_center=True)
    start = np.asarray(start, dtype=float)
    goal = clamp_arm(goal)
    travel = float(np.linalg.norm(goal - start))
    steps = int(max(steps, travel / JOINT_STEP, 100))
    for i in range(1, steps + 1):
        s = _smoothstep5(i / steps)
        q = start + s * (goal - start)
        pg = 0.70 if i > steps * 0.65 else 0.45
        drive_arm(robot, arm_joints, q, force=ARM_FORCE, pg=pg, vg=1.2)
        set_gripper_positions(robot, lp, rp, force=400)
        p.stepSimulation()
        time.sleep(SIM_DT)
    for _ in range(HOLD_SETTLE):
        drive_arm(robot, arm_joints, goal, force=HOLD_ARM_FORCE, pg=0.85, vg=1.4)
        set_gripper_positions(robot, lp, rp, force=400)
        p.stepSimulation()
        time.sleep(SIM_DT)


def _physics_connected():
    """当前进程是否仍连接 PyBullet。"""
    try:
        return bool(p.isConnected())
    except Exception:
        return False


def _is_gui_connection():
    """是否为 GUI 连接模式。"""
    try:
        return int(p.getConnectionInfo().get("connectionMethod", -1)) == int(p.GUI)
    except Exception:
        return False


_GUI_WIN_RECT = None


def _find_pybullet_hwnd():
    """Windows 下查找 PyBullet 仿真窗口句柄（用于恢复窗口位置）。"""
    if sys.platform != "win32":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        user32 = ctypes.windll.user32
        pid = os.getpid()
        found = []

        @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
        def _enum(hwnd, _lp):
            if not user32.IsWindowVisible(hwnd):
                return True
            wpid = wintypes.DWORD()
            user32.GetWindowThreadProcessId(hwnd, ctypes.byref(wpid))
            if int(wpid.value) != pid:
                return True
            length = user32.GetWindowTextLengthW(hwnd)
            buf = ctypes.create_unicode_buffer(length + 1)
            user32.GetWindowTextW(hwnd, buf, length + 1)
            title = (buf.value or "").lower()
            if any(x in title for x in ("powershell", "cmd.exe", "cursor", "visual studio")):
                return True
            rect = wintypes.RECT()
            user32.GetWindowRect(hwnd, ctypes.byref(rect))
            w = rect.right - rect.left
            h = rect.bottom - rect.top
            if w >= 400 and h >= 300:
                found.append((hwnd, w * h))
            return True

        user32.EnumWindows(_enum, 0)
        if not found:
            return None
        found.sort(key=lambda x: x[1], reverse=True)
        return found[0][0]
    except Exception:
        return None


def _save_gui_window_rect():
    """关闭渲染前保存 GUI 窗口矩形。"""
    global _GUI_WIN_RECT
    hwnd = _find_pybullet_hwnd()
    if hwnd is None:
        return
    try:
        import ctypes
        from ctypes import wintypes
        rect = wintypes.RECT()
        ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))
        _GUI_WIN_RECT = (rect.left, rect.top, rect.right, rect.bottom)
    except Exception:
        _GUI_WIN_RECT = None


def _restore_gui_window_rect():
    """重新开渲染后恢复 GUI 窗口位置，避免跳到左上角。"""
    global _GUI_WIN_RECT
    if _GUI_WIN_RECT is None:
        return
    hwnd = _find_pybullet_hwnd()
    if hwnd is None:
        return
    try:
        import ctypes
        left, top, right, bottom = _GUI_WIN_RECT
        ctypes.windll.user32.SetWindowPos(
            hwnd, 0, left, top, right - left, bottom - top, 0x0004 | 0x0010,
        )
    except Exception:
        pass


def _set_rendering(enable: bool):
    """关渲染避免 IK reset 瞬闪；再开后恢复窗口位置防跳左上角。"""
    if not _physics_connected():
        return
    try:
        if enable:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            if _is_gui_connection():
                _restore_gui_window_rect()
        else:
            if _is_gui_connection():
                _save_gui_window_rect()
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    except Exception:
        pass


def _arm_min_link_z(robot, arm_joints):
    """臂各连杆最低世界 Z，用于惩罚贴地折叠 IK 解。"""
    zs = []
    for j in arm_joints:
        child = int(p.getJointInfo(robot, j)[16])
        if child >= 0:
            zs.append(float(p.getLinkState(robot, child)[4][2]))
    return float(min(zs)) if zs else 0.0


def _score_ik_goal(robot, arm_joints, ee_link, goal, tcp):
    """评估 IK 候选：TCP 误差 + 折叠惩罚 + 最低高度惩罚。"""
    goal = clamp_arm(goal)
    for j, a in zip(arm_joints, goal):
        p.resetJointState(robot, j, float(a))
    err = float(np.linalg.norm(get_current_tcp(robot, ee_link) - tcp))
    fold = float(np.sum(np.abs(goal)))
    min_z = _arm_min_link_z(robot, arm_joints)
    cost = err + 0.02 * max(0.0, fold - 6.0) + 0.8 * max(0.0, 0.055 - min_z)
    return err, cost, min_z, fold


def _solve_ik_no_teleport(robot, arm_joints, ee_link, tcp, start, orn):
    """
    求 IK 时关闭渲染并恢复关节，避免可见瞬移。
    GUI 下少种子；多种子择优，拒绝贴地折叠解。
    """
    if not _physics_connected():
        return np.asarray(start, dtype=float).copy(), 1.0

    start = np.asarray(start, dtype=float)
    left0 = float(p.getJointState(robot, LEFT_JOINT)[0])
    right0 = float(p.getJointState(robot, RIGHT_JOINT)[0])
    tcp = np.asarray(tcp, dtype=float)

    seeds = [start, NATURAL_ARM_SEED.copy()]
    if not _is_gui_connection():
        seeds.extend([
            clamp_arm(0.5 * start + 0.5 * NATURAL_ARM_SEED),
            np.array([0.0, 0.35, 0.70, -0.45, 0.15, 0.0], dtype=float),
        ])
    if abs(float(tcp[1])) > 0.04:
        side = NATURAL_ARM_SEED.copy()
        side[0] = 0.25 * np.sign(float(tcp[1]))
        seeds.append(side)

    best_goal = start.copy()
    best_err = 1.0
    best_cost = 1e9

    _set_rendering(False)
    try:
        for seed in seeds:
            try:
                raw = clamp_arm(solve_ik(
                    robot, arm_joints, ee_link, tcp, seed,
                    target_orn=orn, quiet=True,
                ))
            except Exception:
                continue
            err, cost, _, _ = _score_ik_goal(robot, arm_joints, ee_link, raw, tcp)
            if cost < best_cost:
                best_cost, best_err, best_goal = cost, err, raw
        if best_err > 0.06 or best_cost > 0.12:
            best_err = max(best_err, 0.08)
    except p.error:
        return best_goal, 1.0
    finally:
        if _physics_connected():
            try:
                for j, a in zip(arm_joints, start):
                    p.resetJointState(robot, j, float(a))
                p.resetJointState(robot, LEFT_JOINT, left0)
                p.resetJointState(robot, RIGHT_JOINT, right0)
            except p.error:
                pass
            _set_rendering(True)
    return best_goal, best_err


def move_tcp(robot, arm_joints, ee_link, tcp, orn, width=OPEN, steps=200,
             allow_via=True):
    """平滑移动 TCP：可选上方途经点 + quintic 关节插值（无可见瞬移）。"""
    tcp = np.asarray(tcp, dtype=float).copy()
    tcp[2] = max(float(tcp[2]), EE_Z_MIN)

    cur = get_current_tcp(robot, ee_link)
    if allow_via and (cur[2] - tcp[2]) > 0.06:
        via = tcp.copy()
        via[2] = max(tcp[2] + 0.10, cur[2] - 0.02)
        if via[2] > tcp[2] + 0.04:
            move_tcp(
                robot, arm_joints, ee_link, via, orn, width,
                steps=max(120, steps // 2), allow_via=False,
            )

    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    goal, err = _solve_ik_no_teleport(
        robot, arm_joints, ee_link, tcp, start, orn,
    )
    if err > 0.045:
        raise RuntimeError(f"IK 误差过大 {err*1000:.1f} mm @ {np.round(tcp, 3)}")
    interpolate_joints(robot, arm_joints, start, goal, width, steps)
    return get_current_tcp(robot, ee_link)


# =====================================================================
# 力控：PID + 自适应目标力
# =====================================================================

class ForcePID:
    """基于力误差的夹爪宽度增量 PID。输出>0 表示继续闭合。"""

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD,
                 out_max=PID_OUT_MAX, i_max=2e-3):
        """初始化力误差→宽度增量 PID 增益与限幅。"""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max = out_max
        self.i_max = i_max
        self.integral = 0.0
        self.prev_err = 0.0
        self.first = True

    def reset(self):
        """清零积分与微分状态。"""
        self.integral = 0.0
        self.prev_err = 0.0
        self.first = True

    def step(self, f_target, f_meas, dt):
        """一步 PID：输入目标力/实测力，输出闭合宽度增量。"""
        err = float(f_target - f_meas)  # 力不足 → 正 → 闭合
        self.integral = float(np.clip(
            self.integral + err * dt, -self.i_max, self.i_max,
        ))
        deriv = 0.0 if self.first else (err - self.prev_err) / max(dt, 1e-6)
        self.first = False
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        return float(np.clip(out, -self.out_max, self.out_max))


def estimate_stiffness(forces, widths):
    """用探接触段的 ΔF/Δw 估计等效刚度 (N/m)。"""
    if len(forces) < 2:
        return 0.0
    f = np.asarray(forces, dtype=float)
    w = np.asarray(widths, dtype=float)
    dw = float(w[0] - w[-1])
    df = float(f[-1] - f[0])
    f_peak = float(np.max(f))
    f_mean = float(np.mean(f))
    # 硬接触：很小压缩力就很高
    if dw < 3e-4 and f_peak >= 15.0:
        return 5.0e4
    if dw < 1e-5:
        return 1.0e6 if f_peak > 15.0 else 0.0
    k = df / max(dw, 1e-6)
    # 平均力不高且压缩明显 → 偏软，抑制误判
    if dw >= 0.004 and f_mean < 12.0:
        k = min(k, K_HARD * 0.5)
    if f_peak >= 35.0 and dw < 0.004:
        k = max(k, K_HARD * 2.0)
    return float(max(0.0, k))


def choose_safe_force(stiffness, label="", f_peak=0.0, compression=0.0):
    """
    自适应安全抓紧力：
      硬物 → 较高力保证抬升；软物 → 较低力防止捏扁。
    """
    # 压缩较大但峰值力仍有限 → 软
    soft_hint = compression >= 0.004 and f_peak < 22.0
    hard = (stiffness >= K_HARD or f_peak >= 40.0) and not soft_hint
    if hard:
        f = F_HARD_SAFE
        kind = "刚体/硬物"
    else:
        f = F_SOFT_SAFE
        kind = "软/易碎物"
    f = float(np.clip(f, F_SOFT_SAFE * 0.5, F_ABS_MAX))
    print(
        f"  [自适应] {label} k={stiffness:.0f} N/m  "
        f"F_peak={f_peak:.1f}N  dW={compression*1000:.1f}mm → {kind}  F_safe={f:.1f}N"
    )
    return f, kind


class GraspRecorder:
    """力控抓取过程数据记录与导出。"""

    def __init__(self):
        """初始化抓取时序缓冲。"""
        self.rows = []
        self.t0 = None

    def log(self, **kw):
        """追加一行力/宽度/刚度等采样。"""
        now = time.perf_counter()
        if self.t0 is None:
            self.t0 = now
        row = {"time": now - self.t0}
        row.update(kw)
        self.rows.append(row)
        return row

    def save(self, path_prefix):
        """保存 CSV 与力控曲线图到 force_data/。"""
        if not self.rows:
            return None, None
        os.makedirs("force_data", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join("force_data", f"{path_prefix}_{ts}.csv")
        keys = list(self.rows[0].keys())
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        print(f"[保存] {csv_path}")

        import matplotlib.pyplot as plt
        times = [r["time"] for r in self.rows]
        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        axes[0].plot(times, [r.get("f_meas", 0) for r in self.rows],
                     label="实测力", color="C2", lw=1.8)
        axes[0].plot(times, [r.get("f_target", 0) for r in self.rows],
                     label="目标力", color="k", ls="--", lw=1.5)
        axes[0].set_ylabel("力 (N)")
        axes[0].set_title(path_prefix)
        axes[0].legend(loc="upper left")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(times, [r.get("width", 0) for r in self.rows],
                     color="purple", lw=1.5)
        axes[1].set_ylabel("夹爪宽度 (m)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(times, [r.get("obj_z", 0) for r in self.rows],
                     color="C1", lw=1.5, label="物体 Z")
        axes[2].set_xlabel("时间 (s)")
        axes[2].set_ylabel("高度 (m)")
        axes[2].legend(loc="upper left")
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        png_path = csv_path.replace(".csv", ".png")
        fig.savefig(png_path, dpi=140)
        print(f"[绘图] {png_path}")
        plt.close(fig)
        return csv_path, png_path


def force_controlled_grasp(robot, arm_joints, body, label, sim_dt=SIM_DT):
    """
    闭环力控主循环：接触 → 探刚度 → 选 F_safe → PID 恒力 → 返回最终宽度。
    """
    pid = ForcePID()
    rec = GraspRecorder()
    width = OPEN
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])

    phase = "approach"
    width_at_contact = None
    probe_forces, probe_widths = [], []
    f_target = F_SOFT_SAFE
    kind = "未知"
    hold_count = 0
    crushed = False
    f_peak = 0.0
    motor = GRIP_MOTOR_APPROACH

    print(f"\n=== 力控闭合 [{label}] ===")
    set_finger_only_collision(robot, body)

    for step in range(1000):
        drive_arm(
            robot, arm_joints, hold_arm,
            force=HOLD_ARM_FORCE, pg=0.7, vg=1.3,
        )
        f = read_force_smooth(robot, body, n=3)
        f_meas = f["total"]
        f_peak = max(f_peak, f_meas)
        obj_z = p.getBasePositionAndOrientation(body)[0][2]

        if phase == "approach":
            if f_meas >= F_CONTACT:
                width_at_contact = width
                phase = "probe"
                probe_forces = [f_meas]
                probe_widths = [width]
                motor = GRIP_MOTOR
                print(f"  [接触] F={f_meas:.2f}N  width={width:.4f}m → 探刚度")
            else:
                width = max(CLOSE_CENTER, width - 0.00016)

        elif phase == "probe":
            width = max(CLOSE_CENTER, width - 0.00008)
            probe_forces.append(f_meas)
            probe_widths.append(width)
            compression = width_at_contact - width
            # 硬物：极小压缩力已很高；软物需更大压缩
            hard_hint = (f_meas >= 25.0 and compression < 0.003) or f_peak >= 45.0
            done_probe = (
                compression >= PROBE_DW
                or hard_hint
                or len(probe_forces) >= 55
            )
            if done_probe:
                k = estimate_stiffness(probe_forces, probe_widths)
                if hard_hint:
                    k = max(k, K_HARD * 1.5)
                f_target, kind = choose_safe_force(
                    k, label, f_peak=f_peak, compression=compression,
                )
                pid.reset()
                phase = "servo"
                if "软" in kind:
                    motor = GRIP_MOTOR
                else:
                    motor = GRIP_MOTOR_HOLD
                print(f"  [伺服] 进入 PID 力控 → F_target={f_target:.1f}N")

        elif phase == "servo":
            # 伺服中若力陡升且压缩很小，纠正为硬物
            if "软" in kind and f_meas >= 30.0 and (width_at_contact - width) < 0.004:
                f_target, kind = choose_safe_force(
                    K_HARD * 2, label, f_peak=f_meas, compression=width_at_contact - width,
                )
                pid.reset()
                motor = GRIP_MOTOR_HOLD
                print("  [纠正] 接触表现偏硬，提高 F_safe")

            if width_at_contact is not None:
                compression = width_at_contact - width
                if kind.startswith("软") and compression > CRUSH_WIDTH:
                    crushed = True
                    phase = "hold"
                    hold_count = 0
                    motor = GRIP_MOTOR_HOLD
                    print(
                        f"  [保护] 压缩 {compression*1000:.1f}mm 超限，"
                        f"停止加力（防捏扁） F={f_meas:.2f}N"
                    )
                else:
                    dw = pid.step(f_target, f_meas, sim_dt * 3)
                    if f_meas > f_target + 8.0:
                        dw = -abs(dw)  # 过冲则松开
                    elif f_meas > f_target + 3.0:
                        dw *= 0.25
                    if ("刚" in kind) and f_meas < 0.92 * f_target:
                        dw = max(float(dw), 1.0e-4)
                    width = float(np.clip(width - dw, CLOSE_CENTER, OPEN))
                    near = abs(f_meas - f_target) < 5.0 and f_meas > 0.6 * f_target
                    hard_enough = (
                        ("刚" in kind)
                        and (f_meas >= max(28.0, 0.75 * f_target))
                    )
                    if near or hard_enough:
                        hold_count += 1
                        if hold_count >= 18:
                            phase = "hold"
                            hold_count = 0
                            motor = GRIP_MOTOR_HOLD
                            print(f"  [恒力] 稳定于 F={f_meas:.2f}N ≈ {f_target:.1f}N")
                    else:
                        hold_count = 0
                    # 超时：若已有足够力则强制进入保持
                    if step > 700 and f_meas >= 0.7 * f_target:
                        phase = "hold"
                        hold_count = 0
                        motor = GRIP_MOTOR_HOLD
                        print(f"  [恒力] 超时转入保持 F={f_meas:.2f}N")

        elif phase == "hold":
            if f_meas < f_target - 4.0:
                width = max(CLOSE_CENTER, width - 6e-5)
            elif f_meas > f_target + 8.0:
                width = min(OPEN, width + 1e-4)
            hold_count += 1
            if hold_count >= 30:
                break

        set_gripper(robot, width, force=motor, keep_center=True)
        rec.log(
            phase=phase, f_meas=f_meas, f_target=f_target,
            width=width, obj_z=obj_z,
            left=f["left"], right=f["right"],
        )

        if step % 30 == 0:
            print(
                f"  t~{step*sim_dt:.2f}s [{phase}] "
                f"F={f_meas:.1f}/{f_target:.1f}N  w={width:.4f}m"
            )

    info = {
        "f_target": f_target,
        "kind": kind,
        "crushed": crushed,
        "width": width,
        "final_force": read_force_smooth(robot, body, n=5)["total"],
        "recorder": rec,
        "mode": "force_control",
    }
    return width, info


def update_sponge_crush_visual(body, fixed_pos, fixed_orn, compression, max_c=0.018,
                               last_state=None):
    """
    海绵捏扁可视化（XY 钉死在固定位，不滑到 y=0）：
      - 颜色：黄 → 橙 → 红
      - 外形：Z 压矮 + 微倾
    仅在外观有可见变化时才 reset，避免每步 reset 造成闪烁。
    last_state: 可选 dict，缓存上一次已应用的 ratio，用于限频。
    返回 (ratio, stage)。
    """
    ratio = float(np.clip(compression / max(max_c, 1e-6), 0.0, 1.0))
    # 量化，避免浮点微抖动触发无意义 reset
    ratio_q = round(ratio * 40.0) / 40.0
    rgba = [
        min(1.0, 0.95 + 0.05 * ratio_q),
        max(0.08, 0.75 * (1.0 - 0.95 * ratio_q)),
        max(0.04, 0.35 * (1.0 - 1.1 * ratio_q)),
        1.0,
    ]
    z0 = float(fixed_pos[2])
    z = max(0.010, z0 * (1.0 - 0.60 * ratio_q))
    base_eul = list(p.getEulerFromQuaternion(fixed_orn))
    crush_orn = p.getQuaternionFromEuler([
        base_eul[0] + 0.18 * ratio_q,
        base_eul[1],
        base_eul[2],
    ])
    pos = [float(fixed_pos[0]), float(fixed_pos[1]), z]

    need_update = True
    if last_state is not None:
        prev = last_state.get("ratio_q")
        if prev is not None and abs(prev - ratio_q) < 1e-9:
            need_update = False

    if need_update:
        try:
            p.changeVisualShape(body, -1, rgbaColor=rgba)
        except Exception:
            pass
        p.resetBasePositionAndOrientation(body, pos, crush_orn)
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
        if last_state is not None:
            last_state["ratio_q"] = ratio_q

    if ratio < 0.20:
        stage = 0
    elif ratio < 0.45:
        stage = 1
    elif ratio < 0.75:
        stage = 2
    else:
        stage = 3
    return ratio, stage


def blind_close_grasp(robot, arm_joints, body, label, soft_object=False,
                      fixed_pos=None, fixed_orn=None, sim_dt=SIM_DT):
    """
    无力控：夹爪无脑大力闭合。
    对软物放慢闭合并做“捏扁”可视化（变色 + 压矮）。
    """
    rec = GraspRecorder()
    width = OPEN
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    width_at_contact = None
    crushed = False
    f_peak = 0.0
    f_target = 0.0
    crush_stage = -1
    crush_vis = {}
    if fixed_pos is None:
        fixed_pos = np.array(p.getBasePositionAndOrientation(body)[0], dtype=float)
    else:
        fixed_pos = np.asarray(fixed_pos, dtype=float)
    if fixed_orn is None:
        fixed_orn = p.getBasePositionAndOrientation(body)[1]

    print(f"\n=== 无脑闭合（力控关闭）[{label}] ===")
    set_finger_only_collision(robot, body)

    for step in range(900):
        drive_arm(
            robot, arm_joints, hold_arm,
            force=HOLD_ARM_FORCE, pg=0.7, vg=1.3,
        )
        close_step = 0.00006 if soft_object else 0.00026
        f = read_force_smooth(robot, body, n=2)
        f_meas = f["total"]
        f_peak = max(f_peak, f_meas)

        if width_at_contact is None and f_meas >= F_CONTACT:
            width_at_contact = width
            print(f"  [接触] F={f_meas:.2f}N，仍继续无脑加压闭合...")
        # 软物：力读数偏弱时，用“指宽已明显收紧”作为接触起点，确保捏扁过程可见
        if soft_object and width_at_contact is None and width < OPEN - 0.018:
            width_at_contact = width
            print(f"  [接触近似] 指宽已收紧，开始展示捏扁过程 (w={width:.4f})")

        width = max(CLOSE_CENTER, width - close_step)
        set_gripper(robot, width, force=GRIP_MOTOR_BLIND, keep_center=True)

        compression = 0.0
        if width_at_contact is not None:
            compression = width_at_contact - width
            if soft_object:
                ratio, stage = update_sponge_crush_visual(
                    body, fixed_pos, fixed_orn, compression,
                    max_c=BLIND_CRUSH_SHOW, last_state=crush_vis,
                )
                if stage > crush_stage:
                    crush_stage = stage
                    mm = compression * 1000.0
                    msgs = [
                        f"  [捏扁过程] 开始受压  dW={mm:.1f}mm  ({ratio*100:.0f}%)",
                        f"  [捏扁过程] 明显变形  dW={mm:.1f}mm  ({ratio*100:.0f}%)  颜色变橙",
                        f"  [捏扁过程] 严重压扁  dW={mm:.1f}mm  ({ratio*100:.0f}%)  颜色变红",
                        f"  [捏扁过程] 已捏扁!   dW={mm:.1f}mm  ({ratio*100:.0f}%)",
                    ]
                    print(msgs[stage])
                if compression >= BLIND_CRUSH_WIDTH:
                    crushed = True
            elif f_meas >= F_ABS_MAX * 1.2:
                print(f"  [告警] 无力控力尖峰 F={f_meas:.1f}N（过载风险）")

        # 不再每步 reset 硬物位姿（那是 GUI 闪烁主因）

        obj_z = p.getBasePositionAndOrientation(body)[0][2]
        rec.log(
            phase="blind_crush" if crushed else "blind",
            f_meas=f_meas, f_target=f_target,
            width=width, obj_z=obj_z, compression=compression,
            left=f["left"], right=f["right"],
        )

        if step % 35 == 0:
            print(
                f"  t~{step*sim_dt:.2f}s [blind] "
                f"F={f_meas:.1f}N  w={width:.4f}m  dW={compression*1000:.1f}mm"
            )

        if soft_object and crushed and compression >= BLIND_CRUSH_SHOW * 0.85:
            print("  [捏扁展示] 保持压扁姿态，便于观察...")
            # 只更新一次最终外形，然后纯物理步进，避免持续 reset 闪烁
            update_sponge_crush_visual(
                body, fixed_pos, fixed_orn, compression,
                max_c=BLIND_CRUSH_SHOW, last_state=crush_vis,
            )
            for _ in range(160):
                drive_arm(
                    robot, arm_joints, hold_arm,
                    force=HOLD_ARM_FORCE, pg=0.7, vg=1.3,
                )
                set_gripper(robot, width, force=GRIP_MOTOR_BLIND, keep_center=True)
                p.stepSimulation()
                time.sleep(sim_dt)
            break

        if width <= CLOSE_CENTER + 1e-4:
            if soft_object and width_at_contact is not None:
                if (width_at_contact - width) >= BLIND_CRUSH_WIDTH:
                    crushed = True
            break

    if crushed:
        print(f"  [捏扁] {label} 在无力控下被过度压缩！")
    else:
        print(f"  [结束] 无脑闭合完成 F_peak={f_peak:.1f}N width={width:.4f}m")

    info = {
        "f_target": f_target,
        "kind": "软/易碎物(无脑)" if soft_object else "刚体/硬物(无脑)",
        "crushed": crushed,
        "width": width,
        "final_force": read_force_smooth(robot, body, n=5)["total"],
        "recorder": rec,
        "mode": "blind",
        "f_peak": f_peak,
    }
    return width, info


def lift_with_force_hold(robot, arm_joints, ee_link, body, orn, width,
                         f_target, lift_dz=0.15, steps=400, force_control=True):
    """抬升：力控时微修正握力；偏 Y 位姿用分段抬升提高 IK 可达性。"""
    cur = get_current_tcp(robot, ee_link)
    w = width
    z0 = p.getBasePositionAndOrientation(body)[0][2]
    min_f = 1e9
    motor = (GRIP_MOTOR_HOLD + 120) if force_control else GRIP_MOTOR_BLIND

    # 分段抬升目标：每段略向 y=0 / 前方收一点，避免 +Y 极限位姿抬升 IK 失败
    waypoints = []
    n_seg = 4
    for i in range(1, n_seg + 1):
        s = i / n_seg
        wp = cur.copy()
        wp[2] = cur[2] + s * lift_dz
        wp[1] = cur[1] * (1.0 - 0.25 * s)  # 向中线收
        wp[0] = cur[0] - 0.015 * s          # 略回撤，离开远伸极限
        waypoints.append(wp)

    q_cur = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    for wi, tgt in enumerate(waypoints):
        goal, err = _solve_ik_no_teleport(
            robot, arm_joints, ee_link, tgt, q_cur, orn,
        )
        if err > 0.055:
            # 回退：只抬 Z，不改 XY
            tgt2 = cur.copy()
            tgt2[2] = tgt[2]
            goal, err = _solve_ik_no_teleport(
                robot, arm_joints, ee_link, tgt2, q_cur, orn,
            )
        if err > 0.06:
            if wi == 0:
                raise RuntimeError(f"抬升 IK 失败 {err*1000:.1f} mm")
            print(f"  [抬升] 第{wi+1}段 IK 偏大({err*1000:.1f}mm)，提前结束")
            break

        travel = float(np.linalg.norm(goal - q_cur))
        seg_steps = int(max(steps // n_seg, travel / (JOINT_STEP * 0.85), 80))
        for i in range(1, seg_steps + 1):
            s = _smoothstep5(i / seg_steps)
            q = q_cur + s * (goal - q_cur)
            drive_arm(robot, arm_joints, q, force=HOLD_ARM_FORCE, pg=0.55)
            f = read_force(robot, body)
            min_f = min(min_f, f["total"])
            if force_control and f_target > 1.0:
                if f["total"] < 0.65 * f_target:
                    w = max(CLOSE_CENTER, w - 5e-5)
                elif f["total"] > f_target + 18.0:
                    w = min(OPEN, w + 2e-5)
            set_gripper(robot, w, force=motor, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
        q_cur = goal.copy()
        cur = get_current_tcp(robot, ee_link)

    z1 = p.getBasePositionAndOrientation(body)[0][2]
    return {
        "lifted": float(z1 - z0),
        "min_force": float(min_f if min_f < 1e8 else 0.0),
        "final_z": float(z1),
        "width": w,
    }


# =====================================================================
# 单物体完整流程
# =====================================================================

def approach_object(robot, arm_joints, ee_link, obj_pos, orn, off_local, body):
    """
    夹爪去找固定物体。
    接近阶段关碰撞；物体保持冻结，运动过程不反复 reset。
    EE 目标一律用标定 off_local（ee_for_mid），避免张开夹爪时
    实时 mid−ee 的 Y 偏置把目标拉到不可达位。
    """
    obj_pos = np.asarray(obj_pos, dtype=float).copy()
    obj_pos[2] = CUBE_HALF
    obj_orn = TABLE_ORN
    set_robot_obj_collision(robot, body, enable=False)
    pin_body(body, obj_pos, obj_orn)

    # 粗预位：物体正上方（标定偏移）
    ee_g = ee_for_mid(obj_pos, orn, off_local)
    pre = ee_g.copy()
    pre[2] = obj_pos[2] + PRE_CLEAR
    print(f"  预抓取（物体固定于 {np.round(obj_pos, 3)}）...")
    move_tcp(robot, arm_joints, ee_link, pre, orn, OPEN, steps=220, allow_via=True)

    # 先水平对准再下降（仍用标定 EE，不用实时 mid−ee）
    pre2 = ee_g.copy()
    pre2[2] = obj_pos[2] + PRE_CLEAR
    try:
        move_tcp(robot, arm_joints, ee_link, pre2, orn, OPEN, steps=160, allow_via=False)
    except RuntimeError as e:
        print(f"  [预位精调] {e}")

    print("  下降并对准指心...")
    try:
        move_tcp(robot, arm_joints, ee_link, ee_g, orn, OPEN, steps=240, allow_via=False)
    except RuntimeError as e:
        print(f"  [下降] {e}")
        # 可达性不够时：保持当前 XY，只压低 Z
        down = get_current_tcp(robot, ee_link).copy()
        down[2] = max(float(ee_g[2]), EE_Z_MIN)
        try:
            move_tcp(robot, arm_joints, ee_link, down, orn, OPEN, steps=180, allow_via=False)
        except RuntimeError as e2:
            print(f"  [下降回退] {e2}")

    for it in range(8):
        mid, _, _ = get_finger_mid(robot)
        err = obj_pos - mid
        # 张开态 Z 残差受 EE_Z_MIN 限制，以 XY 收敛为准
        mid_err_xy = float(np.linalg.norm(err[:2]))
        if mid_err_xy < 0.007:
            break
        corr = ee_for_mid(obj_pos, orn, off_local)
        # 闭环：把测得的 XY 误差叠到标定目标上
        corr[0] += err[0]
        corr[1] += err[1]
        corr[2] = max(float(ee_g[2]), EE_Z_MIN)
        try:
            move_tcp(
                robot, arm_joints, ee_link, corr, orn, OPEN,
                steps=120 + 10 * it, allow_via=False,
            )
        except RuntimeError as e:
            print(f"  [对准{it}] 跳过: {e}")
            break

    mid, left, right = get_finger_mid(robot)
    span = float(np.linalg.norm(right - left))
    mid_err_xy = float(np.linalg.norm((mid - obj_pos)[:2]))
    mid_err = float(np.linalg.norm(mid - obj_pos))
    # 接近结束再钉一次（冻结态，无弹跳）
    pin_body(body, obj_pos, obj_orn)
    print(
        f"  指心={np.round(mid, 4)}  span={span:.3f}m  "
        f"mid_err_xy={mid_err_xy*1000:.1f}mm  mid_err={mid_err*1000:.1f}mm"
    )
    print(f"  物体仍固定于 {np.round(obj_pos, 4)}")
    return obj_pos.copy()


def recover_home(robot, arm_joints, ee_link, home_pos, orn):
    """平滑回位；失败则抬高再试；仍失败则短暂关渲染复位关节（避免下一抓起点漂）。"""
    open_gripper(robot)
    try:
        move_tcp(
            robot, arm_joints, ee_link, home_pos, orn, OPEN,
            steps=180, allow_via=True,
        )
        return
    except RuntimeError as e:
        print(f"  [回位] 直接回位失败({e})，抬高后重试")
    cur = get_current_tcp(robot, ee_link)
    high = cur.copy()
    high[2] = max(float(home_pos[2]), float(cur[2]) + 0.08)
    try:
        move_tcp(
            robot, arm_joints, ee_link, high, orn, OPEN,
            steps=120, allow_via=False,
        )
        move_tcp(
            robot, arm_joints, ee_link, home_pos, orn, OPEN,
            steps=160, allow_via=False,
        )
        return
    except Exception as e:
        print(f"  [回位] 仍失败: {e}，静默关节复位")
    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    goal, err = _solve_ik_no_teleport(
        robot, arm_joints, ee_link, home_pos, start, orn,
    )
    if err > 0.05:
        # 用零位附近种子再试
        seed = np.zeros(len(arm_joints), dtype=float)
        goal, err = _solve_ik_no_teleport(
            robot, arm_joints, ee_link, home_pos, seed, orn,
        )
    if err <= 0.08:
        _set_rendering(False)
        try:
            for j, a in zip(arm_joints, goal):
                p.resetJointState(robot, j, float(a))
            for _ in range(30):
                if not _physics_connected():
                    break
                drive_arm(robot, arm_joints, goal, force=HOLD_ARM_FORCE, pg=0.8)
                p.stepSimulation()
                time.sleep(SIM_DT)
        except p.error:
            pass
        finally:
            _set_rendering(True)
    else:
        print(f"  [回位] 关节复位也不可达 err={err*1000:.1f}mm")


def freeze_body(body, pos=None, orn=None):
    """原地冻结（保持可见、水平姿态）。"""
    if pos is None:
        pos = p.getBasePositionAndOrientation(body)[0]
    if orn is None:
        orn = TABLE_ORN
    return pin_body(body, pos, orn)


def unfreeze_body(body, pos, orn, mass, soft=False, friction=None):
    """解冻并恢复动力学；姿态强制水平，避免加载残留倾斜。"""
    pos = np.asarray(pos, dtype=float).copy()
    pos[2] = CUBE_HALF
    orn = TABLE_ORN if orn is None else orn
    p.resetBasePositionAndOrientation(body, pos.tolist(), list(orn))
    p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
    if soft:
        restore_sponge_dynamics(
            body, mass=mass, friction=2.2 if friction is None else friction,
        )
    else:
        restore_iron_dynamics(
            body, mass=mass, friction=3.2 if friction is None else friction,
        )


def grasp_one(robot, arm_joints, ee_link, body, obj_pos, orn, off_local,
              label, home_pos, force_control=True, soft_object=False, mass=0.15):
    """对单个物体完成接近→力控/无脑闭合→抬升→回放的完整流程。"""
    mode = "力控开启" if force_control else "力控关闭/无脑闭合"
    print(f"\n{'=' * 60}")
    print(f"  抓取: {label}  [{mode}]")
    print(f"  目标固定位置: {np.round(obj_pos, 4)}")
    print(f"{'=' * 60}")
    obj_pos = np.asarray(obj_pos, dtype=float).copy()
    obj_pos[2] = CUBE_HALF
    obj_orn = TABLE_ORN
    # 姿态由物体固定位置决定（夹爪去够，不挪物体）
    orn = grasp_orn_for_pos(obj_pos)
    print(f"  抓取姿态 euler={np.round(p.getEulerFromQuaternion(orn), 3)}")
    # 接近前保持冻结：位置/姿态不变，不被臂/地面弹跳扰动
    pin_body(body, obj_pos, obj_orn)

    open_gripper(robot)
    settle(12)
    # 先侧向预热到目标同侧，避免 +Y 从 home 直达时对准漂到 5cm 外
    side = np.asarray(home_pos, dtype=float).copy()
    side[0] = 0.30
    side[1] = 0.55 * float(obj_pos[1])
    side[2] = max(float(home_pos[2]), float(obj_pos[2]) + PRE_CLEAR + 0.02)
    try:
        move_tcp(
            robot, arm_joints, ee_link, side, orn, OPEN,
            steps=160, allow_via=True,
        )
    except RuntimeError as e:
        print(f"  [侧向预热] {e}")
    approach_object(robot, arm_joints, ee_link, obj_pos, orn, off_local, body)
    mid, _, _ = get_finger_mid(robot)
    if float(np.linalg.norm((mid - obj_pos)[:2])) > 0.015:
        print("  [对准] mid_err_xy 偏大，回位后二次接近...")
        recover_home(robot, arm_joints, ee_link, home_pos, p.getQuaternionFromEuler(HOME_EULER))
        try:
            move_tcp(
                robot, arm_joints, ee_link, side, orn, OPEN,
                steps=160, allow_via=True,
            )
        except RuntimeError:
            pass
        try:
            approach_object(robot, arm_joints, ee_link, obj_pos, orn, off_local, body)
        except RuntimeError as e:
            print(f"  [对准] 二次接近失败: {e}")

    # 闭合前解冻一次：恢复质量，姿态仍水平贴桌
    unfreeze_body(body, obj_pos, obj_orn, mass, soft=soft_object)
    settle(6)

    if force_control:
        width, info = force_controlled_grasp(robot, arm_joints, body, label)
        prefix = f"adaptive_fc_{label}"
    else:
        width, info = blind_close_grasp(
            robot, arm_joints, body, label, soft_object=soft_object,
            fixed_pos=obj_pos, fixed_orn=obj_orn,
        )
        prefix = f"blind_{label}"
    info["recorder"].save(prefix)

    if (not force_control) and soft_object and info["crushed"]:
        print("\n=== 跳过抬升（易碎品已被捏扁）===")
        print(f"[结果] {label} 失败 FAIL  crushed=True force_ctrl=False")
        print("  松开并回位...")
        for w in np.linspace(width, OPEN, 80):
            set_gripper(robot, float(w), force=300, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
        try:
            p.changeVisualShape(body, -1, rgbaColor=[0.95, 0.75, 0.35, 1.0])
        except Exception:
            pass
        pin_body(body, obj_pos, TABLE_ORN)
        set_robot_obj_collision(robot, body, enable=True)
        recover_home(robot, arm_joints, ee_link, home_pos, p.getQuaternionFromEuler(HOME_EULER))
        return {
            "label": label,
            "success": False,
            "kind": info["kind"],
            "f_target": info["f_target"],
            "lifted": 0.0,
            "crushed": True,
            "final_force": info["final_force"],
            "force_control": force_control,
        }

    # 抬升前保压：力已够则不再收紧；不钉死 XY（钉死+闭合会产生虚高接触力并挤飞物体）
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    for _ in range(40):
        drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
        f = read_force(robot, body)
        if force_control and info["f_target"] > 1.0:
            if f["total"] < 0.85 * info["f_target"]:
                width = max(CLOSE_CENTER, width - 4e-5)
            elif f["total"] > info["f_target"] + 15.0:
                width = min(OPEN, width + 2e-5)
        set_gripper(
            robot, width,
            force=GRIP_MOTOR_HOLD if force_control else GRIP_MOTOR_BLIND,
            keep_center=True,
        )
        p.stepSimulation()
        time.sleep(SIM_DT)
    info["final_force"] = read_force_smooth(robot, body, n=5)["total"]
    print(f"  抬升前保压 F={info['final_force']:.1f}N  width={width:.4f}m")

    # 刚体抬升前略增指面摩擦，减少偏 Y 位姿滑脱
    if not soft_object:
        for link in (LEFT_LINK, RIGHT_LINK):
            p.changeDynamics(
                robot, link,
                lateralFriction=3.5, spinningFriction=0.25, rollingFriction=0.02,
            )

    print("\n=== 抬升验证 ===")
    try:
        lift = lift_with_force_hold(
            robot, arm_joints, ee_link, body, orn, width, info["f_target"],
            force_control=force_control,
        )
    except RuntimeError as e:
        print(f"  [抬升失败] {e}")
        lift = {"lifted": 0.0, "min_force": 0.0, "final_z": 0.0, "width": width}

    print(
        f"  提升高度={lift['lifted']:.3f}m  "
        f"过程最小力={lift['min_force']:.2f}N  "
        f"mode={info.get('mode')}  crushed={info['crushed']}"
    )

    lifted_ok = lift["lifted"] > 0.06
    if force_control:
        if soft_object or ("软" in info["kind"]):
            success = lifted_ok and (not info["crushed"])
        else:
            success = lifted_ok and info["final_force"] >= 0.4 * max(info["f_target"], 1.0)
    else:
        if soft_object:
            success = False
            print("  [对比] 无力控对易碎品不安全 → 失败")
        else:
            success = lifted_ok
            if not success:
                print("  [对比] 无力控抓取不稳定/抬升失败")

    if success:
        print(f"[结果] {label} 成功 OK  ({info['kind']})")
    else:
        print(
            f"[结果] {label} 失败 FAIL  "
            f"lifted={lifted_ok} crushed={info['crushed']} force_ctrl={force_control}"
        )

    print("  松开并回位...")
    for link in (LEFT_LINK, RIGHT_LINK):
        p.changeDynamics(
            robot, link,
            lateralFriction=2.0, spinningFriction=0.1, rollingFriction=0.01,
        )
    cur = get_current_tcp(robot, ee_link)
    down = cur.copy()
    down[2] = max(obj_pos[2] + 0.04, EE_Z_MIN + 0.02)
    try:
        move_tcp(
            robot, arm_joints, ee_link, down, orn, lift["width"],
            steps=160, allow_via=False,
        )
    except RuntimeError:
        pass
    for w in np.linspace(lift["width"], OPEN, 70):
        set_gripper(robot, float(w), force=300, keep_center=True)
        p.stepSimulation()
        time.sleep(SIM_DT)
    try:
        color = [0.95, 0.75, 0.35, 1.0] if soft_object else [0.35, 0.35, 0.38, 1.0]
        p.changeVisualShape(body, -1, rgbaColor=color)
    except Exception:
        pass
    pin_body(body, obj_pos, TABLE_ORN)
    set_robot_obj_collision(robot, body, enable=True)
    recover_home(robot, arm_joints, ee_link, home_pos, p.getQuaternionFromEuler(HOME_EULER))

    return {
        "label": label,
        "success": success,
        "kind": info["kind"],
        "f_target": info["f_target"],
        "lifted": lift["lifted"],
        "crushed": info["crushed"],
        "final_force": info["final_force"],
        "force_control": force_control,
    }



# =====================================================================
# 主流程
# =====================================================================

def parse_args():
    """解析 --force-control / --only / --direct 等参数。"""
    ap = argparse.ArgumentParser(description="自适应力控抓取演示")
    ap.add_argument("--direct", action="store_true", help="无 GUI")
    ap.add_argument("--no-show", action="store_true", help="不弹最终图")
    ap.add_argument("--only", choices=["all", "iron", "sponge"], default="all",
                    help="只测某一类物体")
    ap.add_argument("--iron-mass", type=float, default=0.55)
    ap.add_argument("--sponge-mass", type=float, default=0.04)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--force-control", dest="force_control", action="store_true",
                   help="强制开启力控（覆盖代码开关）")
    g.add_argument("--no-force-control", dest="force_control", action="store_false",
                   help="强制关闭力控（无脑闭合对照）")
    ap.set_defaults(force_control=None)
    return ap.parse_args()



def connect_pybullet(direct=False, width=960, height=720):
    """安全连接：先断开残留会话，再开新 GUI/DIRECT。"""
    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass
    if direct:
        cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.GUI, options=f"--width={width} --height={height}")
    if cid < 0 or not _physics_connected():
        raise RuntimeError("无法连接 PyBullet 物理服务器（请关闭残留 GUI 后重试）")
    return cid


def main():
    """任务2主流程：同场景铁块+海绵，演示自适应力控抓取。"""
    args = parse_args()
    force_control = (
        ENABLE_FORCE_CONTROL if args.force_control is None else bool(args.force_control)
    )

    if args.direct or args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    connect_pybullet(direct=bool(args.direct))
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setPhysicsEngineParameter(numSolverIterations=120, numSubSteps=4)

    robot_data = load_robot()
    robot = robot_data["robot"]
    arm_joints = robot_data["arm_joints"]
    ee_link = robot_data["ee_link"]
    init_gripper(robot_data)
    init_ik(robot_data)

    print("\n========== 自适应力控抓取 ==========")
    print(f"  力控开关: {'开启 ON' if force_control else '关闭 OFF（无脑闭合）'}")
    print("  （可改代码 ENABLE_FORCE_CONTROL，或用 --force-control / --no-force-control）")
    print(f"  F_soft={F_SOFT_SAFE}N  F_hard={F_HARD_SAFE}N  k_hard={K_HARD}N/m")
    print(f"  捏扁阈值={CRUSH_WIDTH*1000:.0f}mm  F_abs_max={F_ABS_MAX}N")
    print("====================================\n")

    # 同场景双物体：贴地水平固定；加载后冻结，互不弹跳/倾斜
    iron_pos0 = np.array([0.38, -0.10, CUBE_HALF], dtype=float)
    sponge_pos0 = np.array([0.38, 0.10, CUBE_HALF], dtype=float)
    iron = load_iron_block(iron_pos0.tolist(), mass=args.iron_mass, pin=True)
    sponge = load_sponge(sponge_pos0.tolist(), mass=args.sponge_mass, pin=True)
    # 两物体互不碰撞，避免贴得近时推挤
    p.setCollisionFilterPair(iron, sponge, -1, -1, 0)

    print("[初始化] 场景就绪（cube 已冻结贴桌，位置/姿态锁定）...")
    # 只步进机器人/地面，cube 为 mass=0 不会沉降抖动
    settle(20)
    iron_orn0 = TABLE_ORN
    sponge_orn0 = TABLE_ORN
    pin_body(iron, iron_pos0, iron_orn0)
    pin_body(sponge, sponge_pos0, sponge_orn0)
    iron_pos = iron_pos0.copy()
    sponge_pos = sponge_pos0.copy()
    print(f"  固定位 Iron={np.round(iron_pos, 3)}  Sponge={np.round(sponge_pos, 3)}")

    home_orn = p.getQuaternionFromEuler(HOME_EULER)
    home_pos = np.array([0.28, 0.0, 0.22], dtype=float)

    open_gripper(robot)
    settle(10)
    move_tcp(robot, arm_joints, ee_link, home_pos, home_orn, OPEN, steps=180)
    off_local = calibrate_off_local(robot, ee_link)
    results = []

    tasks = []
    if args.only in ("all", "iron"):
        tasks.append(("IronBlock", iron, iron_pos, False, args.iron_mass, iron_orn0))
    if args.only in ("all", "sponge"):
        tasks.append(("Sponge", sponge, sponge_pos, True, args.sponge_mass, sponge_orn0))

    try:
        for label, body, pos, soft, mass, born in tasks:
            other = sponge if body is iron else iron
            other_mass = args.sponge_mass if other is sponge else args.iron_mass
            other_fixed = sponge_pos if other is sponge else iron_pos
            other_orn = sponge_orn0 if other is sponge else iron_orn0
            freeze_body(other, other_fixed, other_orn)
            set_robot_obj_collision(robot, other, enable=False)

            # 目标物体保持冻结贴桌，夹爪过去抓
            grasp_pos = np.asarray(pos, dtype=float).copy()
            grasp_pos[2] = CUBE_HALF
            pin_body(body, grasp_pos, TABLE_ORN)
            settle(5)

            try:
                r = grasp_one(
                    robot, arm_joints, ee_link, body, grasp_pos, home_orn, off_local,
                    label, home_pos,
                    force_control=force_control, soft_object=soft, mass=mass,
                )
            except Exception as e:
                print(f"  [异常] {label}: {e}")
                traceback.print_exc()
                r = {
                    "label": label, "success": False, "kind": "未知",
                    "f_target": 0.0, "lifted": 0.0, "crushed": False,
                    "final_force": 0.0, "force_control": force_control,
                }
            results.append(r)

            # 抓完后钉回固定展示位并冻结
            pin_body(body, grasp_pos, TABLE_ORN)
            try:
                color = [0.95, 0.75, 0.35, 1.0] if soft else [0.35, 0.35, 0.38, 1.0]
                p.changeVisualShape(body, -1, rgbaColor=color)
            except Exception:
                pass
            set_robot_obj_collision(robot, body, enable=True)

            pin_body(other, other_fixed, other_orn)
            # 恢复另一物体质量供下次抓取，但仍先冻结展示
            if other is sponge:
                restore_sponge_dynamics(other, mass=other_mass)
            else:
                restore_iron_dynamics(other, mass=other_mass)
            pin_body(other, other_fixed, other_orn)
            set_robot_obj_collision(robot, other, enable=True)
            recover_home(robot, arm_joints, ee_link, home_pos, home_orn)
            settle(20)

    except Exception as e:
        print(f"\n[异常] {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        print("\n========== 汇总 ==========")
        print(f"  力控: {'ON' if force_control else 'OFF'}")
        for r in results:
            flag = "OK" if r["success"] else "FAIL"
            print(
                f"  [{flag}] {r['label']}: {r['kind']}  "
                f"F_safe={r['f_target']:.1f}N  "
                f"抬升={r['lifted']:.3f}m  crushed={r['crushed']}"
            )
        expected = len(tasks)
        if force_control:
            ok = len(results) == expected and all(r["success"] for r in results)
        else:
            soft_fail = any(
                (not r["success"]) and (
                    "Sponge" in r["label"] or "软" in r.get("kind", "")
                )
                for r in results
            )
            ok = soft_fail or (args.only == "iron")
            print("  （无力控对照：易碎品应失败 / 被捏扁）")
        n_ok = sum(1 for r in results if r["success"])
        print(f"总体: {'通过' if ok else '未全部通过'}  ({n_ok}/{expected})")
        print("==========================\n")
        if not args.direct:
            print("关闭 GUI 窗口结束...")
            try:
                while p.isConnected():
                    p.stepSimulation()
                    time.sleep(0.05)
            except Exception:
                pass
        try:
            p.disconnect()
        except Exception:
            pass
        if force_control and not ok:
            sys.exit(1)


if __name__ == "__main__":
    main()
