"""
4.slip_compensation_test.py
============================================================
双向滑移对冲测试（Slip Compensation / Anti-Slip）

进阶场景：
  1. 力控抓取并抬起物体
  2. 对物体施加持续向下的外力（模拟负载突变 / 额外重力）
  3. 通过指尖接触力 + 相对滑移速度，感知“摩擦力/接触异常”
  4. 滑移对冲开启时：自动略微加大抓紧力，防止滑落
  5. 滑移对冲关闭时：固定握宽，易在外力下掉落（对照）

“双向”含义：
  - 左右指正压力失衡时对称收紧（双侧同时加紧）
  - 相对滑移（物体相对指心向下/向上）都触发对冲（抗滑能力双向）

运行（在 simulation2 目录）:
  python 3.slip_compensation_test.py
  python 3.slip_compensation_test.py --compensate
  python 3.slip_compensation_test.py --no-compensate
  python 3.slip_compensation_test.py --direct --no-show --disturb 40

依赖本目录 2.adaptive_force_control_grasp.py 中的抓取 / 力读工具。
说明见 README.md
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data

from utils.robot import load_robot
from utils.gripper import (
    init_gripper,
    LEFT_LINK,
    RIGHT_LINK,
    OPEN,
    CLOSE_CENTER,
    set_gripper,
    open_gripper,
)
from utils.ik import init_ik, get_current_tcp


# =====================================================================
# 【滑移对冲开关】改这里，或用 --compensate / --no-compensate
# =====================================================================
ENABLE_SLIP_COMPENSATION = True

SIM_DT = 1.0 / 240.0
HOME_EULER = [0.0, 0.0, 0.0]

# 扰动与检测
DISTURB_FORCE_N = 40.0       # 等效向下外力 (N)；μ≈1.0 时需略增 Fn 才能稳住
DISTURB_RAMP_S = 0.8
BASELINE_S = 0.8
TEST_S = 3.5
SLIP_VEL_TH = 0.018
SLIP_DZ_TH = 0.0025
F_DROP_RATIO = 0.70
FRICTION_JUMP = 1.45
F_ABS_MAX = 70.0
COMP_DW = 1.2e-4
COMP_DW_MAX = 3.5e-4
RELEASE_DW = 1.0e-5
GRIP_MOTOR_HOLD = 520
HOLD_ARM_FORCE = 3200
DROP_CONFIRM = 25


def _load_afc():
    """加载 2.adaptive_force_control_grasp 作为工具库（不跑 main）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "2.adaptive_force_control_grasp.py")
    spec = importlib.util.spec_from_file_location("afc2", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_contact_bundle(robot, body):
    """
    正压力 + 切向摩擦力（左右指相对物体）。
    PyBullet contact: normalForce=pt[9], lateralFriction1/2=pt[10]/pt[11]
    """
    left = p.getContactPoints(bodyA=robot, linkIndexA=LEFT_LINK, bodyB=body)
    right = p.getContactPoints(bodyA=robot, linkIndexA=RIGHT_LINK, bodyB=body)

    def _agg(pts):
        n = sum(pt[9] for pt in pts)
        # pt[10]=lateralFriction1, pt[12]=lateralFriction2（pt[11]/[13] 是方向向量）
        t = 0.0
        for pt in pts:
            t += abs(float(pt[10]))
            if len(pt) > 12:
                t += abs(float(pt[12]))
        return float(n), float(t)

    ln, lt = _agg(left)
    rn, rt = _agg(right)
    return {
        "left_n": ln, "right_n": rn,
        "total_n": ln + rn,
        "left_t": lt, "right_t": rt,
        "total_t": lt + rt,
        "balance": abs(ln - rn),
    }


def finger_and_obj_state(robot, body):
    mid, left, right = _AFC.get_finger_mid(robot)
    obj_p, _ = p.getBasePositionAndOrientation(body)
    obj_p = np.array(obj_p, dtype=float)
    obj_v, _ = p.getBaseVelocity(body)
    obj_v = np.array(obj_v, dtype=float)
    # 指心速度（左右 link 线速度平均）
    lv = np.array(p.getLinkState(robot, LEFT_LINK, computeLinkVelocity=1)[6], dtype=float)
    rv = np.array(p.getLinkState(robot, RIGHT_LINK, computeLinkVelocity=1)[6], dtype=float)
    mid_v = 0.5 * (lv + rv)
    rel_z = float(obj_p[2] - mid[2])
    slip_vz = float(obj_v[2] - mid_v[2])  # <0：物体相对指心下滑
    return {
        "mid": mid, "obj_p": obj_p, "obj_v": obj_v,
        "mid_v": mid_v, "rel_z": rel_z, "slip_vz": slip_vz,
    }


class SlipRecorder:
    def __init__(self, sim_dt=SIM_DT):
        self.sim_dt = sim_dt
        self.t = 0.0
        self.rows = []

    def log(self, **kw):
        row = {"time": self.t, **kw}
        self.rows.append(row)
        self.t += self.sim_dt

    def save(self, prefix="slip_comp"):
        os.makedirs("force_data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join("force_data", f"{prefix}_{stamp}.csv")
        keys = [
            "time", "phase", "disturb_n", "f_n", "f_t", "f_target",
            "width", "slip_vz", "rel_z", "rel_z0", "slip_dz",
            "balance", "comp_on", "anomaly",
        ]
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                w.writerow(r)
        print(f"[保存] {csv_path}")

        import matplotlib.pyplot as plt
        if not self.rows:
            return csv_path, None
        t = [r["time"] for r in self.rows]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(t, [r["f_n"] for r in self.rows], label="Fn 正压力")
        axes[0].plot(t, [r["f_t"] for r in self.rows], label="Ft 切向(摩擦)", alpha=0.85)
        axes[0].plot(t, [r["f_target"] for r in self.rows], "--", label="F_target")
        axes[0].plot(t, [r["disturb_n"] for r in self.rows], ":", label="向下外力")
        axes[0].set_ylabel("Force (N)")
        axes[0].legend(loc="upper left", fontsize=8)
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title("滑移对冲：力 / 扰动")

        axes[1].plot(t, [r["slip_vz"] for r in self.rows], label="相对滑移速度 vz")
        axes[1].axhline(-SLIP_VEL_TH, color="r", ls="--", alpha=0.5, label="阈值")
        axes[1].set_ylabel("m/s")
        axes[1].legend(loc="upper left", fontsize=8)
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t, [r["slip_dz"] * 1000 for r in self.rows], label="相对下滑量")
        axes[2].plot(t, [(r["width"] - self.rows[0]["width"]) * 1000 for r in self.rows],
                      label="握宽变化 Δw")
        axes[2].set_ylabel("mm")
        axes[2].set_xlabel("time (s)")
        axes[2].legend(loc="upper left", fontsize=8)
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        png_path = csv_path.replace(".csv", ".png")
        fig.savefig(png_path, dpi=140)
        print(f"[绘图] {png_path}")
        plt.close(fig)
        return csv_path, png_path


def detect_anomaly(f, st, base, rel_z0):
    """
    摩擦力/接触异常检测（多信号融合）：
      1) 相对滑移速度超阈（双向：|slip_vz|）
      2) 相对指心下落超阈
      3) 正压力相对基线塌陷
      4) 切向摩擦相对基线突增（外力加载时摩擦需求上升）
      5) 左右力严重失衡
    """
    reasons = []
    if abs(st["slip_vz"]) >= SLIP_VEL_TH:
        reasons.append("slip_vz")
    slip_dz = rel_z0 - st["rel_z"]  # >0 表示相对下滑
    if slip_dz >= SLIP_DZ_TH:
        reasons.append("slip_dz")
    if base["f_n"] > 1.0 and f["total_n"] < F_DROP_RATIO * base["f_n"]:
        reasons.append("fn_drop")
    if base["f_t"] > 0.5 and f["total_t"] > FRICTION_JUMP * base["f_t"] and f["total_n"] < 1.05 * base["f_n"]:
        # 仅当正压力未同步上升时，切向突增才算异常（避免外力下正常 Ft 升高误触发）
        reasons.append("ft_jump")
    if f["balance"] > 12.0 and f["total_n"] > 5.0:
        reasons.append("imbalance")
    return len(reasons) > 0, reasons, float(slip_dz)


def simple_vertical_lift(robot, arm_joints, ee_link, body, orn, width,
                         f_target, lift_dz=0.12):
    """仅抬 Z 的稳妥抬升（避免分段侧向收拢导致握持丢失）。"""
    cur = get_current_tcp(robot, ee_link)
    tgt = cur.copy()
    tgt[2] += lift_dz
    start = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    goal, err = _AFC._solve_ik_no_teleport(
        robot, arm_joints, ee_link, tgt, start, orn,
    )
    if err > 0.05:
        raise RuntimeError(f"抬升 IK 失败 {err*1000:.1f} mm")
    w = float(width)
    z0 = p.getBasePositionAndOrientation(body)[0][2]
    min_f = 1e9
    steps = int(max(280, np.linalg.norm(goal - start) / 0.0020))
    for i in range(1, steps + 1):
        s = _AFC._smoothstep5(i / steps)
        q = start + s * (goal - start)
        _AFC.drive_arm(robot, arm_joints, q, force=HOLD_ARM_FORCE, pg=0.55)
        f = _AFC.read_force(robot, body)
        min_f = min(min_f, f["total"])
        if f["total"] < 0.7 * f_target:
            w = max(CLOSE_CENTER, w - 6e-5)
        set_gripper(robot, w, force=GRIP_MOTOR_HOLD, keep_center=True)
        p.stepSimulation()
        time.sleep(SIM_DT)
    z1 = p.getBasePositionAndOrientation(body)[0][2]
    return {
        "lifted": float(z1 - z0),
        "min_force": float(min_f if min_f < 1e8 else 0.0),
        "width": w,
    }


def slip_compensation_hold(
    robot, arm_joints, body, width, f_hold,
    compensate=True,
    disturb_n=DISTURB_FORCE_N,
    baseline_s=BASELINE_S,
    test_s=TEST_S,
    ramp_s=DISTURB_RAMP_S,
    mass=0.55,
):
    """
    抬升后保持臂不动；用增强重力模拟向下外力，按需动态加紧。
    """
    rec = SlipRecorder()
    hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints], dtype=float)
    w = float(width)
    f_target = float(max(f_hold, 8.0))
    st0 = finger_and_obj_state(robot, body)
    rel_z0 = st0["rel_z"]
    z0 = float(st0["obj_p"][2])
    mass = max(float(mass), 0.05)

    print("\n=== 基线观测（无外力）===")
    base_acc = {"f_n": 0.0, "f_t": 0.0, "n": 0}
    n_base = max(1, int(baseline_s / SIM_DT))
    for i in range(n_base):
        _AFC.drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
        set_gripper(robot, w, force=GRIP_MOTOR_HOLD, keep_center=True)
        f = read_contact_bundle(robot, body)
        st = finger_and_obj_state(robot, body)
        base_acc["f_n"] += f["total_n"]
        base_acc["f_t"] += f["total_t"]
        base_acc["n"] += 1
        anom, reasons, slip_dz = detect_anomaly(f, st, {"f_n": 1e9, "f_t": 1e9}, rel_z0)
        rec.log(
            phase="baseline", disturb_n=0.0,
            f_n=f["total_n"], f_t=f["total_t"], f_target=f_target,
            width=w, slip_vz=st["slip_vz"], rel_z=st["rel_z"], rel_z0=rel_z0,
            slip_dz=slip_dz, balance=f["balance"],
            comp_on=int(compensate), anomaly=0,
        )
        p.stepSimulation()
        time.sleep(SIM_DT)
        if i % 40 == 0:
            print(f"  t~{i*SIM_DT:.2f}s [baseline] Fn={f['total_n']:.1f}N Ft={f['total_t']:.1f}N")

    base = {
        "f_n": base_acc["f_n"] / max(base_acc["n"], 1),
        "f_t": max(0.3, base_acc["f_t"] / max(base_acc["n"], 1)),
    }
    rel_z0 = finger_and_obj_state(robot, body)["rel_z"]
    print(f"  基线 Fn={base['f_n']:.1f}N  Ft={base['f_t']:.1f}N  rel_z0={rel_z0:.4f}m")
    print(f"\n=== 施加等效向下外力 {disturb_n:.1f}N（增强重力）| 对冲={'ON' if compensate else 'OFF'} ===")

    n_test = max(1, int(test_s / SIM_DT))
    n_ramp = max(1, int(ramp_s / SIM_DT))
    max_slip_dz = 0.0
    triggered = False
    drop_event = False
    drop_count = 0

    try:
        for i in range(n_test):
            if i < n_ramp:
                disturb = disturb_n * (i + 1) / n_ramp
            else:
                disturb = disturb_n
            # 增强重力 = 额外向下载荷 / mass，比 applyExternalForce 更不易打飞接触
            p.setGravity(0.0, 0.0, -(9.8 + float(disturb) / mass))

            _AFC.drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
            f = read_contact_bundle(robot, body)
            st = finger_and_obj_state(robot, body)
            anom, reasons, slip_dz = detect_anomaly(f, st, base, rel_z0)
            max_slip_dz = max(max_slip_dz, slip_dz)

            if compensate and anom:
                triggered = True
                # 外力下 Ft 升高是正常的；主要以滑移速度/下滑量/正压力塌陷驱动加紧
                need_close = (
                    ("slip_vz" in reasons)
                    or ("slip_dz" in reasons)
                    or ("fn_drop" in reasons)
                    or (f["total_n"] < 0.85 * f_target)
                )
                if need_close or "ft_jump" in reasons:
                    f_target = float(np.clip(
                        max(f_target, base["f_n"] * 1.15, f_hold * 1.1),
                        f_hold, F_ABS_MAX,
                    ))
                    if need_close and ("slip_vz" in reasons or "slip_dz" in reasons):
                        f_target = float(np.clip(f_target + 6.0, f_hold, F_ABS_MAX))

                if need_close and f["total_n"] < 1.25 * f_target:
                    if f["total_n"] < 1.0:
                        dw = COMP_DW_MAX
                    elif abs(st["slip_vz"]) > 2 * SLIP_VEL_TH or slip_dz > 2 * SLIP_DZ_TH:
                        dw = COMP_DW_MAX
                    else:
                        dw = COMP_DW
                    w = max(CLOSE_CENTER, w - dw)
                    if i % 30 == 0:
                        print(
                            f"  [对冲] reasons={reasons}  "
                            f"Fn={f['total_n']:.1f}/{f_target:.1f}N  "
                            f"slip_vz={st['slip_vz']:+.3f}  dZ={slip_dz*1000:.1f}mm  w={w:.4f}"
                        )
            elif compensate and (not anom) and f["total_n"] > 1.2 * f_target:
                w = min(OPEN, w + RELEASE_DW)
                f_target = max(f_hold, f_target * 0.998)

            set_gripper(robot, w, force=GRIP_MOTOR_HOLD, keep_center=True)

            if st["obj_p"][2] < z0 - 0.05:
                drop_count += 1
                if drop_count >= DROP_CONFIRM:
                    drop_event = True
            else:
                drop_count = 0

            rec.log(
                phase="disturb",
                disturb_n=disturb,
                f_n=f["total_n"], f_t=f["total_t"], f_target=f_target,
                width=w, slip_vz=st["slip_vz"], rel_z=st["rel_z"], rel_z0=rel_z0,
                slip_dz=slip_dz, balance=f["balance"],
                comp_on=int(compensate), anomaly=int(anom),
            )
            p.stepSimulation()
            time.sleep(SIM_DT)

            if i % 48 == 0:
                print(
                    f"  t~{i*SIM_DT:.2f}s [disturb] Fext={disturb:.1f}N  "
                    f"Fn={f['total_n']:.1f}N Ft={f['total_t']:.1f}N  "
                    f"slip_vz={st['slip_vz']:+.3f}  dZ={slip_dz*1000:.1f}mm"
                )

            if drop_event:
                print("  [掉落] 物体相对夹爪滑落超限，结束测试")
                break
    finally:
        p.setGravity(0.0, 0.0, -9.8)

    z1 = float(p.getBasePositionAndOrientation(body)[0][2])
    held = (
        (not drop_event)
        and (z1 > z0 - 0.045)
        and (z1 > 0.08)
        and (max_slip_dz < 0.04)
    )
    info = {
        "held": held,
        "drop": drop_event,
        "triggered": triggered,
        "max_slip_dz": max_slip_dz,
        "z0": z0, "z1": z1,
        "width": w,
        "f_target": f_target,
        "recorder": rec,
        "compensate": compensate,
        "disturb_n": disturb_n,
    }
    return info


def parse_args():
    ap = argparse.ArgumentParser(description="双向滑移对冲测试")
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--mass", type=float, default=0.55, help="物体质量 kg")
    ap.add_argument("--friction", type=float, default=0.55,
                    help="物体摩擦系数（偏低：OFF 易滑落，ON 可对冲稳住）")
    ap.add_argument("--disturb", type=float, default=DISTURB_FORCE_N,
                    help="等效向下外力 N（通过增强重力施加）")
    ap.add_argument("--cube", type=float, nargs=3, default=[0.36, 0.0, 0.025],
                    metavar=("X", "Y", "Z"))
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--compensate", dest="compensate", action="store_true",
                   help="开启滑移对冲")
    g.add_argument("--no-compensate", dest="compensate", action="store_false",
                   help="关闭滑移对冲（对照）")
    ap.set_defaults(compensate=None)
    return ap.parse_args()


def main():
    global _AFC
    args = parse_args()
    compensate = (
        ENABLE_SLIP_COMPENSATION if args.compensate is None else bool(args.compensate)
    )
    _AFC = _load_afc()

    if args.direct or args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    mode = p.DIRECT if args.direct else p.GUI
    if mode == p.GUI:
        p.connect(mode, options="--width=960 --height=720")
    else:
        p.connect(mode)
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    print("\n========== 双向滑移对冲测试 ==========")
    print(f"  滑移对冲: {'开启 ON' if compensate else '关闭 OFF'}")
    print(f"  向下外力: {args.disturb:.1f}N")
    print(f"  质量={args.mass}kg  μ={args.friction}")
    print("====================================\n")

    data = load_robot()
    robot = data["robot"]
    arm_joints = data["arm_joints"]
    ee_link = data["ee_link"]
    init_gripper(data)
    init_ik(data)

    cube_pos = np.array(args.cube, dtype=float)
    cube_pos[2] = _AFC.CUBE_HALF
    body = _AFC.load_iron_block(
        cube_pos.tolist(), mass=args.mass, friction=args.friction, pin=True,
    )
    # 不在此处再改 rollingFriction（会扰动已冻结状态）；抓取解冻时用默认动力学

    print("[初始化] 场景就绪（cube 已冻结贴桌）...")
    _AFC.settle(20)
    born = list(_AFC.TABLE_ORN)
    _AFC.pin_body(body, cube_pos, born)

    home_orn = p.getQuaternionFromEuler(HOME_EULER)
    home_pos = np.array([0.28, 0.0, 0.22], dtype=float)
    open_gripper(robot)
    _AFC.settle(8)
    _AFC.move_tcp(robot, arm_joints, ee_link, home_pos, home_orn, OPEN, steps=180)
    # 标定过程中 cube 保持冻结
    _AFC.pin_body(body, cube_pos, born)
    off_local = _AFC.calibrate_off_local(robot, ee_link)
    _AFC.pin_body(body, cube_pos, born)

    result = {
        "success": False, "held": False, "triggered": False,
        "max_slip_dz": 0.0, "compensate": compensate,
    }
    try:
        # ---- 力控抓取 + 抬升 ----
        orn = _AFC.grasp_orn_for_pos(cube_pos)
        print("\n=== 阶段 A: 力控抓取 ===")
        open_gripper(robot)
        _AFC.settle(8)
        # y≈0 时侧向预热可省略；仍抬到物体上方同侧
        side = home_pos.copy()
        side[0] = 0.30
        side[1] = float(cube_pos[1])
        side[2] = max(float(home_pos[2]), float(cube_pos[2]) + _AFC.PRE_CLEAR)
        try:
            _AFC.move_tcp(robot, arm_joints, ee_link, side, orn, OPEN, steps=160)
        except RuntimeError as e:
            print(f"  [预热] {e}")

        _AFC.approach_object(robot, arm_joints, ee_link, cube_pos, orn, off_local, body)
        mid, _, _ = _AFC.get_finger_mid(robot)
        if float(np.linalg.norm((mid - cube_pos)[:2])) > 0.015:
            print("  [对准] mid_err_xy 偏大，回位后重试...")
            _AFC.recover_home(robot, arm_joints, ee_link, home_pos, home_orn)
            try:
                _AFC.move_tcp(robot, arm_joints, ee_link, side, orn, OPEN, steps=160)
            except RuntimeError:
                pass
            _AFC.approach_object(robot, arm_joints, ee_link, cube_pos, orn, off_local, body)

        mid, _, _ = _AFC.get_finger_mid(robot)
        mid_err_xy = float(np.linalg.norm((mid - cube_pos)[:2]))
        if mid_err_xy > 0.020:
            raise RuntimeError(f"抓取对准失败 mid_err_xy={mid_err_xy*1000:.1f}mm")

        # 闭合前解冻一次
        _AFC.unfreeze_body(
            body, cube_pos, born, args.mass, soft=False, friction=args.friction,
        )
        _AFC.settle(6)

        width, info = _AFC.force_controlled_grasp(robot, arm_joints, body, "SlipCube")
        # 刚体被误判软物时，抬升前强制提到硬物握力
        f_hold = float(info.get("f_target", 20.0))
        if "软" in str(info.get("kind", "")) or f_hold < 20.0:
            f_hold = 28.0
            print(f"  [纠正] 滑移测试用硬物握力 F_hold={f_hold:.1f}N")
        # 抬升前再收紧一点，确保先抬起来
        hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
        for _ in range(40):
            _AFC.drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
            f = _AFC.read_force(robot, body)
            if f["total"] < 0.85 * f_hold:
                width = max(CLOSE_CENTER, width - 5e-5)
            set_gripper(robot, width, force=GRIP_MOTOR_HOLD, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
        print(f"  抬升前: width={width:.4f}  F_hold≈{f_hold:.1f}N")

        print("\n=== 阶段 B: 抬升 ===")
        # 抬升前略增指面摩擦
        for link in (LEFT_LINK, RIGHT_LINK):
            p.changeDynamics(
                robot, link,
                lateralFriction=3.2, spinningFriction=0.2, rollingFriction=0.02,
            )
        lift = simple_vertical_lift(
            robot, arm_joints, ee_link, body, orn, width, f_hold, lift_dz=0.12,
        )
        print(f"  抬升={lift['lifted']:.3f}m  minF={lift['min_force']:.1f}N")
        if lift["lifted"] < 0.05:
            raise RuntimeError("抬升失败，无法进行滑移对冲测试")
        width = lift["width"]

        # 抬升后保持接触，再进入临界握持：略松爪 + 降指面摩擦
        f_now = _AFC.read_force_smooth(robot, body, n=5)["total"]
        if f_now < 8.0 or float(p.getBasePositionAndOrientation(body)[0][2]) < 0.08:
            raise RuntimeError(
                f"抬升后未保持悬空握持 Fn={f_now:.1f}N "
                f"z={p.getBasePositionAndOrientation(body)[0][2]:.3f}"
            )

        for link in (LEFT_LINK, RIGHT_LINK):
            p.changeDynamics(
                robot, link,
                lateralFriction=1.15, spinningFriction=0.08, rollingFriction=0.008,
            )
        p.changeDynamics(
            body, -1,
            lateralFriction=float(args.friction),
            spinningFriction=0.05,
            rollingFriction=0.005,
        )
        width_crit = min(OPEN, float(width) + 0.00022)
        hold_arm = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
        for _ in range(28):
            _AFC.drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
            set_gripper(robot, width_crit, force=GRIP_MOTOR_HOLD, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
        f_now = _AFC.read_force_smooth(robot, body, n=5)["total"]
        z_now = float(p.getBasePositionAndOrientation(body)[0][2])
        if f_now < 8.0 or z_now < 0.08:
            width_crit = max(CLOSE_CENTER, width_crit - 0.00018)
            for _ in range(30):
                _AFC.drive_arm(robot, arm_joints, hold_arm, force=HOLD_ARM_FORCE, pg=0.75)
                set_gripper(robot, width_crit, force=GRIP_MOTOR_HOLD, keep_center=True)
                p.stepSimulation()
                time.sleep(SIM_DT)
            f_now = _AFC.read_force_smooth(robot, body, n=5)["total"]
            z_now = float(p.getBasePositionAndOrientation(body)[0][2])
        if f_now < 5.0 or z_now < 0.08:
            raise RuntimeError(
                f"临界握持失败 Fn={f_now:.1f}N z={z_now:.3f}（无法进入滑移对冲）"
            )
        width = width_crit
        f_hold_test = max(10.0, min(24.0, float(f_now)))
        print(
            f"  扰动前(临界握持): width={width:.4f}  Fn≈{f_now:.1f}N  "
            f"F_hold_test≈{f_hold_test:.1f}N  μ_obj={args.friction}"
        )

        print("\n=== 阶段 C: 向下外力(增强重力) + 滑移对冲 ===")
        slip = slip_compensation_hold(
            robot, arm_joints, body, width, f_hold_test,
            compensate=compensate,
            disturb_n=float(args.disturb),
            mass=float(args.mass),
        )
        if float(p.getBasePositionAndOrientation(body)[0][2]) < 0.06 and slip["z0"] < 0.06:
            # 防御：基线已在桌面则不算对冲成功
            slip["held"] = False
            slip["drop"] = True
        prefix = "slip_comp_ON" if compensate else "slip_comp_OFF"
        slip["recorder"].save(prefix)

        held = slip["held"]
        result.update({
            "success": held if compensate else (not held),  # 对冲开应抓住；关应对照掉/滑
            "held": held,
            "triggered": slip["triggered"],
            "max_slip_dz": slip["max_slip_dz"],
            "drop": slip["drop"],
            "z0": slip["z0"], "z1": slip["z1"],
        })

        print("\n========== 结果 ==========")
        print(f"  对冲: {'ON' if compensate else 'OFF'}")
        print(f"  触发对冲: {slip['triggered']}")
        print(f"  最大相对下滑: {slip['max_slip_dz']*1000:.1f} mm")
        print(f"  物体高度: {slip['z0']:.3f} → {slip['z1']:.3f} m")
        print(f"  掉落: {slip['drop']}")
        if compensate:
            print(f"  判定: {'通过 OK（扰动下仍稳住）' if held else '失败 FAIL（仍滑落）'}")
        else:
            print(
                f"  对照: {'出现滑落/下滑（符合预期）' if (slip['drop'] or slip['max_slip_dz']>0.012) else '未明显滑落（可加大 --disturb）'}"
            )
        print("==========================\n")

        # 松开回位
        print("  松开并回位...")
        for w in np.linspace(slip["width"], OPEN, 70):
            set_gripper(robot, float(w), force=300, keep_center=True)
            p.stepSimulation()
            time.sleep(SIM_DT)
        _AFC.pin_body(body, cube_pos, born)
        _AFC.recover_home(robot, arm_joints, ee_link, home_pos, home_orn)

    except Exception as e:
        print(f"[异常] {e}")
        import traceback
        traceback.print_exc()
        result["success"] = False
    finally:
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

    # 对冲开启时期望 held；关闭时不强制 exit code
    if compensate and not result.get("held", False):
        sys.exit(1)


if __name__ == "__main__":
    main()
