"""
utils/gripper.py

夹爪控制和力传感工具。

物理一致性说明（不改 URDF）：
  URDF 中 joint_left 行程 0~0.05、joint_right 行程 0~0.0715。
  若按比例 left=width*0.7、right=width 同步，闭合时两指几何中心
  会沿开合方向漂移约 1cm，导致单指先触、左右正压力不等。

  现实平行爪：两指相对中心对称开合。本模块默认采用
  「等量回缩」映射：从全开姿态同时减小相同 Δ，保持
  q_right - q_left = RIGHT_UPPER - LEFT_UPPER 恒定，从而指心不漂。
"""

import pybullet as p
import time
import numpy as np

# ==========================
# 默认关节和 link 索引（与 URDF 加载顺序一致）
# ==========================
LEFT_JOINT = 7
RIGHT_JOINT = 8
LEFT_LINK = 7
RIGHT_LINK = 8

# URDF 行程
LEFT_UPPER = 0.05
RIGHT_UPPER = 0.0715
# 保持指心不变时：qR - qL 恒等于该差值
STROKE_DIFF = RIGHT_UPPER - LEFT_UPPER  # 0.0215 m

OPEN = RIGHT_UPPER          # 全开（右关节指令，对应左右均到上限）
CLOSE_CENTER = STROKE_DIFF  # 对称闭合下限（再小会破坏指心）
CLOSE = 0.0                 # 绝对全闭（仅非对称模式使用）

# 兼容旧代码的比例（keep_center=False 时）
LEFT_RATIO = LEFT_UPPER / RIGHT_UPPER


def init_gripper(robot_data):
    """从 load_robot() 同步索引，并使能关节力/力矩传感器。""" 
    global LEFT_JOINT, RIGHT_JOINT, LEFT_LINK, RIGHT_LINK
    LEFT_JOINT = robot_data["left_joint"]
    RIGHT_JOINT = robot_data["right_joint"]
    LEFT_LINK = robot_data["left_link"]
    RIGHT_LINK = robot_data["right_link"]

    robot = robot_data["robot"]
    p.enableJointForceTorqueSensor(robot, LEFT_JOINT, enableSensor=1)
    p.enableJointForceTorqueSensor(robot, RIGHT_JOINT, enableSensor=1)

    print(f"夹爪关节索引已更新: left={LEFT_JOINT}, right={RIGHT_JOINT}")
    print(f"夹爪link索引已更新: left={LEFT_LINK}, right={RIGHT_LINK}")
    print("关节力传感器已使能")
    print(f"开合模式: 指心保持 (STROKE_DIFF={STROKE_DIFF:.4f}m)")


def width_to_joint_positions(width, keep_center=True):
    """
    将开口指令 width 映射为 (left_pos, right_pos)。

    keep_center=True（默认，贴近真实平行爪）:
        从全开等量回缩：d = RIGHT_UPPER - width
        left  = LEFT_UPPER - d
        right = RIGHT_UPPER - d
        从而 right - left = STROKE_DIFF 恒定，指心不漂移。
        width 有效对称区间约为 [CLOSE_CENTER, OPEN]。

    keep_center=False（旧比例模式，仅调试用）:
        left = width * LEFT_RATIO, right = width
    """
    width = float(np.clip(width, CLOSE, OPEN))
    if keep_center:
        d = float(np.clip(RIGHT_UPPER - width, 0.0, LEFT_UPPER))
        left_pos = LEFT_UPPER - d
        right_pos = RIGHT_UPPER - d
    else:
        left_pos = float(np.clip(width * LEFT_RATIO, CLOSE, LEFT_UPPER))
        right_pos = width
    return left_pos, right_pos


def set_gripper_positions(robot, left_pos, right_pos, force=500):
    """直接设置左右指关节目标（力均衡等独立控制时使用）。"""
    left_pos = float(np.clip(left_pos, CLOSE, LEFT_UPPER))
    right_pos = float(np.clip(right_pos, CLOSE, RIGHT_UPPER))
    p.setJointMotorControl2(
        robot, LEFT_JOINT, p.POSITION_CONTROL,
        targetPosition=left_pos, force=force
    )
    p.setJointMotorControl2(
        robot, RIGHT_JOINT, p.POSITION_CONTROL,
        targetPosition=right_pos, force=force
    )
    return left_pos, right_pos


def set_gripper(robot, width, force=500, keep_center=True):
    """
    控制夹爪开口。

    参数:
        robot : PyBullet body id
        width : 开口指令 (m)，全开=OPEN≈0.0715；对称闭合建议 >= CLOSE_CENTER
        force : 电机最大力 (N)
        keep_center : True=指心保持（默认）；False=旧比例映射
    """
    left_pos, right_pos = width_to_joint_positions(width, keep_center=keep_center)
    return set_gripper_positions(robot, left_pos, right_pos, force=force)


def balance_gripper_forces(robot, left_pos, right_pos, left_force, right_force,
                           close_step=0.00015, balance_gain=0.00012,
                           target_each=None, force=200):
    """
    接触后的双指力均衡 + 可选继续对称收紧。

    现实直觉：哪边正压力更大，哪边略松开；哪边更小，哪边略收紧。
    同时若指定 target_each，两指合力不足时再对称闭合。

    返回: (new_left_pos, new_right_pos)
    """
    imbalance = float(left_force) - float(right_force)  # >0 左指力更大
    # 左力大 → 左指略打开(增大关节位移)、右指略闭合(减小)
    left_pos = left_pos + balance_gain * imbalance
    right_pos = right_pos - balance_gain * imbalance

    if target_each is not None:
        avg = 0.5 * (float(left_force) + float(right_force))
        if avg < target_each:
            left_pos -= close_step
            right_pos -= close_step

    return set_gripper_positions(robot, left_pos, right_pos, force=force)


def get_gripper_contact_forces(robot, left=None, right=None):
    """夹爪指端接触点法向力之和（虚拟触觉）。"""
    if left is None:
        left = LEFT_LINK
    if right is None:
        right = RIGHT_LINK

    left_contacts = p.getContactPoints(bodyA=robot, linkIndexA=left)
    right_contacts = p.getContactPoints(bodyA=robot, linkIndexA=right)

    left_force = sum(pt[9] for pt in left_contacts)
    right_force = sum(pt[9] for pt in right_contacts)

    return {
        "left": left_force,
        "right": right_force,
        "total": left_force + right_force,
        "left_contacts": left_contacts,
        "right_contacts": right_contacts
    }


def get_gripper_joint_reaction_forces(robot, left=None, right=None):
    """夹爪关节反作用力（需先 enableJointForceTorqueSensor）。"""
    if left is None:
        left = LEFT_JOINT
    if right is None:
        right = RIGHT_JOINT

    left_state = p.getJointState(robot, left)
    right_state = p.getJointState(robot, right)
    left_force = left_state[2][:3]
    right_force = right_state[2][:3]
    left_mag = float(np.linalg.norm(left_force))
    right_mag = float(np.linalg.norm(right_force))

    return {
        "left_force_vector": left_force,
        "right_force_vector": right_force,
        "left_magnitude": left_mag,
        "right_magnitude": right_mag,
        "total_magnitude": left_mag + right_mag
    }


def get_gripper_force_readings(robot, left_link=None, right_link=None,
                               left_joint=None, right_joint=None):
    """综合返回接触力与关节反力。"""
    return {
        "contact": get_gripper_contact_forces(
            robot, left=left_link, right=right_link
        ),
        "reaction": get_gripper_joint_reaction_forces(
            robot, left=left_joint, right=right_joint
        ),
    }


def open_gripper(robot):
    """完全打开夹爪（指心保持轨迹）。"""
    print("打开夹爪...")
    # 从当前对称闭合下限平滑开到全开
    start_w = max(CLOSE_CENTER, p.getJointState(robot, RIGHT_JOINT)[0])
    for w in np.linspace(start_w, OPEN, 80):
        set_gripper(robot, w, keep_center=True)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
    print("夹爪已打开")


def close_gripper(robot, force_threshold=8.0, step=0.0003):
    """
    指心保持闭合；双指均超过阈值后停止。
    """
    print("力控闭合夹爪（指心保持）...")
    current_width = OPEN
    contact_count = 0
    iter_count = 0

    while current_width > CLOSE_CENTER:
        current_width -= step
        set_gripper(robot, current_width, force=300, keep_center=True)
        for _ in range(3):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

        forces = get_gripper_contact_forces(robot)
        iter_count += 1

        if iter_count % 20 == 0:
            print(
                f"  宽度:{current_width:.4f}m "
                f"左力:{forces['left']:.1f}N 右力:{forces['right']:.1f}N"
            )

        if forces["left"] > force_threshold and forces["right"] > force_threshold:
            contact_count += 1
            if contact_count >= 3:
                print(
                    f"夹持稳定，左:{forces['left']:.1f}N "
                    f"右:{forces['right']:.1f}N"
                )
                break
        else:
            contact_count = 0

        if forces["total"] > 80:
            break

    print("夹爪闭合完成")
