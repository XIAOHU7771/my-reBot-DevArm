"""
utils/robot.py

reBot-DevArm机器人加载模块

功能：
1. 加载URDF模型
2. 按名称识别机械臂6个旋转关节 (joint1~joint6)
3. 获取末端执行器(gripper_end)
4. 获取夹爪左右控制关节
5. 获取左右夹爪link，用于力/触觉传感

适用于：
B2 基于物理仿真环境的自适应力控抓取
"""

import os
import pybullet as p


# ==================================================
# URDF路径（相对仓库根目录）
# ==================================================

ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        ".."
    )
)

URDF_PATH = os.path.join(
    ROOT_DIR,
    "urdf",
    "00-arm-rs_asm-v3",
    "urdf",
    "00-arm-rs_asm-v3.urdf"
)

# 臂部关节按 URDF 名称固定顺序，避免「凡是 revolute 都收进来」的隐患
ARM_JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6"
]


# ==================================================
# 工具函数
# ==================================================

def _get_joint_index(robot, joint_name):
    """
    根据joint名字获取PyBullet joint index
    """
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode("utf-8")
        if name == joint_name:
            return i

    raise RuntimeError(f"未找到joint: {joint_name}")


def _get_link_index(robot, link_name):
    """
    根据link名字获取PyBullet link index
    （PyBullet 中 child link 索引 = 创建该 link 的 joint 索引）
    """
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        child_link = info[12].decode("utf-8")
        if child_link == link_name:
            return i

    raise RuntimeError(f"未找到link: {link_name}")


# ==================================================
# 加载机器人
# ==================================================

def load_robot():
    """
    加载 reBot-DevArm URDF（固定基座），并解析关键索引。

    返回字典键：
      robot, arm_joints, ee_link,
      left_joint, right_joint, left_link, right_link
    """
    if not os.path.isfile(URDF_PATH):
        raise FileNotFoundError(f"URDF 不存在: {URDF_PATH}")

    robot = p.loadURDF(
        URDF_PATH,
        useFixedBase=True,
        # 【修改说明】使用 URDF 自带惯性，避免质量/惯量被默认值覆盖
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    # -----------------------------
    # 【修改说明】机械臂旋转关节
    # 旧逻辑：收集所有 JOINT_REVOLUTE。当前 URDF 只有 6 个，结果正确，
    # 但若日后增加其他转动关节会静默混入。改为按 joint1~joint6 名称查找。
    # -----------------------------
    arm_joints = [_get_joint_index(robot, name) for name in ARM_JOINT_NAMES]

    # -----------------------------
    # IK末端：gripper_end（fixed 连接到 link6）
    # 注意：索引为 6，不是 link6(5)。IK 时勿再叠加 link6→gripper_end 偏移。
    # -----------------------------
    ee_link = _get_link_index(robot, "gripper_end")

    # -----------------------------
    # 夹爪控制joint（prismatic）
    # -----------------------------
    left_joint = _get_joint_index(robot, "joint_left")
    right_joint = _get_joint_index(robot, "joint_right")

    # -----------------------------
    # 夹爪碰撞link（用于力传感器）
    # -----------------------------
    left_link = _get_link_index(robot, "gripper_left")
    right_link = _get_link_index(robot, "gripper_right")

    print("======================")
    print("机器人加载完成")
    print("URDF:", URDF_PATH)
    print("Arm joints:", arm_joints)
    print("EE link (gripper_end):", ee_link)
    print("Left joint:", left_joint)
    print("Right joint:", right_joint)
    print("Left link:", left_link)
    print("Right link:", right_link)
    print("======================")

    return {
        "robot": robot,
        "arm_joints": arm_joints,
        "ee_link": ee_link,
        "left_joint": left_joint,
        "right_joint": right_joint,
        "left_link": left_link,
        "right_link": right_link
    }
