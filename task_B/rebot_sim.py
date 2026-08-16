"""
task_B/rebot_sim.py
============================================================
任务 B 教学示例：虚拟机械臂点到点运动

小白先记住整条流水线（本文件就是最短实现）：
  1. 连接 PyBullet，加载 URDF（得到虚拟臂）
  2. 你输入末端目标坐标 (x, y, z) —— 笛卡尔空间
  3. 逆运动学 IK：把 (x,y,z) 换成各关节目标角 —— 关节空间
  4. 关节角从「当前」平滑插值到「目标」，每步推进仿真
  5. 画面上看到末端慢慢移到你指定的点

更完整的概念讲解见仓库根目录：新手入门讲解.md
"""

import os
import time

import numpy as np
import pybullet as p

# ---------------------------------------------------------------------------
# 1) 打开仿真世界
#    GUI = 带 3D 窗口；重力 -9.8 与真实世界同方向（Z 向上时向下为负）
# ---------------------------------------------------------------------------
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# ---------------------------------------------------------------------------
# 2) 加载 URDF
#    URDF = 机器人“零件说明书”（连杆、关节、限位、外观网格）
#    useFixedBase=True：底座钉在地上，不会整机翻倒
# ---------------------------------------------------------------------------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
urdf_path = os.path.join(
    _ROOT,
    "urdf",
    "reBot-DevArm_fixend_description",
    "urdf",
    "reBot-DevArm_fixend.urdf",
)
print("加载模型:", urdf_path)
robotId = p.loadURDF(urdf_path, useFixedBase=True)
print("模型加载成功")

# ---------------------------------------------------------------------------
# 3) 找出所有“可动关节”
#    PyBullet 里每个关节有类型；FIXED 是焊死的，不参与运动控制
#    joint_indices 后面会传给电机接口和 IK 结果切片
# ---------------------------------------------------------------------------
joint_indices = []
for i in range(p.getNumJoints(robotId)):
    # getJointInfo(...)[2] == 关节类型
    if p.getJointInfo(robotId, i)[2] != p.JOINT_FIXED:
        joint_indices.append(i)
print(f"运动关节索引: {joint_indices}")

# 末端执行器（手）所在 link 的编号。可用 find_ee.py 核对，本模型常见为 6
end_effector_index = 6

# ---------------------------------------------------------------------------
# 4) 人机交互：输入笛卡尔目标点
#    推荐工作空间大约 x:0.05~0.25, y:-0.15~0.15, z:0.05~0.25
#    点太远 → IK 失败或姿态很怪
# ---------------------------------------------------------------------------
while True:
    try:
        user_input = input(
            "请输入目标坐标 (x, y, z)，用逗号分隔，例如 0.15, 0.0, 0.15: "
        )
        target_pos = [float(x.strip()) for x in user_input.split(",")]
        if len(target_pos) != 3:
            print("请输入三个数值，用逗号分隔！")
            continue
        break
    except ValueError:
        print("输入错误，请输入数字，用逗号分隔！")

# ---------------------------------------------------------------------------
# 5) 读当前关节角（插值起点）
#    getJointState 返回 (位置, 速度, 反作用力...), [0] 是角度(rad)或位移(m)
# ---------------------------------------------------------------------------
current_angles = [p.getJointState(robotId, i)[0] for i in joint_indices]

# ---------------------------------------------------------------------------
# 6) 逆运动学 IK
#    输入：机器人、末端 link、想要的手部位置
#    输出：一串关节角建议（长度可能含固定关节，下面再按 joint_indices 取）
#    本质：数值迭代，不是唯一解析公式；可能多解或失败
# ---------------------------------------------------------------------------
joint_poses = p.calculateInverseKinematics(
    robotId,
    end_effector_index,
    target_pos,
    maxNumIterations=500,
    residualThreshold=1e-5,
)

if len(joint_poses) == p.getNumJoints(robotId):
    target_angles = [joint_poses[i] for i in joint_indices]
else:
    # 有的版本返回长度 = 可动关节数，直接切片
    target_angles = joint_poses[: len(joint_indices)]

print(f"目标位置: {target_pos}")
print(f"目标关节角度: {[round(a, 3) for a in target_angles]}")

# ---------------------------------------------------------------------------
# 7) 关节空间平滑插值运动（本任务的“控制实现”）
#    t 从 0→1：中间角 = 当前 + t * (目标 - 当前)
#    每一步：
#      - POSITION_CONTROL：让电机去跟这个中间角（位置伺服）
#      - stepSimulation：物理世界前进一帧（碰撞、积分）
#      - sleep：放慢一点，方便人眼观察（不是控制器本身必须的）
#    steps 越大越平滑，但总时间更长；太小会像“瞬移”
# ---------------------------------------------------------------------------
steps = 100
for t in np.linspace(0, 1, steps):
    interp = [
        current_angles[i] + t * (target_angles[i] - current_angles[i])
        for i in range(len(joint_indices))
    ]
    p.setJointMotorControlArray(
        robotId,
        joint_indices,
        p.POSITION_CONTROL,
        targetPositions=interp,
    )
    p.stepSimulation()
    time.sleep(0.01)

print("运动完成！窗口保持打开（关闭 GUI 或 Ctrl+C 结束）...")
while True:
    time.sleep(0.1)
