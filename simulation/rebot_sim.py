
# 虚拟仿真控制
# 输入目标坐标 (x,y,z)，末端平滑移动到该点

import os
import time

import numpy as np
import pybullet as p

# 连接到 PyBullet 模拟器（GUI模式）,并设置重力
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# 相对仓库根目录加载 URDF（避免写死本机绝对路径）
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
urdf_path = os.path.join(
    _ROOT,
    "urdf",
    "reBot-DevArm_fixend_description",
    "urdf",
    "reBot-DevArm_fixend.urdf",
)
print("加载模型:", urdf_path)
# 加载URDF模型，设置为固定基座, 并返回机器人ID
robotId = p.loadURDF(urdf_path, useFixedBase=True)
print("✅ 模型加载成功")

# 获取所有非固定关节的索引
joint_indices = []
# 遍历所有关节，检查关节类型是否为非固定关节
for i in range(p.getNumJoints(robotId)):
# 获取关节信息并检查关节类型
    if p.getJointInfo(robotId, i)[2] != p.JOINT_FIXED:
        joint_indices.append(i)
print(f"运动关节索引: {joint_indices}")

# 末端链接索引（根据 find_ee.py 结果）
end_effector_index = 6

# 输入目标坐标
while True:
    try:
        user_input = input("请输入目标坐标 (x, y, z)，用逗号分隔，例如 0.15, 0.0, 0.15: ")
        target_pos = [float(x.strip()) for x in user_input.split(",")]
        if len(target_pos) != 3:
            print(" 请输入三个数值，用逗号分隔！")
            continue
        break
    except ValueError:
        print(" 输入错误，请输入数字，用逗号分隔！")

# 获取当前关节角度
current_angles = [p.getJointState(robotId, i)[0] for i in joint_indices]

# 逆运动学求解，IK求解，计算关节角度
joint_poses = p.calculateInverseKinematics(
    robotId,
    end_effector_index,
    target_pos,
    maxNumIterations=500,
    residualThreshold=1e-5
)

# 提取目标关节角度
if len(joint_poses) == p.getNumJoints(robotId):
    target_angles = [joint_poses[i] for i in joint_indices]
else:
    target_angles = joint_poses[:len(joint_indices)]

print(f"🎯 目标位置: {target_pos}")
print(f"目标关节角度: {[round(a, 3) for a in target_angles]}")

# 平滑插值运动
steps = 100
for t in np.linspace(0, 1, steps):
    interp = [current_angles[i] + t * (target_angles[i] - current_angles[i])
              for i in range(len(joint_indices))]
    p.setJointMotorControlArray(robotId, joint_indices, p.POSITION_CONTROL,
                                targetPositions=interp)
    p.stepSimulation()
    time.sleep(0.01)

print("✅ 运动完成！窗口保持打开...")
while True:
    time.sleep(0.1)

