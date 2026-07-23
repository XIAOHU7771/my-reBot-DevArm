import os

import pybullet as p

# 连接到PyBullet模拟器（DIRECT模式，无GUI）
p.connect(p.DIRECT)

# 相对仓库根目录加载 URDF（避免写死本机绝对路径）
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
urdf_path = os.path.join(
    _ROOT,
    "urdf",
    "reBot-DevArm_fixend_description",
    "urdf",
    "reBot-DevArm_fixend.urdf",
)
print("正在加载:", urdf_path)

# 加载URDF模型，设置为固定基座, 并返回机器人ID
robot = p.loadURDF(urdf_path, useFixedBase=True)

print("关节索引 | 关节名 | 子链接名")
print("--------------------------------")
#   遍历所有关节，获取关节信息并打印关节索引、关节名和子链接名
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    link_name = info[12].decode('utf-8')
    print(f"  {i:2d}    | {joint_name:12s} | {link_name}")
    

# 寻找所有关节的子链接名，检查是否包含 'end'，如果包含则打印索引和链接名
print("\n可能的末端链接索引（名称包含 'end'）：")
for i in range(p.getNumJoints(robot)):
# 获取关节信息
    link_name = p.getJointInfo(robot, i)[12].decode('utf-8')
    # 检查链接名是否包含 'end'（不区分大小写）
    if 'end' in link_name.lower():
        # 打印索引和链接名
        print(f"  索引 {i} : {link_name}")
# 打印所有关节的索引、关节名和类型
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(i, info[1].decode(), "类型 =", info[2])




"""         Base（底座）
            │
      joint1（可转）
            │
         link1
            │
      joint2（可转）
            │
         link2
            │
      joint3（可转）
            │
         link3
            │
      joint4（可转）
            │
         link4
            │
      joint5（可转）
            │
         link5
            │
      joint6（可转）
            │
         link6
            │
    end_joint（固定）
            │
        end_link """