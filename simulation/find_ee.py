import pybullet as p

p.connect(p.DIRECT)

# 使用原始字符串（r"..."）避免反斜杠转义问题
urdf_path = r"D:\Code\reBot-DevArm\simulation\urdf\reBot-DevArm_fixend_description\urdf\reBot-DevArm_fixend.urdf"
print("正在加载:", urdf_path)

robot = p.loadURDF(urdf_path, useFixedBase=True)

print("关节索引 | 关节名 | 子链接名")
print("--------------------------------")
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    link_name = info[12].decode('utf-8')
    print(f"  {i:2d}    | {joint_name:12s} | {link_name}")

# 寻找包含 'end' 的链接（末端）
print("\n可能的末端链接索引（名称包含 'end'）：")
for i in range(p.getNumJoints(robot)):
    link_name = p.getJointInfo(robot, i)[12].decode('utf-8')
    if 'end' in link_name.lower():
        print(f"  索引 {i} : {link_name}")