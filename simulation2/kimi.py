import pybullet as p
import pybullet_data
import time
import numpy as np
import csv
import os
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

from utils.robot import load_robot
from utils.gripper import init_gripper, LEFT_JOINT, RIGHT_JOINT, LEFT_LINK, RIGHT_LINK, \
    set_gripper, open_gripper, close_gripper, get_gripper_force_readings

def load_test_cube(position=[0.30, 0.0, 0.04], mass=0.15):
    # 加载测试Cube，设置摩擦系数和颜色
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    cube = p.loadURDF("cube_small.urdf", basePosition=position)

    p.changeDynamics(cube, -1, mass=mass, lateralFriction=200.0, rollingFriction=1.0, restitution=0.0)
    p.changeVisualShape(cube, -1, rgbaColor=[0.2, 0.6, 0.2, 1])
    print(f"测试Cube加载完成，位置: {position}")
    return cube

def check_cube_contact(robot, cube, left_link=LEFT_LINK, right_link=RIGHT_LINK):
    #   检查夹爪与Cube是否接触，并返回接触力
    left_contacts = p.getContactPoints(bodyA=robot, linkIndexA=left_link, bodyB=cube)
    right_contacts = p.getContactPoints(bodyA=robot, linkIndexA=right_link, bodyB=cube)
    left_force = sum(pt[9] for pt in left_contacts)
    right_force = sum(pt[9] for pt in right_contacts)
    has_contact = len(left_contacts) > 0 or len(right_contacts) > 0
    return has_contact, left_force, right_force

def move_to_position(robot, arm_joints, target_gripper_end_pos, gripper_width, steps=200):
    GRIPPER_END_LINK = 6
    start_angles = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    target_orn = p.getQuaternionFromEuler([0, 0, 0])
    
    left_joint_pos = gripper_width * 0.7
    right_joint_pos = gripper_width
    
    result = p.calculateInverseKinematics(
        robot, GRIPPER_END_LINK, target_gripper_end_pos,
        targetOrientation=target_orn,
        currentPositions=start_angles.tolist() + [left_joint_pos, right_joint_pos],
        maxNumIterations=5000, solver=p.IK_DLS
    )
    target_angles = np.array(result[:6])
    
    for step_idx, t in enumerate(np.linspace(0, 1, steps)):
        s = t * t * (3 - 2 * t)
        current_angles = start_angles + s * (target_angles - start_angles)
        for j, angle in zip(arm_joints, current_angles):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=angle, force=2000)
        set_gripper(robot, gripper_width, force=500)
        p.stepSimulation()
        time.sleep(1/60)
    
    return p.getLinkState(robot, GRIPPER_END_LINK)[0]

def calibrate_gripper_offset(robot, arm_joints, cube_pos):
    GRIPPER_END_LINK = 6
    init_width = 0.035
    
    print("\n=== 校准夹爪偏移 ===")
    open_gripper(robot)
    
    calib_pos = np.array([cube_pos[0], cube_pos[1], cube_pos[2] + 0.2])
    if calib_pos[2] > 0.4:
        calib_pos[2] = 0.4
    
    move_to_position(robot, arm_joints, calib_pos, init_width, steps=200)
    
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1/60)
    
    gripper_end_zero = p.getLinkState(robot, GRIPPER_END_LINK)[0]
    left_zero = p.getLinkState(robot, LEFT_LINK)[0]
    right_zero = p.getLinkState(robot, RIGHT_LINK)[0]
    
    left_offset = np.array([left_zero[0]-gripper_end_zero[0], 
                            left_zero[1]-gripper_end_zero[1], 
                            left_zero[2]-gripper_end_zero[2]])
    right_offset = np.array([right_zero[0]-gripper_end_zero[0], 
                             right_zero[1]-gripper_end_zero[1], 
                             right_zero[2]-gripper_end_zero[2]])
    
    print(f"校准位置: {calib_pos}")
    print(f"夹爪末端位置: {gripper_end_zero}")
    print(f"左指位置: {left_zero}, 偏移: {left_offset}")
    print(f"右指位置: {right_zero}, 偏移: {right_offset}")
    
    return left_offset, right_offset

def check_workspace(cube_pos):
    """检查cube位置是否在机械臂工作空间内"""
    x_ok = 0.2 <= cube_pos[0] <= 0.55
    y_ok = -0.35 <= cube_pos[1] <= 0.35
    z_ok = 0.03 <= cube_pos[2] <= 0.3
    
    if not x_ok:
        print(f"❌ X坐标 {cube_pos[0]:.2f} 超出工作空间 [0.2, 0.55]")
    if not y_ok:
        print(f"❌ Y坐标 {cube_pos[1]:.2f} 超出工作空间 [-0.35, 0.35]")
    if not z_ok:
        print(f"❌ Z坐标 {cube_pos[2]:.2f} 超出工作空间 [0.03, 0.3]")
    
    return x_ok and y_ok and z_ok

def main():
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    robot_data = load_robot()
    robot = robot_data["robot"]
    arm_joints = robot_data["arm_joints"]
    init_gripper(robot_data)

    p.enableJointForceTorqueSensor(robot, LEFT_JOINT, enableSensor=1)
    p.enableJointForceTorqueSensor(robot, RIGHT_JOINT, enableSensor=1)

    GRIPPER_END_LINK = 6
    
    cube_pos = np.array([0.40, 0., 0.04])
    print(f"📌 当前Cube位置: {cube_pos}")
    print(f"💡 提示: 你可以修改第125行的 cube_pos 来改变Cube位置")
    print(f"💡 工作空间范围: X=[0.2,0.55], Y=[-0.35,0.35], Z=[0.03,0.3]")
    
    if not check_workspace(cube_pos):
        print("\n❌ Cube位置超出工作空间范围，请调整位置后重试")
        p.disconnect()
        return
    
    cube = load_test_cube(position=cube_pos, mass=0.15)

    print("\n等待场景稳定...")
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    left_offset, right_offset = calibrate_gripper_offset(robot, arm_joints, cube_pos)

    cube_size = 0.05
    init_width = 0.035
    
    target_left = cube_pos + np.array([0, -cube_size/2, 0])
    target_right = cube_pos + np.array([0, cube_size/2, 0])
    target_gripper_end = (target_left - left_offset + target_right - right_offset) / 2

    print("\n=== 移动到预抓取位置 ===")
    move_to_position(robot, arm_joints, target_gripper_end + np.array([0, 0, 0.08]), init_width, steps=200)

    print("\n=== 下降到抓取位置 ===")
    target_gripper_end_grasp = target_gripper_end + np.array([0, 0, -0.03])
    move_to_position(robot, arm_joints, target_gripper_end_grasp, init_width, steps=150)

    print("\n=== 检测夹爪位置 ===")
    left_link_pos = p.getLinkState(robot, LEFT_LINK)[0]
    right_link_pos = p.getLinkState(robot, RIGHT_LINK)[0]
    gripper_center_x = (left_link_pos[0] + right_link_pos[0]) / 2
    gripper_center_z = (left_link_pos[2] + right_link_pos[2]) / 2
    print(f"夹爪左指位置: {left_link_pos}")
    print(f"夹爪右指位置: {right_link_pos}")
    print(f"夹爪中心: ({gripper_center_x:.4f}, {gripper_center_z:.4f})")
    
    cube_actual, _ = p.getBasePositionAndOrientation(cube)
    print(f"Cube实际位置: {cube_actual}")
    
    cube_z_center = cube_actual[2] + 0.025
    x_error = abs(cube_actual[0] - gripper_center_x)
    z_error = abs(cube_z_center - gripper_center_z)
    print(f"位置误差: X={x_error:.4f}m, Z={z_error:.4f}m")

    print("\n=== 闭合夹爪（力控模式）===")
    force_records = []
    start_time = time.time()
    contact_detected = False
    
    current_width = init_width
    step = 0.0003
    
    while current_width > 0.0:
        current_width -= step
        if current_width < 0.0:
            current_width = 0.0
        
        set_gripper(robot, current_width, force=500)
        
        for _ in range(3):
            p.stepSimulation()
            time.sleep(1/240)
        
        elapsed_time = time.time() - start_time
        force_data = get_gripper_force_readings(robot)
        
        has_cube_contact, cube_left_force, cube_right_force = check_cube_contact(robot, cube)
        
        if has_cube_contact and not contact_detected:
            contact_detected = True
            print(f"✅ [时间={elapsed_time:.2f}s] 检测到与Cube接触！左力={cube_left_force:.2f}N, 右力={cube_right_force:.2f}N")
        
        force_records.append({
            "step": len(force_records),
            "time": elapsed_time,
            "joint_pos": current_width,
            "contact_left": force_data["contact"]["left"],
            "contact_right": force_data["contact"]["right"],
            "contact_total": force_data["contact"]["total"],
            "reaction_left": force_data["reaction"]["left_magnitude"],
            "reaction_right": force_data["reaction"]["right_magnitude"],
            "reaction_total": force_data["reaction"]["total_magnitude"],
            "has_cube_contact": has_cube_contact
        })
        
        if len(force_records) % 10 == 0:
            cube_status = "接触中" if has_cube_contact else "未接触"
            print(f"[时间={elapsed_time:.2f}s] 宽度={current_width:.4f}m | 接触力: {force_data['contact']['total']:.2f}N | Cube状态: {cube_status}")
        
        if force_data["contact"]["total"] > 80:
            print(f"⚠️ [时间={elapsed_time:.2f}s] 接触力超过80N，停止闭合")
            break
    
    if not contact_detected:
        print("\n❌ 错误：夹爪闭合完毕，但未检测到与Cube的任何接触！")
        print("可能原因：")
        print("1. 夹爪位置与Cube不重合")
        print("2. 夹爪初始张开角度太小")
        print("3. Cube已被碰离原位")
        p.disconnect()
        raise RuntimeError("夹爪未接触到Cube，抓取失败！")
    
    print(f"\n✅ 夹爪已闭合，检测到与Cube接触")

    print("\n=== 提升Cube ===")
    start_angles = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    target_gripper_end_up = target_gripper_end_grasp + np.array([0, 0, 0.14])
    
    left_joint_pos = current_width * 0.7
    right_joint_pos = current_width
    
    result = p.calculateInverseKinematics(
        robot, GRIPPER_END_LINK, target_gripper_end_up,
        targetOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        currentPositions=start_angles.tolist() + [left_joint_pos, right_joint_pos],
        maxNumIterations=5000, solver=p.IK_DLS
    )
    target_angles = np.array(result[:6])

    for step_idx, t in enumerate(np.linspace(0, 1, 300)):
        s = t * t * (3 - 2 * t)
        current = start_angles + s * (target_angles - start_angles)
        for j, angle in zip(arm_joints, current):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=angle, force=2000)
        set_gripper(robot, current_width, force=500)
        p.stepSimulation()
        time.sleep(1/60)
        
        if step_idx % 50 == 0:
            elapsed_time = time.time() - start_time
            print(f"[提升中 时间={elapsed_time:.2f}s]")

    cube_current, _ = p.getBasePositionAndOrientation(cube)
    print(f'Cube位置: {cube_current}')
    
    cube_initial_z = cube_pos[2]
    cube_lifted = cube_current[2] - cube_initial_z
    
    print(f'Cube提升高度: {cube_lifted:.4f}m')
    
    has_contact_after_lift, _, _ = check_cube_contact(robot, cube)
    
    if cube_lifted > 0.05 and has_contact_after_lift:
        print('✅ 抓取成功！')
    else:
        if cube_lifted <= 0.05:
            print('❌ 错误：Cube未被提升！可能是夹爪夹紧力不足或位置不对')
        if not has_contact_after_lift:
            print('❌ 错误：提升过程中Cube与夹爪失去接触！')
        print('抓取失败！')
        p.disconnect()
        raise RuntimeError(f"抓取失败: Cube提升高度={cube_lifted:.4f}m, 接触状态={has_contact_after_lift}")

    print("\n=== 保存力传感器数据 ===")
    output_dir = "force_data"
    os.makedirs(output_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"gripper_force_{timestamp}.csv")
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["step", "time(s)", "joint_pos(m)", 
                         "contact_left(N)", "contact_right(N)", "contact_total(N)",
                         "reaction_left(N)", "reaction_right(N)", "reaction_total(N)"])
        for record in force_records:
            writer.writerow([
                record["step"],
                f"{record['time']:.4f}",
                f"{record['joint_pos']:.6f}",
                f"{record['contact_left']:.4f}",
                f"{record['contact_right']:.4f}",
                f"{record['contact_total']:.4f}",
                f"{record['reaction_left']:.4f}",
                f"{record['reaction_right']:.4f}",
                f"{record['reaction_total']:.4f}"
            ])
    
    print(f"力传感器数据已保存到: {filename}")

    print("\n=== 绘制力传感器时序曲线 ===")
    times = [r["time"] for r in force_records]
    contact_left = [r["contact_left"] for r in force_records]
    contact_right = [r["contact_right"] for r in force_records]
    contact_total = [r["contact_total"] for r in force_records]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(times, contact_left, label='左指接触力', color='blue', alpha=0.8)
    ax1.plot(times, contact_right, label='右指接触力', color='red', alpha=0.8)
    ax1.plot(times, contact_total, label='总接触力', color='green', linewidth=2)
    ax1.set_ylabel('接触力 (N)')
    ax1.set_title('夹爪接触力时序曲线')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    joint_pos = [r["joint_pos"] for r in force_records]
    ax2.plot(times, joint_pos, label='夹爪宽度', color='purple')
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('夹爪宽度 (m)')
    ax2.set_title('夹爪宽度变化')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    plot_filename = filename.replace('.csv', '.png')
    plt.savefig(plot_filename, dpi=150)
    print(f"力传感器曲线已保存到: {plot_filename}")
    
    plt.show()

    p.disconnect()

if __name__ == "__main__":
    main()