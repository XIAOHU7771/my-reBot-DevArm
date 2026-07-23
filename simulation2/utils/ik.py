"""
utils/ik.py — reBot-DevArm 逆运动学与 TCP 控制
================================================
功能：
  1. init_ik：与 load_robot 同步夹爪关节索引
  2. solve_ik：PyBullet DLS 初值 + 姿态约束雅可比精调
  3. move_tcp：关节空间线性插值移动 TCP（简易接口）
  4. get_current_tcp：读取当前 TCP 世界坐标

兼容 PyBullet 3.2.7；quiet=True 可抑制频繁打印。
TCP_OFFSET 默认为 0（ee_link 已是 gripper_end）。
"""

import pybullet as p
import numpy as np
import time

# ============================================================
# 【修改说明】TCP_OFFSET
# 旧值 [0.16186, 0, 0] 是「link6 → gripper_end」的近似偏移。
# 但 robot.py 的 ee_link 已是 gripper_end（索引 6），再叠加该偏移
# 会把 TCP 沿夹爪坐标系 X 多偏约 16cm，导致 IK 目标错误。
# URDF: j_gripper_end 相对 link6 为 xyz="0 0 0.16621"；
# 以 gripper_end 为末端时，TCP 即该 link 原点，偏移应为 0。
# ============================================================
TCP_OFFSET = np.array([0.0, 0.0, 0.0])

# 臂部 6 关节的限位（弧度）——与 URDF joint1~joint6 一致
LOWER_LIMITS = [-2.8, 0.0, 0.0, -1.57, -1.57, -3.14]
UPPER_LIMITS = [2.8, 3.14, 3.14, 1.57, 1.57, 3.14]

# 夹爪关节索引（默认与 URDF 加载顺序一致；可用 init_ik 同步）
LEFT_JOINT = 7
RIGHT_JOINT = 8
ARM_JOINT_COUNT = 6

# 夹爪关节行程（URDF prismatic limit）
LEFT_JOINT_UPPER = 0.05
RIGHT_JOINT_UPPER = 0.0715

# 数值微分步长
DELTA = 1e-4

_TARGET_EULER = [-1.5708, 0, 0]


def init_ik(robot_data):
    """
    【修改说明】新增：与 robot.load_robot() / gripper.init_gripper() 同步索引，
    避免 ik 模块写死 7/8 而 gripper 已动态更新时不一致。
    """
    global LEFT_JOINT, RIGHT_JOINT
    LEFT_JOINT = robot_data["left_joint"]
    RIGHT_JOINT = robot_data["right_joint"]
    print(f"IK 夹爪关节索引已更新: left={LEFT_JOINT}, right={RIGHT_JOINT}")


def _get_tcp_position(robot, arm_joints, ee_link):
    """根据当前仿真状态返回 TCP 世界坐标"""
    state = p.getLinkState(robot, ee_link)
    ee_pos = np.array(state[4])
    ee_orn = state[5]
    offset_world = p.rotateVector(ee_orn, TCP_OFFSET.tolist())
    return ee_pos + np.array(offset_world)


def _set_arm_joints(robot, arm_joints, angles):
    """立即设置臂关节角度（用于迭代优化）"""
    for j, ang in zip(arm_joints, angles):
        p.resetJointState(robot, j, ang)


def _compute_jacobian(robot, arm_joints, ee_link, current_angles):
    """有限差分计算 6×6 雅可比矩阵（位置+姿态）"""
    orig_angles = [p.getJointState(robot, j)[0] for j in arm_joints]
    _set_arm_joints(robot, arm_joints, current_angles)

    base_state = p.getLinkState(robot, ee_link)
    base_pos = np.array(base_state[4])
    base_orn = base_state[5]
    # base_tcp 仅用于与差分参考一致；TCP_OFFSET=0 时等于 base_pos
    _ = base_pos + np.array(p.rotateVector(base_orn, TCP_OFFSET.tolist()))

    J = np.zeros((6, len(arm_joints)))
    for i in range(len(arm_joints)):
        # 正向扰动
        angles_plus = current_angles.copy()
        angles_plus[i] += DELTA
        angles_plus[i] = np.clip(angles_plus[i], LOWER_LIMITS[i], UPPER_LIMITS[i])
        _set_arm_joints(robot, arm_joints, angles_plus)
        state_plus = p.getLinkState(robot, ee_link)
        pos_plus = np.array(state_plus[4])
        orn_plus = state_plus[5]
        tcp_plus = pos_plus + np.array(p.rotateVector(orn_plus, TCP_OFFSET.tolist()))

        # 反向扰动
        angles_minus = current_angles.copy()
        angles_minus[i] -= DELTA
        angles_minus[i] = np.clip(angles_minus[i], LOWER_LIMITS[i], UPPER_LIMITS[i])
        _set_arm_joints(robot, arm_joints, angles_minus)
        state_minus = p.getLinkState(robot, ee_link)
        pos_minus = np.array(state_minus[4])
        orn_minus = state_minus[5]
        tcp_minus = pos_minus + np.array(p.rotateVector(orn_minus, TCP_OFFSET.tolist()))

        # 位置雅可比
        J[:3, i] = (tcp_plus - tcp_minus) / (2 * DELTA)
        orn_diff = p.getDifferenceQuaternion(orn_minus, orn_plus)
        axis_d, angle_d = p.getAxisAngleFromQuaternion(orn_diff)
        J[3:, i] = np.array(axis_d) * angle_d / (2 * DELTA)

    # 恢复原始状态
    _set_arm_joints(robot, arm_joints, orig_angles)
    return J


def refine_joints(robot, arm_joints, ee_link, target_tcp, target_orn, initial_angles,
                  max_iter=120, tolerance=1e-4, lr=0.6, quiet=False):
    """
    带姿态约束的高斯-牛顿迭代优化（自适应阻尼）
    quiet=True 时不打印迭代信息。
    """
    angles = np.array(initial_angles, dtype=float)
    _set_arm_joints(robot, arm_joints, angles)

    it = 0
    for it in range(max_iter):
        # 当前位姿
        state = p.getLinkState(robot, ee_link)
        current_pos = np.array(state[4])
        current_orn = state[5]

        # TCP位置误差
        offset_world = p.rotateVector(current_orn, TCP_OFFSET.tolist())
        current_tcp = current_pos + np.array(offset_world)
        pos_error = target_tcp - current_tcp
        pos_error_norm = np.linalg.norm(pos_error)

        # 姿态误差（轴角向量）
        orn_error = p.getDifferenceQuaternion(current_orn, target_orn)
        axis, angle = p.getAxisAngleFromQuaternion(orn_error)
        orn_error_vec = np.array(axis) * angle

        # 总误差6维
        error = np.concatenate([pos_error, orn_error_vec])
        if pos_error_norm < tolerance and angle < 0.01:
            break

        # 计算雅可比
        J = _compute_jacobian(robot, arm_joints, ee_link, angles)

        # 自适应阻尼：误差大时防震荡，误差小时提精度
        lam = 1e-2 if pos_error_norm > 0.01 else 1e-3
        J_T = J.T
        pseudo_inv = J_T @ np.linalg.inv(J @ J_T + lam * np.eye(6))

        # 关节角更新 + 限位
        delta_angles = pseudo_inv @ error
        angles_new = angles + lr * delta_angles
        for i in range(len(arm_joints)):
            angles_new[i] = np.clip(angles_new[i], LOWER_LIMITS[i], UPPER_LIMITS[i])

        angles = angles_new
        _set_arm_joints(robot, arm_joints, angles)

    final_tcp = _get_tcp_position(robot, arm_joints, ee_link)
    final_error = np.linalg.norm(target_tcp - final_tcp)
    if not quiet:
        print(f"[数值优化] 迭代 {it+1} 次, 最终位置误差: {final_error*1000:.2f} mm")
    return angles, final_error


def solve_ik(robot, arm_joints, ee_link, tcp_target, current_arm_angles,
             target_orn=None, quiet=False):
    """
    两级IK求解：PyBullet带姿态初值 + 数值精调
    quiet=True 时抑制调试打印（平滑运动路径会频繁调用）。
    """
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler(_TARGET_EULER)
    # TCP目标 → EE目标 转换（TCP_OFFSET=0 时二者相同）
    offset_world = p.rotateVector(target_orn, TCP_OFFSET.tolist())
    ee_target = np.array(tcp_target) - np.array(offset_world)

    # 锁定夹爪关节（保持当前开合，不参与臂部 IK）
    left_cur = p.getJointState(robot, LEFT_JOINT)[0]
    right_cur = p.getJointState(robot, RIGHT_JOINT)[0]
    all_current = list(current_arm_angles) + [left_cur, right_cur]

    # ============================================================
    # 【修改说明】夹爪限位
    # 旧写法 lower=upper=当前值 → jointRanges=0，DLS 易数值不稳定。
    # 改为使用 URDF 真实行程，靠 restPoses/currentPositions 保持开合。
    # ============================================================
    full_lower = LOWER_LIMITS + [0.0, 0.0]
    full_upper = UPPER_LIMITS + [LEFT_JOINT_UPPER, RIGHT_JOINT_UPPER]

    if not quiet:
        print("================")
        print(f"TCP 目标: {tcp_target}")
        print(f"EE  目标: {ee_target}")

    # PyBullet 带姿态IK求初值
    # 注意：返回长度为可动关节数(8)，不含 fixed 的 j_gripper_end
    result = p.calculateInverseKinematics(
        robot,
        ee_link,
        ee_target,
        targetOrientation=target_orn,
        lowerLimits=full_lower,
        upperLimits=full_upper,
        jointRanges=[u - l for u, l in zip(full_upper, full_lower)],
        restPoses=all_current,
        currentPositions=all_current,
        maxNumIterations=5000,
        solver=p.IK_DLS
    )
    # arm_joints 为 [0..5]，与 IK 返回前 6 项一一对应
    initial_angles = np.array([result[j] for j in arm_joints])

    _set_arm_joints(robot, arm_joints, initial_angles)
    initial_tcp = _get_tcp_position(robot, arm_joints, ee_link)
    initial_error = np.linalg.norm(np.array(tcp_target) - initial_tcp)

    if not quiet:
        print(f"初始IK误差: {initial_error*1000:.2f} mm")

    if initial_error < 0.005:
        return initial_angles

    optimized_angles, opt_error = refine_joints(
        robot, arm_joints, ee_link, np.array(tcp_target), target_orn, initial_angles,
        max_iter=120, tolerance=1e-4, lr=0.8, quiet=quiet,
    )

    _set_arm_joints(robot, arm_joints, optimized_angles)
    opt_tcp = _get_tcp_position(robot, arm_joints, ee_link)
    actual_opt_error = np.linalg.norm(np.array(tcp_target) - opt_tcp)

    if actual_opt_error > max(initial_error * 2, 0.01):
        if not quiet:
            print(
                f"[IK回退] 数值优化恶化 "
                f"(初始{initial_error*1000:.2f}mm → 优化后{actual_opt_error*1000:.2f}mm)，"
                f"使用初始IK解"
            )
        return initial_angles

    return optimized_angles


def move_tcp(robot, arm_joints, ee_link, tcp_target, steps=240):
    """
    控制 TCP 平滑移动到目标位置，返回最终实际 TCP
    """
    start_angles = np.array([p.getJointState(robot, j)[0] for j in arm_joints])
    target_angles = solve_ik(robot, arm_joints, ee_link, tcp_target, start_angles)

    # 【修改说明】直接使用本模块已同步的 LEFT_JOINT/RIGHT_JOINT，
    # 不再从 gripper 二次 import（避免未 init_gripper 时索引不一致）
    left_cur = p.getJointState(robot, LEFT_JOINT)[0]
    right_cur = p.getJointState(robot, RIGHT_JOINT)[0]

    for t in np.linspace(0, 1, steps):
        current = start_angles + t * (target_angles - start_angles)
        _set_arm_joints(robot, arm_joints, current)

        p.resetJointState(robot, LEFT_JOINT, left_cur)
        p.resetJointState(robot, RIGHT_JOINT, right_cur)

        p.setJointMotorControl2(
            robot, LEFT_JOINT, p.POSITION_CONTROL,
            targetPosition=left_cur, force=500
        )
        p.setJointMotorControl2(
            robot, RIGHT_JOINT, p.POSITION_CONTROL,
            targetPosition=right_cur, force=500
        )

        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # 【修改说明】与仿真常用 240Hz 对齐（原 1000Hz 过快）

    _set_arm_joints(robot, arm_joints, target_angles)

    p.resetJointState(robot, LEFT_JOINT, left_cur)
    p.resetJointState(robot, RIGHT_JOINT, right_cur)

    p.setJointMotorControlArray(
        robot,
        arm_joints,
        p.POSITION_CONTROL,
        targetPositions=target_angles,
        forces=[2000] * ARM_JOINT_COUNT
    )
    p.setJointMotorControl2(
        robot, LEFT_JOINT, p.POSITION_CONTROL,
        targetPosition=left_cur, force=500
    )
    p.setJointMotorControl2(
        robot, RIGHT_JOINT, p.POSITION_CONTROL,
        targetPosition=right_cur, force=500
    )

    final_tcp = _get_tcp_position(robot, arm_joints, ee_link)
    error = np.linalg.norm(final_tcp - np.array(tcp_target))
    print("================")
    print(f"最终实际 TCP: {final_tcp}")
    print(f"目标 TCP: {tcp_target}")
    print(f"TCP 误差: {error*1000:.2f} mm")
    return final_tcp


def get_current_tcp(robot, ee_link):
    """返回当前 TCP 在世界坐标系下的位置"""
    state = p.getLinkState(robot, ee_link)
    ee_pos = np.array(state[4])
    ee_orn = state[5]
    offset = p.rotateVector(ee_orn, TCP_OFFSET.tolist())
    return ee_pos + np.array(offset)
