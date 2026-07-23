# my-reBot-DevArm

基于 [PyBullet](https://pybullet.org/) 的 [reBot-DevArm](https://github.com/) 六轴机械臂虚拟仿真与力控抓取实现。

**仓库**：[https://github.com/XIAOHU7771/my-reBot-DevArm](https://github.com/XIAOHU7771/my-reBot-DevArm)

[![Python 3.10+](https://img.shields.io/badge/Python-%3E%3D3.10-blue)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey)]()
[![Release](https://img.shields.io/github/v/release/XIAOHU7771/my-reBot-DevArm)](https://github.com/XIAOHU7771/my-reBot-DevArm/releases)

---

## 一、项目背景与任务对应

本仓库完成测试任务 **B** 与进阶任务 **B2**，分别对应基础运动仿真与自适应力控抓取。

| 任务 | 要求摘要 | 本仓库实现 |
|------|----------|------------|
| **任务 B** | 无硬件条件下搭建虚拟机械臂；输入目标坐标 `(x,y,z)`，末端平滑移动到该点；提交源码 + Video Demo | `task_B/` |
| **任务 B2** | ① 虚拟力传感并记录正压力时序；② 铁块/海绵自适应力控抓取；③ 抬升后外力下的滑移对冲；更新 GitHub + 对比录屏 | `task_B2/` |

> 工程要点：URDF 左右夹爪行程不一致（左 0-0.05 m，右 0-0.0715 m）。本项目采用**指心恒定对称开合**，避免单指先触、左右正压力失衡。

---

## 二、功能说明

### 任务 B — 机械臂运动仿真与虚拟控制（`task_B/`）

- 加载 reBot-DevArm URDF 三维模型（固定末端）
- 阅读并使用 PyBullet 逆运动学（IK）求解关节角
- 终端输入目标 TCP 坐标 `(x, y, z)`，末端平滑插值运动
- 3D GUI 可视化；`find_ee.py` 辅助查询关节 / Link 索引
S
### 任务 B2 — 自适应力控抓取（`task_B2/`）

对应任务三项核心进阶要求：

1. **虚拟力传感器部署**（`1.Force_Sensor_Simulation.py`）  
   - `getContactPoints` 正压力 + 关节力/力矩传感器  
   - 闭合过程实时打印，导出 CSV / 曲线

2. **自适应力控抓取**（`2.adaptive_force_control_grasp.py`）  
   - 同场景：重硬铁块 + 轻软海绵  
   - 接触 → 探刚度 → 选安全力 `F_safe` → PID 恒力  
   - 力控开：安全抓起；力控关：易碎物可被“捏扁”（对照 Demo）

3. **双向滑移对冲**（`3.slip_compensation_test.py`）  
   - 抓起后施加向下外力  
   - 感知摩擦/滑移异常时自动加紧；关闭对冲则易掉落

---

## 三、仓库结构

```text
my-reBot-DevArm/
├── task_B/                                    # 任务 B：运动仿真与虚拟控制
│   ├── find_ee.py                             # 查询关节 / 末端 Link 索引
│   └── rebot_sim.py                           # 输入 (x,y,z) → IK → 平滑运动
├── task_B2/                                   # 任务 B2：力传感 / 力控 / 滑移对冲
│   ├── 1.Force_Sensor_Simulation.py           # ① 虚拟力传感器 + 正压力时序
│   ├── 2.adaptive_force_control_grasp.py      # ② 自适应力控（铁块 + 海绵）
│   ├── 3.slip_compensation_test.py            # ③ 滑移对冲对照
│   ├── utils/
│   │   ├── robot.py                           # 加载含夹爪 URDF，解析索引
│   │   ├── gripper.py                         # 指心对称开合 + 力读数
│   │   ├── ik.py                              # TCP 逆解与数值精调
│   │   └── scene.py                           # 可选默认场景
│   └── force_data/                            # 运行后生成 CSV / 曲线图
├── urdf/                                      # 共用模型（task_B / task_B2 均使用）
│   ├── reBot-DevArm_fixend_description/       # 任务 B：固定末端
│   └── 00-arm-rs_asm-v3/                      # 任务 B2：含平行夹爪
├── requirements.txt
├── .gitignore
└── README.md
```

---

## 四、环境配置

### 环境要求

| 项目 | 要求 |
|------|------|
| Python | ≥ 3.10 |
| 操作系统 | Windows / Linux / macOS |
| 依赖 | PyBullet、NumPy、Matplotlib |

### 安装

```bash
git clone https://github.com/XIAOHU7771/my-reBot-DevArm.git
cd my-reBot-DevArm

# Windows
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Linux / macOS
# python3 -m venv .venv && source .venv/bin/activate

pip install -r requirements.txt
```

> Windows 安装 PyBullet 若提示缺少编译工具：安装 [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)，勾选「使用 C++ 的桌面开发」，然后重启终端再装。

---

## 五、运行说明

### 5.1 任务 B — 点位运动仿真

```bash
# 首次：确认末端 Link 索引（常见为 6）
python task_B/find_ee.py

# 主程序：输入目标坐标，例如 0.15, 0.0, 0.15
python task_B/rebot_sim.py
```

推荐工作空间：`x: 0.05~0.25`，`y: -0.15~0.15`，`z: 0.05~0.25`。

**任务产出**：屏幕录像，展示 IK 求解后末端平滑到达指定点。

### 5.2 任务 B2 — 力控与滑移对冲

建议在 `task_B2` 目录下运行（便于写入 `force_data/`）：

```bash
cd task_B2

# ① 虚拟力传感器：实时正压力 + CSV/曲线
python 1.Force_Sensor_Simulation.py

# ② 自适应力控开启（铁块 + 海绵安全抓起）
python 2.adaptive_force_control_grasp.py --force-control

# ② 力控关闭对照（海绵易被捏扁）
python 2.adaptive_force_control_grasp.py --no-force-control --only sponge

# ③ 滑移对冲开 / 关
python 3.slip_compensation_test.py --compensate
python 3.slip_compensation_test.py --no-compensate
```

也可改脚本内开关：`ENABLE_FORCE_CONTROL`、`ENABLE_SLIP_COMPENSATION`。

**任务产出（Demo 2.0）**：

- 力控关：夹爪无脑闭合 → 易碎物被捏扁 / 异常  
- 力控开：接触后轻柔停在安全力 → 重物与易碎品均能抬起  
- （进阶）滑移对冲：外力扰动下自动加紧防掉落  

---

## 六、核心算法简述

### 6.1 逆运动学（任务 B）

1. 输入：目标 TCP `(x,y,z)` + 当前关节角  
2. PyBullet `calculateInverseKinematics` 迭代求解（如最多 500 次）  
3. 关节空间线性插值，电机位置控制平滑运动到目标  

### 6.2 指心对称开合（任务 B2）

左右行程不同时，等量回缩保持 `q_right - q_left` 恒定，指心不漂，利于双侧均匀接触。

### 6.3 自适应力控 + 滑移对冲（任务 B2）

```text
闭合接触 → 微探估计刚度 k≈ΔF/Δw → 选 F_safe(硬高/软低)
         → PID 力伺服维持恒力 → 抬升验证
抬升后外力 ↓ → 检测滑移/摩擦异常 → 对称加紧（对冲开启时）
```

---

## 七、任务交付对照

| 产出要求 | 状态 |
|----------|------|
| 任务 B：位置 IK 运动录屏 | ✅ [基础点位运动视频 (v1.0 / Demo.mp4)](https://github.com/XIAOHU7771/my-reBot-DevArm/releases/tag/v1.0) |
| 任务 B2：力控开/关对比 Demo 录屏 | ✅ [自适应力控抓取仿真演示视频 (v2.0)](https://github.com/XIAOHU7771/my-reBot-DevArm/releases/tag/v2.0) |

---

## 八、故障排查

| 问题 | 处理 |
|------|------|
| PyBullet 安装失败 | 安装 C++ Build Tools（桌面 C++ 工作负载）后重装 |
| IK 失败 / 臂不动 | 运行 `find_ee.py` 核对末端索引；目标点改到推荐工作空间 |
| 平滑插值运动过快 / 抖动 / 像瞬移 | `task_B/rebot_sim.py` 中增大插值步数 `steps`（如 100→200~300），或略增大 `time.sleep`（如 0.01→0.02）；确认每步都调用了 `p.stepSimulation()` |
| 平滑插值走完仍未到目标点 | 先确认 IK 解合理（打印的目标关节角无异常跳变）；目标点勿超出工作空间；可适当增大 `maxNumIterations` |
| 夹爪左右力差过大 | 确认 `keep_center=True`（指心对称开合） |
| 找不到 URDF | 确认仓库根目录存在 `urdf/...`，并从仓库根或 `task_B2/` 按上文命令运行 |

---

## 九、许可证与致谢

本项目基于 reBot-DevArm 开源机械臂，遵循 **CERN-OHL-W-2.0**。

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/main) — 开源六轴机械臂硬件
- PyBullet — 多刚体物理仿真引擎
- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) — 运动学控制与实机控制参考
