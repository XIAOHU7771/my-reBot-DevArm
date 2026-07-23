# reBot-DevArm 虚拟仿真控制

基于 PyBullet 的 reBot-DevArm 机械臂虚拟仿真与控制项目。

仓库地址：[https://github.com/XIAOHU7771/my-reBot-DevArm](https://github.com/XIAOHU7771/my-reBot-DevArm)

---

## 📌 项目简介

本项目实现了 reBot-DevArm 六轴机械臂在 PyBullet 物理引擎中的虚拟仿真与控制：

- **simulation/**：终端输入目标坐标 `(x, y, z)`，末端平滑移动到指定位置
- **simulation2/**：虚拟力传感、自适应力控抓取、滑移对冲对照实验

### 核心功能

- ✅ 加载 reBot-DevArm 机械臂 URDF 模型
- ✅ 支持用户交互式输入目标坐标
- ✅ 基于 PyBullet IK 求解器计算逆运动学
- ✅ 平滑插值轨迹，实现末端平滑运动
- ✅ 夹爪虚拟力 / 触觉传感与时序记录
- ✅ 自适应力控抓取（铁块 + 海绵）
- ✅ 滑移对冲（抬升后外力扰动对照）
- ✅ 3D 可视化仿真界面

---

## 📁 项目结构

```text
my-reBot-DevArm/
├── simulation/                              # 基础 TCP 点动
│   ├── rebot_sim.py                         # 主控制脚本
│   ├── find_ee.py                           # 查找末端链接索引
│   ├── README.md
│   ├── requirements.txt
│   └── urdf/                                # （兼容旧路径）模型副本
├── simulation2/                             # 力觉 / 力控 / 滑移对冲
│   ├── 1.Force_Sensor_Simulation.py
│   ├── 2.adaptive_force_control_grasp.py
│   ├── 3.slip_compensation_test.py
│   ├── utils/
│   ├── force_data/                          # 运行后生成 CSV / 图
│   ├── README.md
│   └── requirements.txt
├── urdf/                                    # 机械臂 URDF（推荐使用）
│   ├── 00-arm-rs_asm-v3/                    # 含夹爪（simulation2）
│   └── reBot-DevArm_fixend_description/     # 固定末端（simulation）
├── requirements.txt
└── README.md
```

---

## 🛠 环境要求

| 项目 | 要求 |
|------|------|
| Python | 3.10+ |
| 操作系统 | Windows / Linux / macOS |
| 依赖库 | PyBullet, NumPy, Matplotlib |

---

## 📦 安装步骤

### 1. 克隆项目

```bash
git clone https://github.com/XIAOHU7771/my-reBot-DevArm.git
cd my-reBot-DevArm
```

### 2. 创建并激活虚拟环境

**Windows:**

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

**Linux / macOS:**

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. 安装依赖

```bash
pip install -r requirements.txt
```

> **注意**：在 Windows 上安装 PyBullet 时，如果提示缺少 Visual C++ 编译工具，请先安装 [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)，确保勾选 **“使用 C++ 的桌面开发”** 工作负载。

---

## 🚀 运行方法

### A. 基础 TCP 点动（simulation）

#### 第一步：查找末端链接索引（首次运行）

```bash
python simulation/find_ee.py
```

记下 `end_link` 的索引（常见为 `6`）。

#### 第二步：运行主控制脚本

```bash
python simulation/rebot_sim.py
```

终端提示输入目标坐标，例如：

```text
0.15, 0.0, 0.15
```

> ⚠️ 建议工作空间：`x: 0.05~0.25`, `y: -0.15~0.15`, `z: 0.05~0.25`。

详细说明见 [`simulation/README.md`](simulation/README.md)。

### B. 力传感 / 力控 / 滑移对冲（simulation2）

```bash
cd simulation2

# 任务1：虚拟力传感器
python 1.Force_Sensor_Simulation.py

# 任务2：自适应力控（开 / 关对照）
python 2.adaptive_force_control_grasp.py --force-control
python 2.adaptive_force_control_grasp.py --no-force-control --only sponge

# 任务3：滑移对冲（开 / 关对照）
python 3.slip_compensation_test.py --compensate
python 3.slip_compensation_test.py --no-compensate
```

详细说明见 [`simulation2/README.md`](simulation2/README.md)。

---

## 📊 代码架构

```text
┌─────────────────────────────────────────────────────────────┐
│         simulation / simulation2 实验与控制脚本              │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  utils（simulation2）：robot / gripper / ik / scene         │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    PyBullet 物理引擎                        │
│  URDF · IK · 接触力 · 刚体动力学 · 3D 渲染                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 🛠 故障排查

### 1. 安装 PyBullet 时提示缺少 Visual C++ 编译工具

- 安装 Microsoft C++ Build Tools，勾选 **“使用 C++ 的桌面开发”**
- 安装完成后重启电脑

### 2. IK 求解失败

- 重新运行 `python simulation/find_ee.py` 确认末端索引
- 目标点可能超出工作空间，先试 `[0.15, 0.0, 0.15]`

### 3. simulation2 找不到 URDF

- 请从仓库根目录或 `simulation2/` 目录按 README 运行
- 确认存在 `urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf` 及对应 `meshes/`

---

## 🤝 致谢

- reBot-DevArm — 开源硬件项目
- reBotArm_control_py — 运动学控制库参考
- [PyBullet](https://pybullet.org/) — 物理仿真引擎
