# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

四足机器人 (SpotMicro) 控制系统，使用 12 个 Feetech ST-2000-C001 串行总线舵机（20KG 扭矩，360°/4096 分辨率），通过 USB 串口由 Jetson Nano 控制。项目完全自包含，不依赖外部文件。

## 运行命令

```bash
# 环境配置
conda create -n env_dog python=3.10
conda activate env_dog
pip install -r requirements.txt   # numpy, rustypot

# 主程序入口
python main.py                     # 交互式菜单: 1=校准 2=姿态管理

# 直接运行工具
python -m tools.calibrate          # 舵机零位校准（首次必须）
python -m tools.pose_manager       # 姿态管理（需先校准）
```

## 架构

**数据流向**: `RobotConfig` → `HardwareInterface` → `tools/*` → `main.py`

### 核心模块

**config/robot_config.py** — 硬件配置中心（`RobotConfig` 类）
- 舵机ID映射（12个舵机，每腿3关节: hip/thigh/calf）
- 串口参数（默认 `/dev/ttyUSB0`, 1000000 波特率）
- URDF 机器人尺寸（身体/腿部连杆长度）
- **关节限位计算**（`joint_limits` 动态属性，基于校准零位实时计算）
- 校准数据持久化（`servo_offsets.json`）
- 姿态数据管理（`poses.json`）

**core/hardware_interface.py** — rustypot 通信封装（`HardwareInterface` 类）
- **版本兼容**: 通过 `hasattr` 检查支持新旧 rustypot API
  - 新版: `feetech()` + `sync_read_present_position` / `sync_write_goal_position`
  - 旧版: `Sts3215PyController` + `read_present_position` / `write_goal_position`
- **自动钳位**: 所有 `write_position()` / `write_all_positions()` 调用自动钳位到关节限位
- 扭矩控制: `enable_all_torque()` / `disable_all_torque()`

### 工具模块

**tools/calibrate.py** — 零位校准（首次使用必须执行）
- 流程: 放松舵机 → 手动调整 ID 0~11 → 按回车确认 → 保存原始位置值
- 输出: `config/servo_offsets.json`（所有后续控制的基准）

**tools/pose_manager.py** — 姿态管理（需先完成校准）
- 执行姿态: 平滑插值过渡（可设置时长）+ 锁定舵机
- 回归默认: 所有关节回到校准零位
- 放松舵机: 失能扭矩，允许手动调整
- CRUD 操作: 新建/编辑/删除/查看自定义姿态

**main.py** — 交互式菜单入口

### 开发状态
- ✅ 已实现: 校准、姿态管理
- ⏳ 待开发: 逆运动学(IK)、步态生成、运动控制、RL策略推理

## 数据文件

### config/servo_offsets.json — 校准零位
```json
{"0": 2048, "1": 2100, "2": 1950, ...}  // servo_id: raw_position (0-4095)
```
- 由 `tools/calibrate.py` 生成，`RobotConfig.save_offsets()` 写入
- 所有控制操作的基准零位
- 未校准时默认使用舵机中位 2048

### config/poses.json — 姿态库
```json
{
  "默认": {"0": 0.0, "1": 0.0, ..., "11": 0.0},
  "站立": {"0": 0.0, "1": 0.785, "2": 0.524, ...}
}
```
- 键: 姿态名称，值: `{servo_id_str: 角度偏移(弧度)}`
- 角度偏移相对于校准零位
- 执行时转换: `实际位置 = 校准零位 + 角度偏移 × 651.9`
- 所有姿态必须通过 `validate_pose()` 验证（关节限位检查）

## 关节限位系统（硬约束）

**所有舵机运动必须在限位内，违反会导致机械损坏。**

### 自动保护机制
- `write_position()` / `write_all_positions()` 自动钳位到限位范围
- 插值过程中超限位置会被截断到边界值
- 放松状态可手动移动到任意位置，但执行姿态时会自动钳位回安全范围

### 关节角度范围（相对于校准零位，来自 dog.urdf）

| 关节类型 | 角度范围 | 弧度范围 |
| -------- | -------- | -------- |
| hip (髋) | -45° ~ +45° | -0.785398 ~ 0.785398 |
| thigh (大腿) | -80° ~ +80° | -1.396263 ~ 1.396263 |
| calf (小腿) | 0° ~ 70° | 0.0 ~ 1.221730 |

### 限位计算原理

```python
# 1. 校准零位（原始值 0-4095）
servo_offsets = {"0": 2048, "1": 2100, ...}

# 2. URDF 角度范围（弧度）
_urdf_joint_ranges = {"FR_hip_joint": (-0.785398, 0.785398), ...}

# 3. 动态计算绝对限位（@property joint_limits）
raw_per_radian = 4096 / (2π) ≈ 651.9
limit_lower = offset + int(lower_rad × raw_per_radian)
limit_upper = offset + int(upper_rad × raw_per_radian)
```

**示例**: ID=0 (FL髋), 校准零位=2000, 范围±45°
- 下限 = 2000 + (-0.785 × 651.9) = 1488
- 上限 = 2000 + (0.785 × 651.9) = 2512

### 校准零位建议（确保足够运动范围）
- **髋关节**: 腿部自然垂直（左右居中）
- **大腿关节**: 站立姿态（向前倾斜 45°~60°）
- **小腿关节**: 完全伸直（0° 零位）

## 硬件配置

### 舵机 ID 映射
```
FL (左前): 0=hip, 1=thigh, 2=calf
RL (左后): 3=hip, 4=thigh, 5=calf
FR (右前): 6=hip, 7=thigh, 8=calf
RR (右后): 9=hip, 10=thigh, 11=calf
```

### 通信参数
- 串口: `/dev/ttyUSB0`
- 波特率: `1000000`
- 控制频率: 50Hz
- 舵机型号: Feetech ST-2000-C001 (20KG, 360°/4096)
- 位置范围: 0-4095 (原始值对应 0-360°)

### 机器人尺寸（来自 dog.urdf）
- 身体: 0.265m × 0.1095m × 0.07m (长×宽×高)
- 髋关节偏移: 0.02585m
- 大腿长度: 0.10725m
- 小腿长度: 0.1184m

## 开发约定

- **语言**: 注释和文档使用简体中文
- **rustypot 兼容**: 使用 `hasattr` 检查 API 版本
  - 新版: `feetech()`, `sync_read_present_position`, `sync_write_goal_position`, `sync_write_torque_enable`
  - 旧版: `Sts3215PyController`, `read_present_position`, `write_goal_position`, `enable_torque`/`disable_torque`
- **舵机方向**: `joint_directions` 字典预留方向翻转（1=正, -1=反），当前全部为 1
- **位置单位**: 始终使用原始值（0-4095），角度转换系数 `raw_per_radian ≈ 651.9`
