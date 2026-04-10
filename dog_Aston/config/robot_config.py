"""
机器人配置模块

包含舵机映射、串口参数、关节限位等所有硬件相关配置。
所有参数均来源于 dog.urdf 机器人模型定义。
"""

import os
import json
import math


class RobotConfig:
    """机器人硬件配置"""

    def __init__(self):
        # ==================== 串口配置 ====================
        self.usb_port = "/dev/ttyUSB0"
        self.baudrate = 1000000

        # ==================== 舵机ID映射 ====================
        # 12个 Feetech ST-2000-C001 串行总线舵机 (20KG, 360°/4096)
        # 每条腿3个关节: hip(髋), thigh(大腿), calf(小腿)
        self.joints = {
            "FL_hip_joint":   0,   # 左前髋
            "FL_thigh_joint": 1,   # 左前大腿
            "FL_calf_joint":  2,   # 左前小腿
            "RL_hip_joint":   3,   # 左后髋
            "RL_thigh_joint": 4,   # 左后大腿
            "RL_calf_joint":  5,   # 左后小腿
            "FR_hip_joint":   9,   # 右前髋
            "FR_thigh_joint": 10,   # 右前大腿
            "FR_calf_joint":  11,   # 右前小腿
            "RR_hip_joint":   6,   # 右后髋
            "RR_thigh_joint": 7,  # 右后大腿
            "RR_calf_joint":  8,  # 右后小腿
        }

        # 反向映射: ID -> 关节名
        self.id_to_joint = {v: k for k, v in self.joints.items()}

        # 腿组定义
        self.legs = {
            "FL": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
            "RL": ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
            "FR": ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
            "RR": ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
        }

        # 腿名中文映射
        self.leg_names_cn = {
            "FL": "左前腿", "RL": "左后腿",
            "FR": "右前腿", "RR": "右后腿",
        }

        # 关节类型中文映射
        self.joint_type_cn = {
            "hip": "髋关节", "thigh": "大腿关节", "calf": "小腿关节",
        }

        # ==================== 舵机方向 ====================
        # 1 = 正方向, -1 = 反方向
        self.joint_directions = {name: 1 for name in self.joints}

        # ==================== URDF关节角度范围 (弧度, 相对于校准零位) ====================
        # hip: ±45°, thigh: ±80°, calf: 0°~70°
        self._urdf_joint_ranges = {
            "FL_hip_joint":   (-0.785398, 0.785398),   # -45° ~ 45°
            "FL_thigh_joint": (-1.396263, 1.396263),   # -80° ~ 80°
            "FL_calf_joint":  (0.0, 1.221730),         # 0° ~ 70°
            "RL_hip_joint":   (-0.785398, 0.785398),
            "RL_thigh_joint": (-1.396263, 1.396263),
            "RL_calf_joint":  (0.0, 1.221730),
            "FR_hip_joint":   (-0.785398, 0.785398),
            "FR_thigh_joint": (-1.396263, 1.396263),
            "FR_calf_joint":  (0.0, 1.221730),
            "RR_hip_joint":   (-0.785398, 0.785398),
            "RR_thigh_joint": (-1.396263, 1.396263),
            "RR_calf_joint":  (0.0, 1.221730),
        }

        # ==================== 舵机转换参数 ====================
        # ST-2000-C001: 0-4095 对应 0-360°
        self.raw_per_radian = 4096 / (2 * math.pi)  # ≈ 651.9

        # ==================== 机器人尺寸 (米, 来自URDF) ====================
        self.body_length = 0.265     # 身体长度 (X)
        self.body_width = 0.1095     # 身体宽度 (Y)
        self.body_height = 0.07      # 身体高度 (Z)
        self.hip_offset = 0.02585    # 髋关节偏移 (coxa)
        self.thigh_length = 0.10725  # 大腿长度 (femur)
        self.calf_length = 0.1184    # 小腿长度 (tibia)

        # ==================== 控制参数 ====================
        self.control_frequency = 50  # Hz
        self.control_dt = 1.0 / self.control_frequency

        # ==================== 配置文件路径 ====================
        self._config_dir = os.path.dirname(os.path.abspath(__file__))
        self.calibration_file = os.path.join(self._config_dir, "servo_offsets.json")
        self.poses_file = os.path.join(self._config_dir, "poses.json")

        # ==================== 舵机零位偏移 ====================
        # 校准后的原始位置值, 作为所有控制的基准
        self.servo_offsets = self._load_offsets()

        # ==================== 姿态数据 ====================
        self.poses = self._load_poses()

    def _load_offsets(self) -> dict:
        """从文件加载校准偏移量, 不存在则返回空字典"""
        if os.path.exists(self.calibration_file):
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                return json.load(f)
        return {}

    def save_offsets(self, offsets: dict):
        """保存校准偏移量到文件"""
        with open(self.calibration_file, "w", encoding="utf-8") as f:
            json.dump(offsets, f, indent=2, ensure_ascii=False)
        self.servo_offsets = offsets

    def is_calibrated(self) -> bool:
        """检查是否已完成校准"""
        return len(self.servo_offsets) == 12

    @property
    def joint_limits(self) -> dict:
        """
        基于校准零位的关节限位（原始舵机位置值）。
        每个关节的可动范围 = 校准零位 ± URDF角度范围 转换为原始值。
        未校准时以舵机中位(2048)为零位。
        """
        limits = {}
        for name, (lower_rad, upper_rad) in self._urdf_joint_ranges.items():
            servo_id = self.joints[name]
            offset = self.servo_offsets.get(str(servo_id), 2048)
            limits[name] = (
                offset + int(lower_rad * self.raw_per_radian),
                offset + int(upper_rad * self.raw_per_radian),
            )
        return limits

    def get_joint_info(self, servo_id: int) -> str:
        """获取舵机的可读描述"""
        joint_name = self.id_to_joint.get(servo_id, f"unknown_{servo_id}")
        parts = joint_name.split("_")
        leg = parts[0]
        jtype = parts[1]
        leg_cn = self.leg_names_cn.get(leg, leg)
        jtype_cn = self.joint_type_cn.get(jtype, jtype)
        return f"ID={servo_id}  {joint_name}  ({leg_cn} {jtype_cn})"

    # ==================== 姿态管理 ====================

    def _load_poses(self) -> dict:
        """从文件加载姿态数据"""
        if os.path.exists(self.poses_file):
            with open(self.poses_file, "r", encoding="utf-8") as f:
                return json.load(f)
        return {"默认": {str(i): 0.0 for i in range(12)}}

    def save_poses(self):
        """保存姿态数据到文件"""
        with open(self.poses_file, "w", encoding="utf-8") as f:
            json.dump(self.poses, f, indent=2, ensure_ascii=False)

    def validate_pose(self, pose_data: dict) -> tuple:
        """
        验证姿态是否在关节限位内。

        参数:
            pose_data: {servo_id_str: 角度偏移(弧度)}, 相对于校准零位

        返回:
            (是否有效, 错误信息)
        """
        for sid_str, angle_rad in pose_data.items():
            servo_id = int(sid_str)
            joint_name = self.id_to_joint.get(servo_id)
            if joint_name is None:
                return False, f"无效的舵机ID: {servo_id}"

            lower_rad, upper_rad = self._urdf_joint_ranges[joint_name]
            if angle_rad < lower_rad or angle_rad > upper_rad:
                info = self.get_joint_info(servo_id)
                return False, (
                    f"{info} 角度超出限位!\n"
                    f"  设定值: {math.degrees(angle_rad):.1f}°\n"
                    f"  允许范围: {math.degrees(lower_rad):.1f}° ~ {math.degrees(upper_rad):.1f}°"
                )
        return True, ""

    def pose_to_raw_positions(self, pose_data: dict) -> dict:
        """
        将姿态角度偏移转换为舵机原始位置值。

        参数:
            pose_data: {servo_id_str: 角度偏移(弧度)}

        返回:
            {servo_id(int): 原始位置值}
        """
        positions = {}
        for sid_str, angle_rad in pose_data.items():
            servo_id = int(sid_str)
            offset = self.servo_offsets.get(str(servo_id), 2048)
            direction = self.joint_directions[self.id_to_joint[servo_id]]
            positions[servo_id] = offset + int(direction * angle_rad * self.raw_per_radian)
        return positions

    def clamp_position(self, servo_id: int, position: int) -> int:
        """
        将舵机原始位置值钳位到关节限位范围内。

        参数:
            servo_id: 舵机ID
            position: 原始位置值

        返回:
            钳位后的位置值
        """
        joint_name = self.id_to_joint.get(servo_id)
        if joint_name is None:
            return position  # 未知舵机，不钳位

        limits = self.joint_limits
        if joint_name not in limits:
            return position  # 无限位信息，不钳位

        lower, upper = limits[joint_name]
        return max(lower, min(upper, position))
