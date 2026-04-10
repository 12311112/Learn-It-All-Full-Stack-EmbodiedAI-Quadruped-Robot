"""
硬件接口模块

封装 rustypot 库与 Feetech ST-2000-C001 舵机的通信。
提供连接、读取位置、写入位置、使能/失能扭矩等基础操作。
"""

import rustypot
import numpy as np
from config.robot_config import RobotConfig


class HardwareInterface:
    """舵机硬件通信接口"""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.io = None
        self.connected = False
        self.servo_ids = list(range(12))

    # ==================== 连接管理 ====================

    def connect(self) -> bool:
        """连接舵机控制器"""
        try:
            self.io = self._create_controller()
            self.connected = True
            print(f"已连接到 {self.config.usb_port} (波特率: {self.config.baudrate})")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """断开连接"""
        if self.connected:
            self.disable_all_torque()
            self.connected = False
            self.io = None
            print("已断开舵机连接")

    def _create_controller(self):
        """创建舵机控制器实例（兼容不同 rustypot 版本）"""
        if hasattr(rustypot, "feetech"):
            return rustypot.feetech(self.config.usb_port, self.config.baudrate)
        if hasattr(rustypot, "Sts3215PyController"):
            return rustypot.Sts3215PyController(
                self.config.usb_port, self.config.baudrate, 0.1
            )
        raise RuntimeError("不支持的 rustypot 版本, 请检查安装")

    # ==================== 扭矩控制 ====================

    def disable_all_torque(self):
        """失能所有舵机扭矩（放松）"""
        if not self.connected:
            return
        if hasattr(self.io, "sync_write_torque_enable"):
            self.io.sync_write_torque_enable(self.servo_ids, [False] * 12)
        elif hasattr(self.io, "disable_torque"):
            self.io.disable_torque(self.servo_ids)

    def enable_all_torque(self):
        """使能所有舵机扭矩"""
        if not self.connected:
            return
        if hasattr(self.io, "sync_write_torque_enable"):
            self.io.sync_write_torque_enable(self.servo_ids, [True] * 12)
        elif hasattr(self.io, "enable_torque"):
            self.io.enable_torque(self.servo_ids)

    # ==================== 位置读写 ====================

    def read_position(self, servo_id: int) -> int:
        """读取单个舵机的当前原始位置值"""
        if hasattr(self.io, "sync_read_present_position"):
            positions = self.io.sync_read_present_position([servo_id])
            return int(positions[0])
        return int(self.io.read_present_position(servo_id))

    def read_all_positions(self) -> list:
        """读取所有舵机的当前原始位置值"""
        if hasattr(self.io, "sync_read_present_position"):
            return [int(p) for p in self.io.sync_read_present_position(self.servo_ids)]
        return [int(self.io.read_present_position(sid)) for sid in self.servo_ids]

    def write_position(self, servo_id: int, position: int):
        """
        写入单个舵机的目标位置（原始值）。
        自动钳位到关节限位范围内，确保机械安全。
        """
        clamped = self.config.clamp_position(servo_id, position)
        if hasattr(self.io, "sync_write_goal_position"):
            self.io.sync_write_goal_position([servo_id], [clamped])
        else:
            self.io.write_goal_position([servo_id], [clamped])

    def write_all_positions(self, positions: dict):
        """
        批量写入所有舵机的目标位置。
        自动钳位到关节限位范围内，确保机械安全。

        参数:
            positions: {servo_id(int): 原始位置值}
        """
        ids = sorted(positions.keys())
        # 钳位所有位置值
        values = [self.config.clamp_position(sid, positions[sid]) for sid in ids]
        if hasattr(self.io, "sync_write_goal_position"):
            self.io.sync_write_goal_position(ids, values)
        else:
            self.io.write_goal_position(ids, values)
