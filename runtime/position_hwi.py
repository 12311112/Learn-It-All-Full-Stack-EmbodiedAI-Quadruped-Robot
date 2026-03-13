import time

import numpy as np
import rustypot

class HWI:
    def __init__(self, usb_port: str = "/dev/ttyUSB0"):
        # --- 用户自定义的12舵机四足配置 ---
        self.joints = {
            "left_front_hip_joint": 0,
            "left_front_knee_joint": 1,
            "left_front_ankle_joint": 2,
            "left_back_hip_joint": 3,
            "left_back_knee_joint": 4,
            "left_back_ankle_joint": 5,
            "right_front_hip_joint": 9,
            "right_front_knee_joint": 10,
            "right_front_ankle_joint": 11,
            "right_back_hip_joint": 6,
            "right_back_knee_joint": 7,
            "right_back_ankle_joint": 8,
        }

        # 零位字典 (所有关节归中)
        self.zero_pos = {
            "left_front_hip_joint": 0.0,
            "left_front_knee_joint": 0.0,
            "left_front_ankle_joint": 0.0,
            "left_back_hip_joint": 0.0,
            "left_back_knee_joint": 0.0,
            "left_back_ankle_joint": 0.0,
            "right_front_hip_joint": 0.0,
            "right_front_knee_joint": 0.0,
            "right_front_ankle_joint": 0.0,
            "right_back_hip_joint": 0.0,
            "right_back_knee_joint": 0.0,
            "right_back_ankle_joint": 0.0,
        }

        # 初始站立姿态字典
        # 注意：这里我先全部填0，你需要根据你的机械结构实际情况修改这些数值！
        self.init_pos = {
        "left_front_hip_joint": -0.138058,
        "left_front_knee_joint": 2.38534,
        "left_front_ankle_joint": 0.371223,
        "left_back_hip_joint": 0.34668,
        "left_back_knee_joint": -0.648874,
        "left_back_ankle_joint": 0.403437,
        "right_front_hip_joint": -0.949534,
        "right_front_knee_joint": 1.756408,
        "right_front_ankle_joint": -0.441786,
        "right_back_hip_joint": 0.016874,
        "right_back_knee_joint": -0.648874,
        "right_back_ankle_joint": 2.176719,
        }
    
        # 舵机偏移量字典 (用于校准机械零点)
        # 请在校准后填入具体数值
        self.joints_offsets = {joint: 0.0 for joint in self.joints.keys()}

        # --- 控制参数 ---
        self.kps = np.ones(len(self.joints)) * 32  # 默认刚度
        self.kds = np.ones(len(self.joints)) * 0   # 默认阻尼
        self.low_torque_kps = np.ones(len(self.joints)) * 2 # 启动时低刚度

        self.io = rustypot.feetech(usb_port, 1000000)


    def set_kps(self, kps):
        self.kps = kps
        self.io.set_kps(list(self.joints.values()), self.kps)

    def set_kds(self, kds):
        self.kds = kds
        self.io.set_kds(list(self.joints.values()), self.kds)

    def set_kp(self, id, kp):
        self.io.set_kps([id], [kp])

    def turn_on(self):
        self.io.set_kps(list(self.joints.values()), self.low_torque_kps)
        print("turn on : low KPS set")
        time.sleep(1)
        self.set_position_all(self.init_pos)
        print("turn on : init pos set")
        time.sleep(1)
        self.io.set_kps(list(self.joints.values()), self.kps)
        print("turn on : high kps")

    def turn_off(self):
        self.io.disable_torque(list(self.joints.values()))

    def set_position(self, joint_name, pos):
        """
        pos is in radians
        """
        id = self.joints[joint_name]
        pos = pos + self.joints_offsets[joint_name]
        self.io.write_goal_position([id], [pos])

    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: position + self.joints_offsets[joint]
            for joint, position in joints_positions.items()
        }

        self.io.write_goal_position(
            list(self.joints.values()), list(ids_positions.values())
        )

    def get_present_positions(self, ignore=[]):
        """
        Returns the present positions in radians
        """

        try:
            present_positions = self.io.read_present_position(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None

        present_positions = [
            pos - self.joints_offsets[joint]
            for joint, pos in zip(self.joints.keys(), present_positions)
            if joint not in ignore
        ]
        return np.array(np.around(present_positions, 3))

    def get_present_velocities(self, rad_s=True, ignore=[]):
        """
        Returns the present velocities in rad/s (default) or rev/min
        """
        try:
            present_velocities = self.io.read_present_velocity(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None

        present_velocities = [
            vel
            for joint, vel in zip(self.joints.keys(), present_velocities)
            if joint not in ignore
        ]
        return np.array(np.around(present_velocities, 3))

