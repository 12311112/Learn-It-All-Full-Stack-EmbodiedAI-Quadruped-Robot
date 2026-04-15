import time

import numpy as np
import rustypot

class HWI:
    def __init__(self, usb_port: str = "/dev/ttyUSB0"):


        # --- 用户自定义的12舵机四足配置 --- #################
        self.joints = {
            "right_front_hip_joint": 6,
            "right_front_knee_joint": 7,
            "right_front_ankle_joint": 8,

            "left_front_hip_joint": 0,
            "left_front_knee_joint": 4,   # 4
            "left_front_ankle_joint": 5,  # 5

            "right_back_hip_joint": 9,    # 9
            "right_back_knee_joint": 10,  # 10
            "right_back_ankle_joint": 11, # 11

            "left_back_hip_joint": 3,
            "left_back_knee_joint": 1,    # 1
            "left_back_ankle_joint": 2,   # 2
        }

        # 初始站立姿态字典
        self.init_pos = {
            "right_front_hip_joint": -0.058291,
            "right_front_knee_joint": 0.696427,
            "right_front_ankle_joint": -0.058291,

            "left_front_hip_joint": -0.055223,
            "left_front_knee_joint": -0.587515,
            "left_front_ankle_joint": 0.257709,

            "right_back_hip_joint": -0.895845,
            "right_back_knee_joint": 0.80534,
            "right_back_ankle_joint": -0.409573,

            "left_back_hip_joint": 0.299126,
            "left_back_knee_joint": 0.185612,
            "left_back_ankle_joint": -0.572175,
        }
        self.real_pose_signs_rl = {
            "right_front_hip_joint": -1.0,
            "right_front_knee_joint": 1.0,
            "right_front_ankle_joint": 1.0,

            "left_front_hip_joint": 1.0,       #######
            "left_front_knee_joint": -1.0,
            "left_front_ankle_joint": -1.0,

            "right_back_hip_joint": 1.0,
            "right_back_knee_joint": 1.0,
            "right_back_ankle_joint": 1.0,

            "left_back_hip_joint": -1.0,
            "left_back_knee_joint": -1.0,      ####
            "left_back_ankle_joint": -1.0,     #####
        }

        # self.real_pose = {  #####################不通用
        # "left_front_hip_joint":0 , ##+++++++  
        # "left_front_knee_joint":-0.96754,#-----
        # "left_front_ankle_joint":1.850729,###-----

        # "left_back_hip_joint": 0,#------
        # "left_back_knee_joint": -0.96754,#----
        # "left_back_ankle_joint": 1.850729,#-----

        # "right_front_hip_joint": 0,#+++
        # "right_front_knee_joint": -0.96754,#++++
        # "right_front_ankle_joint": 1.850729,#+++++

        # "right_back_hip_joint":  0,#----
        # "right_back_knee_joint": -0.96754,####+++++
        # "right_back_ankle_joint": 1.850729,#+++++
        # }


        # real_pose 坐标系到控制器坐标系的符号映射
        # 如果某个关节 real_pose 是反向定义，就把该关节改成 -1
    #     self.real_pose_signs = {  #####################不通用
    #     "left_front_hip_joint": 1.0,
    #     "left_front_knee_joint": 1.0,
    #     "left_front_ankle_joint": 1.0,

    #     "left_back_hip_joint": -1.0,
    #     "left_back_knee_joint": 1.0,
    #     "left_back_ankle_joint": 1.0,

    #     "right_front_hip_joint": -1.0,
    #     "right_front_knee_joint": -1.0,  
    #     "right_front_ankle_joint": -1.0,

    #     "right_back_hip_joint": 1.0,
    #     "right_back_knee_joint": -1.0,
    #     "right_back_ankle_joint": -1.0,
    # }

        # 舵机偏移量字典 (用于校准机械零点)
        # 请在校准后填入具体数值
        self.joints_offsets = {joint: 0.0 for joint in self.joints.keys()}

        # --- 控制参数 ---
        self.kps = np.ones(len(self.joints)) * 8  # 默认刚度
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
        self.io.set_kps(list(self.joints.values()), self.kps)################
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
        ids = []
        positions = []

        for joint, position in joints_positions.items():
            if joint not in self.joints:
                raise KeyError(f"Unknown joint name: {joint}")
            ids.append(self.joints[joint])
            positions.append(position + self.joints_offsets[joint])

        self.io.write_goal_position(ids, positions)

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
