#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Leika simulation(play/controller/gait/kinematics) 同构的真机开环脚本。

只保留一个前提：
- URDF 关节顺序与你的真实电机 ID 对齐
"""

import argparse
import math
import os
import sys
import time
from enum import Enum
from typing import Dict, List, TypedDict

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
    
from runtime.position_hwi import HWI

class GaitType(Enum):
    TROT = 0
    CRAWL = 1


default_offset = {
    GaitType.TROT: [0, 0.5, 0.5, 0],
    GaitType.CRAWL: [0, 1 / 4, 2 / 4, 3 / 4],
}

default_stand_frac = {
    GaitType.TROT: 3 / 4,
    GaitType.CRAWL: 3 / 4,
}


class KinConfig:
    # 与 Leika simulation/src/robot/kinematics.py 保持一致
    coxa = 60.5 / 100.0
    coxa_offset = 10.0 / 100.0
    femur = 111.2 / 100.0
    tibia = 118.5 / 100.0
    L = 207.5 / 100.0
    W = 78.0 / 100.0

    mount_offsets = np.array([[L / 2, 0, W / 2], [L / 2, 0, -W / 2], [-L / 2, 0, W / 2], [-L / 2, 0, -W / 2]])

    default_feet_positions = np.array(
        [
            [mount_offsets[0][0], 0, mount_offsets[0][2] + coxa],
            [mount_offsets[1][0], 0, mount_offsets[1][2] - coxa],
            [mount_offsets[2][0], 0, mount_offsets[2][2] + coxa],
            [mount_offsets[3][0], 0, mount_offsets[3][2] - coxa],
        ]
    )

    max_roll = 15 * np.pi / 2
    max_pitch = 15 * np.pi / 2
    max_body_shift_x = W / 3
    max_body_shift_z = W / 3
    max_leg_reach = femur + tibia - coxa_offset
    min_body_height = max_leg_reach * 0.45
    max_body_height = max_leg_reach * 0.9
    body_height_range = max_body_height - min_body_height
    max_step_length = max_leg_reach * 0.8
    max_step_height = max_leg_reach / 2

    default_step_depth = 0.002
    default_body_height = min_body_height + body_height_range / 2
    default_step_height = default_body_height / 2


class BodyState(TypedDict):
    omega: float
    phi: float
    psi: float
    xm: float
    ym: float
    zm: float
    px: float
    py: float
    pz: float
    feet: np.ndarray
    default_feet: np.ndarray


class GaitState(TypedDict):
    step_height: float
    step_x: float
    step_z: float
    step_angle: float
    step_depth: float
    stand_frac: float
    offset: List[float]
    gait_type: GaitType


length_multipliers = np.array([-1.4, -1.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 1.5, 1.5, 1.4, 1.0])
height_profile = np.array([0.0, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0.0, 0.0])


def sine_curve(length, angle, depth, phase):
    x_polar = np.cos(angle)
    z_polar = np.sin(angle)
    step = length * (1 - 2 * phase)
    x = step * x_polar
    z = step * z_polar
    y = -depth * np.cos((np.pi * (x + z)) / (2 * length)) if length != 0 else 0
    return np.array([x, y, z])


def yaw_arc(default_foot, current_foot):
    foot_mag = np.sqrt(default_foot[0] ** 2 + default_foot[2] ** 2)
    foot_dir = np.arctan2(default_foot[2], default_foot[0])
    offset_x = current_foot[0] - default_foot[0]
    offset_z = current_foot[2] - default_foot[2]
    offset_mag = np.sqrt(offset_x**2 + offset_z**2)
    offset_mod = np.arctan2(offset_mag, foot_mag)
    return np.pi / 2.0 + foot_dir + offset_mod


def get_control_points(length, angle, height):
    x_polar = np.cos(angle)
    z_polar = np.sin(angle)
    x = length * length_multipliers * x_polar
    z = length * length_multipliers * z_polar
    y = height * height_profile
    return np.stack([x, y, z], axis=1)


def bezier_curve(length, angle, height, phase):
    ctrl = get_control_points(length, angle, height)
    n = len(ctrl) - 1
    coeffs = np.array([math.comb(n, i) * (phase**i) * ((1 - phase) ** (n - i)) for i in range(n + 1)])
    return np.sum(ctrl * coeffs[:, None], axis=0)


class GaitController:
    def __init__(self, default_position: np.ndarray):
        self.default_position = default_position.copy()
        self.phase = 0.0

    def step(self, gait: GaitState, body: BodyState, dt: float):
        step_x, step_z, angle = gait["step_x"], gait["step_z"], gait["step_angle"]
        if not any((step_x, step_z, angle)):
            body["feet"] = body["feet"] + (self.default_position - body["feet"]) * dt * 10
            self.phase = 0.0
            return


        period = 2.0  # 秒
        self.phase = (self.phase + dt / period) % 1

        #self.phase = (self.phase + dt) % 1

        stand_fraction = gait["stand_frac"]
        depth = gait["step_depth"]
        height = gait["step_height"]
        offsets = gait["offset"]

        length = np.hypot(step_x, step_z)
        if step_x < 0:
            length = -length
        turn_amplitude = np.arctan2(step_z, length if length != 0 else 1e-8)

        new_feet = self.default_position.copy()
        for i, (default_foot, current_foot) in enumerate(zip(self.default_position, body["feet"])):
            phase = (self.phase + offsets[i]) % 1
            if phase < stand_fraction:
                ph_norm, curve_fn, amp = phase / stand_fraction, sine_curve, -depth
            else:
                ph_norm, curve_fn, amp = (phase - stand_fraction) / (1 - stand_fraction), bezier_curve, height

            delta_pos = curve_fn(length / 2, turn_amplitude, amp, ph_norm)
            delta_rot = curve_fn(angle * 2, yaw_arc(default_foot, current_foot), amp, ph_norm)

            new_feet[i][0] = default_foot[0] + delta_pos[0] + delta_rot[0] * 0.2
            new_feet[i][2] = default_foot[2] + delta_pos[2] + delta_rot[2] * 0.2
            if length or angle:
                new_feet[i][1] = default_foot[1] + delta_pos[1] + delta_rot[1] * 0.2

        body["feet"] = new_feet


class Kinematics:
    def __init__(self):
        self.coxa = KinConfig.coxa
        self.coxa_offset = KinConfig.coxa_offset
        self.femur = KinConfig.femur
        self.tibia = KinConfig.tibia
        self.mount_offsets = KinConfig.mount_offsets.copy()
        self.inv_mount_rot = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])

    def inverse_kinematics(self, body_state: BodyState):
        roll, pitch, yaw = np.deg2rad(body_state["omega"]), np.deg2rad(body_state["phi"]), np.deg2rad(body_state["psi"])
        xm, ym, zm = body_state["xm"], body_state["ym"], body_state["zm"]

        rot = self._rotation_matrix(roll, pitch, yaw)
        inv_rot = rot.T
        inv_tr = -inv_rot @ np.array([xm, ym, zm])

        angles = []
        for idx, foot_world in enumerate(body_state["feet"]):
            foot_body = inv_rot @ foot_world + inv_tr
            foot_local = self.inv_mount_rot @ (foot_body - self.mount_offsets[idx])
            x_local = -foot_local[0] if idx % 2 else foot_local[0]
            angles.extend(self._leg_ik(x_local, foot_local[1], foot_local[2]))
        return np.array(angles, dtype=float)

    def _leg_ik(self, x, y, z):
        f = np.sqrt(max(0.0, x * x + y * y - self.coxa * self.coxa))
        g = f - self.coxa_offset
        h = np.sqrt(g * g + z * z)

        theta1 = -np.arctan2(y, x) - np.arctan2(f, -self.coxa)
        d = (h * h - self.femur * self.femur - self.tibia * self.tibia) / (2 * self.femur * self.tibia)
        theta3 = np.arccos(max(-1.0, min(1.0, d)))
        theta2 = np.arctan2(z, g) - np.arctan2(self.tibia * np.sin(theta3), self.femur + self.tibia * np.cos(theta3))
        return theta1, theta2, theta3

    def _rotation_matrix(self, roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                [cp * cy, -cp * sy, sp],
                [sr * sp * cy + sy * cr, -sr * sp * sy + cr * cy, -sr * cp],
                [sr * sy - sp * cr * cy, sr * cy + sp * sy * cr, cr * cp],
            ]
        )


# 源 URDF 顺序（FL, FR, RL, RR）
URDF_JOINT_ORDER = [
    "motor_front_left_shoulder",
    "motor_front_left_leg",
    "foot_motor_front_left",
    "motor_front_right_shoulder",
    "motor_front_right_leg",
    "foot_motor_front_right",
    "motor_rear_left_shoulder",
    "motor_rear_left_leg",
    "foot_motor_rear_left",
    "motor_rear_right_shoulder",
    "motor_rear_right_leg",
    "foot_motor_rear_right",
]

URDF_TO_LOCAL_JOINT = {
    "motor_front_left_shoulder": "left_front_hip_joint",
    "motor_front_left_leg": "left_front_knee_joint",
    "foot_motor_front_left": "left_front_ankle_joint",
    "motor_front_right_shoulder": "right_front_hip_joint",
    "motor_front_right_leg": "right_front_knee_joint",
    "foot_motor_front_right": "right_front_ankle_joint",
    "motor_rear_left_shoulder": "left_back_hip_joint",
    "motor_rear_left_leg": "left_back_knee_joint",
    "foot_motor_rear_left": "left_back_ankle_joint",
    "motor_rear_right_shoulder": "right_back_hip_joint",
    "motor_rear_right_leg": "right_back_knee_joint",
    "foot_motor_rear_right": "right_back_ankle_joint",
}



JOINT_ORDER = [URDF_TO_LOCAL_JOINT[name] for name in URDF_JOINT_ORDER]
LEG_JOINTS = {
    "left_front": ["left_front_hip_joint", "left_front_knee_joint", "left_front_ankle_joint"],
    "right_front": ["right_front_hip_joint", "right_front_knee_joint", "right_front_ankle_joint"],
    "left_back": ["left_back_hip_joint", "left_back_knee_joint", "left_back_ankle_joint"],
    "right_back": ["right_back_hip_joint", "right_back_knee_joint", "right_back_ankle_joint"],
}
LEG_INDEX = {"left_front": 0, "right_front": 1, "left_back": 2, "right_back": 3}



def joints_to_local_cmd(joints_12: np.ndarray) -> Dict[str, float]:
    cmd = joints_12
    return {name: float(cmd[i]) for i, name in enumerate(JOINT_ORDER)}


def mech_to_servo(hwi, joint_name: str, mech_angle: float) -> float:
    real_pose = hwi.real_pose[joint_name]
    sign = hwi.real_pose_signs.get(joint_name, 1.0)
    return (mech_angle - real_pose) * sign + hwi.init_pos[joint_name] + hwi.joints_offsets[joint_name]


def send_local_cmd_ordered(hwi, local_cmd: Dict[str, float]):
    ordered_ids = [hwi.joints[name] for name in JOINT_ORDER]
    ordered_positions = [mech_to_servo(hwi, name, local_cmd[name]) for name in JOINT_ORDER]

    hwi.io.write_goal_position(ordered_ids, ordered_positions)


def isolate_legs_cmd(local_cmd: Dict[str, float], active_leg: str, hold_pose: Dict[str, float]) -> Dict[str, float]:
    if active_leg == "all":
        return local_cmd
    out = local_cmd.copy()
    for leg_name, joints in LEG_JOINTS.items():
        if leg_name == active_leg:
            continue
        for joint_name in joints:
            out[joint_name] = hold_pose[joint_name]
    return out


def print_leg_phase(active_leg: str, gait: GaitController, gait_state: GaitState, body_state: BodyState):
    leg_names = ["left_front", "right_front", "left_back", "right_back"] if active_leg == "all" else [active_leg]
    stand_frac = gait_state["stand_frac"]
    offsets = gait_state["offset"]

    for leg_name in leg_names:
        idx = LEG_INDEX[leg_name]
        phase = (gait.phase + offsets[idx]) % 1.0
        phase_name = "STANCE" if phase < stand_frac else "SWING"
        foot = body_state["feet"][idx]
        home = body_state["default_feet"][idx]
        dx, dy, dz = foot - home
        print(f"[LEG] {leg_name:11s} phase={phase:.3f} {phase_name:6s} dxyz=({dx:+.4f},{dy:+.4f},{dz:+.4f})")


def setup_gui_sliders():
    import pybullet as p

    if p.getConnectionInfo()["isConnected"] == 0:
        p.connect(p.GUI)

    sliders = {
        # 与 Leika simulation/src/utils/gui.py 严格一致
        "x": p.addUserDebugParameter("x", -1, 1, 0),
        "y": p.addUserDebugParameter("y", 0, 1, 0.5),
        "z": p.addUserDebugParameter("z", -1, 1, 0),
        "yaw": p.addUserDebugParameter("yaw", -1, 1, 0),
        "pitch": p.addUserDebugParameter("pitch", -1, 1, 0),
        "roll": p.addUserDebugParameter("roll", -1, 1, 0),
        "pivot_x": p.addUserDebugParameter("pivot x", -1, 1, 0),
        "pivot_y": p.addUserDebugParameter("pivot y", -1, 1, 0),
        "pivot_z": p.addUserDebugParameter("pivot z", -1, 1, 0),
        "step_x": p.addUserDebugParameter("Step x", -1, 1, 0),
        "step_z": p.addUserDebugParameter("Step z", -1, 1, 0),
        "angle": p.addUserDebugParameter("Angle", -1, 1, 0),
        "step_height": p.addUserDebugParameter("Step height", 0, 1, 0.5),
        "step_depth": p.addUserDebugParameter("Step depth", 0, 0.01, 0.002),
        "stand_frac": p.addUserDebugParameter("Stand frac", 0, 1, 0.75),
    }
    return p, sliders


def update_state_from_gui(p, sliders, body_state: BodyState, gait_state: GaitState):
    # gait_state（与 gui.py::update_gait_state 对齐）
    gait_state["step_x"] = p.readUserDebugParameter(sliders["step_x"]) * KinConfig.max_step_length
    gait_state["step_z"] = p.readUserDebugParameter(sliders["step_z"]) * KinConfig.max_step_length
    gait_state["step_angle"] = p.readUserDebugParameter(sliders["angle"])
    gait_state["step_height"] = p.readUserDebugParameter(sliders["step_height"]) * KinConfig.max_step_height
    gait_state["step_depth"] = p.readUserDebugParameter(sliders["step_depth"])
    gait_state["stand_frac"] = p.readUserDebugParameter(sliders["stand_frac"])
    gait_state["offset"] = default_offset[GaitType.TROT]
    gait_state["gait_type"] = GaitType.TROT

    # body_state（与 gui.py::update_body_state 对齐）
    body_state["xm"] = p.readUserDebugParameter(sliders["x"]) * KinConfig.max_body_shift_x
    body_state["ym"] = p.readUserDebugParameter(sliders["y"]) * KinConfig.body_height_range + KinConfig.min_body_height
    body_state["zm"] = p.readUserDebugParameter(sliders["z"]) * KinConfig.max_body_shift_z
    body_state["omega"] = p.readUserDebugParameter(sliders["roll"]) * KinConfig.max_roll
    body_state["phi"] = p.readUserDebugParameter(sliders["pitch"]) * KinConfig.max_pitch
    body_state["psi"] = p.readUserDebugParameter(sliders["yaw"]) * KinConfig.max_pitch
    body_state["px"] = p.readUserDebugParameter(sliders["pivot_x"]) * KinConfig.max_body_shift_x
    body_state["py"] = p.readUserDebugParameter(sliders["pivot_y"]) * KinConfig.max_body_shift_x
    body_state["pz"] = p.readUserDebugParameter(sliders["pivot_z"]) * KinConfig.max_body_shift_z


def main():
    parser = argparse.ArgumentParser(description="Leika 同构真机开环控制")
    parser.add_argument("--usb-port", default="/dev/ttyUSB0")
    parser.add_argument("--duration", type=float, default=0.0, help="秒，0=持续运行")
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument(
        "--active-leg",
        choices=["all", "left_front", "right_front", "left_back", "right_back"],
        default="right_front",
        help="只下发指定腿，其余腿锁在 real_pose",
    )
    args = parser.parse_args()

    gait_type = GaitType.TROT
    standby = KinConfig.default_feet_positions.copy()


    body_state: BodyState = {
        "omega": 0.0,
        "phi": 0.0,
        "psi": 0.0,
        "xm": 0.0,
        "ym": KinConfig.default_body_height,
        "zm": 0.0,
        "px": 0.0,
        "py": 0.0,
        "pz": 0.0,
        "feet": standby.copy(),
        "default_feet": standby.copy(),
    }


    gait_state: GaitState = {
        "step_height": KinConfig.default_step_height,
        "step_x": 0.0,
        "step_z": 0.0,
        "step_angle": 0.0,
        "step_depth": KinConfig.default_step_depth,
        "stand_frac": default_stand_frac[gait_type],
        "offset": default_offset[gait_type],
        "gait_type": gait_type,
    }

    gait = GaitController(standby)
    ik = Kinematics()
    hwi = HWI(usb_port=args.usb_port)

    hwi.turn_on()
    pb, sliders = setup_gui_sliders()#####don care


    joints = ik.inverse_kinematics(body_state)#####逆运动学
    target_cmd = joints_to_local_cmd(joints)####指令转换诚意正负号


    start_t = time.time()
    next_t = start_t


    try:
        while True:
            now = time.time()
            if args.duration > 0.0 and now - start_t >= args.duration:
                break

            update_state_from_gui(pb, sliders, body_state, gait_state)

            keys = pb.getKeyboardEvents()
            
            if keys.get(ord("q")) or keys.get(27):
                print("[INFO] GUI quit key pressed")
                break

            gait.step(gait_state, body_state, args.dt)
            print_leg_phase(args.active_leg, gait, gait_state, body_state)


            joints = ik.inverse_kinematics(body_state)
            local_cmd = joints_to_local_cmd(joints)
            local_cmd = isolate_legs_cmd(local_cmd, args.active_leg, hwi.real_pose)
            send_local_cmd_ordered(hwi, local_cmd)


            next_t += args.dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] stopping...")
        try:
            hwi.set_position_all(hwi.init_pos)
            time.sleep(0.6)
        except Exception:
            pass
        try:
            pb.disconnect()
        except Exception:
            pass
        hwi.turn_off()
        print("[INFO] torque disabled")


if __name__ == "__main__":
    main()
