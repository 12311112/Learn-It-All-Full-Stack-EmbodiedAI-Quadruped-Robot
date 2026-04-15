import time
import pickle
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI
from runtime.onnx_infer import OnnxInfer
from runtime.raw_imu import Imu
from runtime.xbox import XBoxController
from runtime.rl_utils import make_action_dict, LowPassActionFilter

HOME_DIR = os.path.expanduser("~")


class RLWalk:
    def __init__(
        self,
        onnx_model_path: str,
        serial_port: str = "/dev/ttyUSB0",
        control_freq: float = 50,
        pid=[30, 0, 0],
        action_scale=0.25,
        commands=False,
        pitch_bias=0,
        cutoff_frequency=None,
    ):

        self.commands = commands
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 12
        self.max_motor_velocity = 5.24  # rad/s

        # Control
        self.control_freq = control_freq
        self.pid = pid


        self.action_filter = None
        if cutoff_frequency is not None:
            self.action_filter = LowPassActionFilter(
                self.control_freq, cutoff_frequency
            )

        self.hwi = HWI(serial_port)

        self.start()

        self.imu = Imu(
            sampling_freq=int(self.control_freq),
            user_pitch_bias=self.pitch_bias,
            upside_down=False,
        )

        # Scales
        self.action_scale = action_scale

        self.last_action = np.zeros(self.num_dofs)
        self.last_last_action = np.zeros(self.num_dofs)
        self.last_last_last_action = np.zeros(self.num_dofs)

        self.init_pos = list(self.hwi.init_pos.values())
        self.joint_signs = list(self.hwi.real_pose_signs_rl.values()) 

        self.motor_targets = np.array(self.init_pos.copy())

        # [lin_vel_x, lin_vel_y, yaw_rate]
        self.last_commands = [0.0, 0.0, 0.0]

        self.paused = False        
        self.command_freq = 20  # hz
        if self.commands:
            self.xbox_controller = XBoxController(self.command_freq)


    def get_obs(self):

        imu_data = self.imu.get_data()

        dof_pos = self.hwi.get_present_positions()  # rad  ####得到的顺序 顺序对 但是符号不一定对 减去一个init 然后乘以负号就行了

        dof_vel = self.hwi.get_present_velocities()  # rad/s#####  乘以负号就行

        if dof_pos is None or dof_vel is None:
            return None

        if len(dof_pos) != self.num_dofs:
            print(f"ERROR len(dof_pos) != {self.num_dofs}")
            return None

        if len(dof_vel) != self.num_dofs:
            print(f"ERROR len(dof_vel) != {self.num_dofs}")
            return None

        cmds = np.asarray(self.last_commands, dtype=float)[:3]
        dof_pos_rel = (dof_pos - self.init_pos) * self.joint_signs ##############没问题
        dof_vel_rl = dof_vel * self.joint_signs

        obs = np.concatenate(
            [
                imu_data["gyro"],
                imu_data["accelero"],
                cmds,
                dof_pos_rel,
                dof_vel_rl * 0.05,
                self.last_action,
                self.last_last_action,
                self.last_last_last_action,
            ]
        )
        return obs

    def start(self):
        # 四足 12 关节，别再写死 14
        n = len(self.hwi.joints)

        kp = float(self.pid[0])
        kd = float(self.pid[2])

        # 先给统一增益；HWI.turn_on() 内部会低刚度上电 -> 到 init_pos -> 切到这里设置的 kp
        kps = [kp] * n
        kds = [kd] * n

        self.hwi.set_kps(kps)#####
        self.hwi.set_kds(kds)
        self.hwi.turn_on()

        # 给机械体到位和总线稳定时间
        time.sleep(1.0)



    def run(self):
        i = 0
        try:
            print("Starting")
            start_t = time.time()
            while True:
                t = time.time()

                if self.commands:
                    self.last_commands, self.buttons, _, _ = (
                        self.xbox_controller.get_last_command()
                    )
                    if self.buttons.A.triggered:
                        self.paused = not self.paused
                        if self.paused:
                            print("PAUSE")
                        else:
                            print("UNPAUSE")

                if self.paused:
                    time.sleep(0.1)
                    continue

                obs = self.get_obs()
                if obs is None:
                    continue

                action = np.asarray(self.policy.infer(obs), dtype=float)####这里的顺序是
                self.last_last_last_action = self.last_last_action.copy()
                self.last_last_action = self.last_action.copy()
                self.last_action = action.copy()

                self.motor_targets = (
                    self.init_pos + action * self.action_scale*self.joint_signs    ###########没有乘以数值   init不对！！！！！！
                )

                if self.action_filter is not None:
                    self.action_filter.push(self.motor_targets)
                    filtered_motor_targets = self.action_filter.get_filtered_action()
                    if (
                        time.time() - start_t > 1
                    ):  # give time to the filter to stabilize
                        self.motor_targets = filtered_motor_targets

                action_dict = make_action_dict(
                    self.motor_targets, list(self.hwi.joints.keys())
                )

                self.hwi.set_position_all(action_dict)
                i += 1
                took = time.time() - t
                # print("Full loop took", took, "fps : ", np.around(1 / took, 2))
                if (1 / self.control_freq - took) < 0:
                    print(
                        "Policy control budget exceeded by",
                        np.around(took - 1 / self.control_freq, 3),
                    )
                time.sleep(max(0, 1 / self.control_freq - took))

        except KeyboardInterrupt:
            pass
        finally:
            print("TURNING OFF")
            try:
                if self.commands and hasattr(self, "xbox_controller"):
                    self.xbox_controller.close()
            except Exception as e:
                print("Failed to close controller:", e)

            try:
                self.hwi.turn_off()
            except Exception as e:
                print("Failed to turn off motors:", e)



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, default="/home/jetson/Desktop/dog/dog_feetch/TEST.onnx")
    parser.add_argument("-a", "--action_scale", type=float, default=0.25)
    parser.add_argument("-p", type=int, default=30)
    parser.add_argument("-i", type=int, default=0)
    parser.add_argument("-d", type=int, default=0)
    parser.add_argument("-c", "--control_freq", type=int, default=50)
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument(
        "--commands",
        action="store_true",
        default=True,
        help="external commands, keyboard or gamepad. Launch control_server.py on host computer",
    )
    parser.add_argument("--cutoff_frequency", type=float, default=None)

    args = parser.parse_args()
    pid = [args.p, args.i, args.d]

    print("Done parsing args")
    rl_walk = RLWalk(
        args.onnx_model_path,
        action_scale=args.action_scale,
        pid=pid,
        control_freq=args.control_freq,
        commands=args.commands,
        pitch_bias=args.pitch_bias,
        cutoff_frequency=args.cutoff_frequency,
    )
    print("Done instantiating RLWalk")
    rl_walk.run()
