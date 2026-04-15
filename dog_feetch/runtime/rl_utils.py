# This is the joints order when loading using IsaacGymEnvs
# ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'left_antenna', 'right_antenna', 'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle']
# This is the "standard" order (from mujoco)
# ['right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle', 'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'left_antenna', 'right_antenna']
#
# We need to reorder the joints to match the IsaacGymEnvs order
#
import numpy as np

mujoco_joints_order = [
    "right_front_hip_joint",
    "right_front_knee_joint",
    "right_front_ankle_joint",
    "left_front_hip_joint",
    "left_front_knee_joint",
    "left_front_ankle_joint",
    "right_back_hip_joint",
    "right_back_knee_joint",
    "right_back_ankle_joint",
    "left_back_hip_joint",
    "left_back_knee_joint",
    "left_back_ankle_joint",  
]
def action_to_pd_targets(action, offset, scale):
    return offset + scale * action

def make_action_dict(action, joints_order):
    action_dict = {}
    for i, a in enumerate(action):
        if "antenna" not in joints_order[i]:
            action_dict[joints_order[i]] = a
    
    return action_dict


def quat_rotate_inverse(q, v):
    q = np.array(q)
    v = np.array(v)

    q_w = q[-1]
    q_vec = q[:3]

    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0

    return a - b + c


class ActionFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.action_buffer = []

    def push(self, action):
        self.action_buffer.append(action)
        if len(self.action_buffer) > self.window_size:
            self.action_buffer.pop(0)

    def get_filtered_action(self):
        return np.mean(self.action_buffer, axis=0)


class LowPassActionFilter:
    def __init__(self, control_freq, cutoff_frequency=30.0):
        self.last_action = 0
        self.current_action = 0
        self.control_freq = float(control_freq)
        self.cutoff_frequency = float(cutoff_frequency)
        self.alpha = self.compute_alpha()

    def compute_alpha(self):
        return (1.0 / self.cutoff_frequency) / (
            1.0 / self.control_freq + 1.0 / self.cutoff_frequency
        )

    def push(self, action):
        self.current_action = action

    def get_filtered_action(self):
        self.last_action = (
            self.alpha * self.last_action + (1 - self.alpha) * self.current_action
        )
        return self.last_action