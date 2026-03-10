import argparse
import os
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from runtime.position_hwi import HWI


def parse_args():
    parser = argparse.ArgumentParser(
        description="12-servo robustness test: concurrent control + position/velocity sampling"
    )
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--duration", type=float, default=20.0, help="seconds")
    parser.add_argument("--control-freq", type=float, default=50.0, help="Hz")
    parser.add_argument("--wave-freq", type=float, default=0.6, help="Hz")
    parser.add_argument("--amplitude", type=float, default=0.22, help="radians")
    parser.add_argument("--warmup", type=float, default=1.5, help="seconds")
    parser.add_argument("--kp", type=int, default=24)
    return parser.parse_args()


def pct(arr, p):
    if len(arr) == 0:
        return float("nan")
    return float(np.percentile(np.array(arr), p))


def mean(arr):
    if len(arr) == 0:
        return float("nan")
    return float(np.mean(np.array(arr)))


def run_test(args):
    hwi = HWI(usb_port=args.port)
    joint_names = list(hwi.joints.keys())
    joint_count = len(joint_names)

    print(f"[INFO] Connected. joints={joint_count}, port={args.port}")
    print("[INFO] Turning on motors...")
    hwi.turn_on()##


    base = np.array([hwi.init_pos[name] for name in joint_names], dtype=np.float64)
    phase = np.linspace(0.0, 2.0 * np.pi, joint_count, endpoint=False)

    period = 1.0 / args.control_freq
    end_time = time.monotonic() + args.duration

    ctrl_lat_ms = []
    read_lat_ms = []
    loop_lat_ms = []
    pos_err_abs = []
    vel_abs = []
    overruns = 0
    read_fail = 0
    bad_shape = 0
    loops = 0

    print(f"[INFO] Warmup {args.warmup:.1f}s...")
    t_warm = time.monotonic() + args.warmup
    while time.monotonic() < t_warm:
        hwi.set_position_all(hwi.init_pos)
        time.sleep(0.02)

    print("[INFO] Start stress test...")
    t0 = time.monotonic()

    try:
        while time.monotonic() < end_time:
            loop_start = time.monotonic()
            t = loop_start - t0

            targets = base + args.amplitude * np.sin(2.0 * np.pi * args.wave_freq * t + phase)
            target_dict = {name: float(targets[i]) for i, name in enumerate(joint_names)}

            c0 = time.monotonic()
            hwi.set_position_all(target_dict)
            c1 = time.monotonic()

            r0 = time.monotonic()
            pos = hwi.get_present_positions()
            vel = hwi.get_present_velocities()
            r1 = time.monotonic()

            ctrl_lat_ms.append((c1 - c0) * 1000.0)
            read_lat_ms.append((r1 - r0) * 1000.0)

            if pos is None or vel is None:
                read_fail += 1
            else:
                if len(pos) != joint_count or len(vel) != joint_count:
                    bad_shape += 1
                else:
                    err = np.abs(pos - targets)
                    pos_err_abs.extend(err.tolist())
                    vel_abs.extend(np.abs(vel).tolist())

            loops += 1
            took = time.monotonic() - loop_start
            loop_lat_ms.append(took * 1000.0)
            sleep_t = period - took
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                overruns += 1

    except KeyboardInterrupt:
        print("\n[WARN] Interrupted by user.")

    finally:
        print("[INFO] Returning to init pose and turning off...")
        try:
            hwi.set_position_all(hwi.init_pos)
            time.sleep(0.6)
        finally:
            hwi.turn_off()

    effective_secs = args.duration
    achieved_hz = loops / effective_secs if effective_secs > 0 else float("nan")
    good_reads = max(0, loops - read_fail - bad_shape)

    print("\n========== 12-Servo Robustness Report ==========")
    print(f"loops: {loops}")
    print(f"target duration: {args.duration:.2f}s")
    print(f"target freq: {args.control_freq:.2f} Hz")
    print(f"achieved freq: {achieved_hz:.2f} Hz")
    print(f"overruns: {overruns} ({(overruns / loops * 100.0) if loops else 0:.2f}%)")
    print(f"read fail: {read_fail}")
    print(f"bad shape: {bad_shape}")
    print(f"good read loops: {good_reads}")

    print("\n[Control write latency ms]")
    print(
        f"mean={mean(ctrl_lat_ms):.3f}, p95={pct(ctrl_lat_ms, 95):.3f}, max={max(ctrl_lat_ms) if ctrl_lat_ms else float('nan'):.3f}"
    )

    print("\n[Read latency ms]")
    print(
        f"mean={mean(read_lat_ms):.3f}, p95={pct(read_lat_ms, 95):.3f}, max={max(read_lat_ms) if read_lat_ms else float('nan'):.3f}"
    )

    print("\n[Full loop latency ms]")
    print(
        f"mean={mean(loop_lat_ms):.3f}, p95={pct(loop_lat_ms, 95):.3f}, max={max(loop_lat_ms) if loop_lat_ms else float('nan'):.3f}"
    )

    print("\n[Position tracking abs error rad]")
    print(
        f"mean={mean(pos_err_abs):.4f}, p95={pct(pos_err_abs, 95):.4f}, max={max(pos_err_abs) if pos_err_abs else float('nan'):.4f}"
    )

    print("\n[Velocity abs rad/s]")
    print(
        f"mean={mean(vel_abs):.4f}, p95={pct(vel_abs, 95):.4f}, max={max(vel_abs) if vel_abs else float('nan'):.4f}"
    )
    print("================================================")


if __name__ == "__main__":
    args = parse_args()
    run_test(args)
