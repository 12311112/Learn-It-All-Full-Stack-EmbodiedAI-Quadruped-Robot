"""
dog_Aston — 四足机器人控制系统

入口文件。提供舵机零位校准和姿态管理功能。
"""

import sys
from tools.calibrate import run_calibration
from tools.pose_manager import run_pose_manager


def main():
    print()
    print("========================================")
    print("  dog_Aston 四足机器人控制系统")
    print("========================================")
    print()
    print("  1. 舵机零位校准")
    print("  2. 姿态管理")
    print("  0. 退出")
    print()

    choice = input("请选择功能: ").strip()

    if choice == "1":
        run_calibration()
    elif choice == "2":
        run_pose_manager()
    elif choice == "0":
        print("再见!")
    else:
        print("无效选择。")


if __name__ == "__main__":
    main()
