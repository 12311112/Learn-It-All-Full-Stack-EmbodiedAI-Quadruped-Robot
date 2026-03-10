import time
import json
from commu import ZLServoController  # 确保这是你提供的那个驱动文件

# ==============================================
# 12个舵机的ID+关节名称映射
# ==============================================
SERVO_CALIBRATION_LIST = [
    {"id": 0, "name": "左前腿-髋关节 (left_front_hip)"},
    {"id": 1, "name": "左前腿-膝关节 (left_front_knee)"},
    {"id": 2, "name": "左前腿-踝关节 (left_front_ankle)"},
    {"id": 3, "name": "左后腿-髋关节 (left_back_hip)"},
    {"id": 4, "name": "左后腿-膝关节 (left_back_knee)"},
    {"id": 5, "name": "左后腿-踝关节 (left_back_ankle)"},
    {"id": 6, "name": "右前腿-髋关节 (right_front_hip)"},
    {"id": 7, "name": "右前腿-膝关节 (right_front_knee)"},
    {"id": 8, "name": "右前腿-踝关节 (right_front_ankle)"},
    {"id": 9, "name": "右后腿-髋关节 (right_back_hip)"},
    {"id": 10, "name": "右后腿-膝关节 (right_back_knee)"},
    {"id": 11, "name": "右后腿-踝关节 (right_back_ankle)"},
]

SERIAL_PORT = "/dev/ttyTHS1"
BAUDRATE = 115200
CALIBRATION_LOG_FILE = "servo_calibration_log.json"

class ServoHardwareCalibrator:
    def __init__(self):
        self.driver = ZLServoController(port=SERIAL_PORT, baudrate=BAUDRATE)
        self.calibration_log = []

    def init_hardware(self):
        """初始化：连接串口、启动轮询、使能所有舵机"""
        if not self.driver.connect():
            print("❌ 串口连接失败")
            return False
        
        # 启动自动轮询，保证数据能读到
        self.driver.start_auto_query(servo_ids=[s["id"] for s in SERVO_CALIBRATION_LIST])
        time.sleep(1) # 等待数据稳定
        
        # 先使能所有舵机，防止狗散架
        print("🔒 正在使能所有舵机以保持姿态...")
        for servo in SERVO_CALIBRATION_LIST:
            self.driver.enable_servo(servo["id"])
            time.sleep(0.05)
        
        print("✅ 硬件初始化完成")
        return True

    def calibrate_single_servo(self, servo_id, servo_name):
        """
        核心校准逻辑：
        1. 失能 -> 2. 手动摆位 -> 3. 读数 -> 4. 写入中值(PSCK) -> 5. 设置启动位 -> 6. 使能
        """
        print(f"\n" + "="*60)
        print(f"🔧 校准关节：【{servo_name}】 (ID={servo_id})")
        print("="*60)

        # 1. 失能当前舵机
        self.driver.disable_servo(servo_id)
        time.sleep(0.3)
        print(f"1/6 ✅ 舵机已失能，扭力已释放")

        # 2. 等待用户手动摆到机械中位
        input(f"2/6 👉 请用手将【{servo_name}】扳到**机械零位/初始姿态**，\n      摆好姿势后，按回车键继续...")
        time.sleep(0.5) # 等待稳定

        # 3. 读取当前位置（作为原始参考）
        print(f"3/6 📡 正在读取当前位置...")
        self.driver.get_position(servo_id) # 强制触发一次查询
        time.sleep(0.3)
        
        original_pwm = self.driver.get_servo_pwm(servo_id)
        original_angle = self.driver.get_servo_pos(servo_id)
        
        if original_pwm is None:
            print("❌ 读取位置失败，跳过该舵机")
            self.driver.enable_servo(servo_id)
            return False
        
        print(f"   原始位置读取成功：PWM={original_pwm}, Angle={original_angle}°")

        # 4. 【核心指令】执行中值矫正 (PSCK)
        # 告诉舵机：“你现在的物理位置，就是新的1500中值”
        print(f"4/6 📝 正在写入舵机内部中值 (执行 PSCK 指令)...")
        self.driver.calibrate_mid_value(servo_id)
        time.sleep(0.5)

        # 5. 设置开机启动位置为1500
        # 这样下次上电，舵机就会自动转到这个位置
        print(f"5/6 📝 正在设置开机启动位 (1500)...")
        self.driver.set_start_position(servo_id, position=1500)
        time.sleep(0.3)

        # 6. 重新使能舵机，锁定位置
        print(f"6/6 🔒 正在使能舵机，锁定当前位置...")
        self.driver.enable_servo(servo_id)
        time.sleep(0.5)

        # 校验结果
        self.driver.get_position(servo_id)
        time.sleep(0.3)
        final_pwm = self.driver.get_servo_pwm(servo_id)
        final_angle = self.driver.get_servo_pos(servo_id)

        # 记录日志
        log_data = {
            "id": servo_id,
            "name": servo_name,
            "original_pwm_before_calib": original_pwm,
            "final_pwm_after_calib": final_pwm,
            "final_angle": final_angle
        }
        self.calibration_log.append(log_data)

        print(f"\n✅ 【{servo_name}】校准完成！")
        print(f"   校准后读取值: PWM={final_pwm} (目标: 1500), 角度={final_angle}°")
        
        if 1470 <= final_pwm <= 1530:
            print("   ✅ 校验通过，中值写入成功")
        else:
            print("   ⚠️  PWM偏差略大，建议重新运行校准程序检查")
        
        return True

    def run_full_calibration(self):
        print("\n" + "="*60)
        print("       📐 12路舵机硬件中值校准程序")
        print("="*60)
        print("⚠️  注意事项：")
        print("1. 请确保机械狗供电充足。")
        print("2. 校准过程中，只有当前操作的舵机会失能，其余保持锁定。")
        print("3. 校准成功后，该位置会被记录为舵机内部的1500中值。")
        print("="*60)
        
        input("👉 准备好了吗？按回车键开始逐个校准...")

        for servo in SERVO_CALIBRATION_LIST:
            self.calibrate_single_servo(servo["id"], servo["name"])
            time.sleep(0.5)

        self.save_and_finish()

    def save_and_finish(self):
        print("\n" + "="*60)
        print("       🎉 全部校准流程结束")
        print("="*60)
        
        # 打印摘要
        print("\n校准摘要：")
        for log in self.calibration_log:
            status = "✅" if 1470 <= log['final_pwm_after_calib'] <= 1530 else "⚠️"
            print(f"{status} ID={log['id']} {log['name']:15} | 最终PWM={log['final_pwm_after_calib']}")

        # 保存日志文件
        try:
            with open(CALIBRATION_LOG_FILE, 'w', encoding='utf-8') as f:
                json.dump(self.calibration_log, f, indent=4, ensure_ascii=False)
            print(f"\n✅ 详细日志已保存至: {CALIBRATION_LOG_FILE}")
        except Exception as e:
            print(f"⚠️ 保存日志失败: {e}")

        print("\n💡 下一步：")
        print("1. 可以断电重启舵机，测试上电是否自动回到校准后的中位。")
        print("2. 在你的RL代码中，控制目标角度 135° 即对应此中位。")

    def shutdown(self):
        self.driver.stop_auto_query()
        self.driver.disconnect()
        print("\n👋 程序已退出")

if __name__ == "__main__":
    calibrator = ServoHardwareCalibrator()
    if not calibrator.init_hardware():
        exit(1)

    try:
        calibrator.run_full_calibration()
    except KeyboardInterrupt:
        print("\n🛑 用户强制停止")
    finally:
        calibrator.shutdown()