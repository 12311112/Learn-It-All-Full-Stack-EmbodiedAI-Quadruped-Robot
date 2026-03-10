# set_servo_id.py
# 专门用于 修改舵机ID + 测试转动 纯同步版本，无任何线程干扰
# ============================================================
import serial
import time
import subprocess

class ZLServoIDSetter:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def _set_permissions(self):
        try:
            subprocess.run(['sudo', 'chmod', '777', self.port], check=True)
            print("✅ 串口权限 OK")
        except:
            pass

    def connect(self):
        self._set_permissions()
        time.sleep(0.1)
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print("✅ 串口连接成功")
            return True
        except Exception as e:
            print(f"❌ 串口失败: {e}")
            return False

    def disconnect(self):
        if self.ser:
            self.ser.close()

    def set_id(self, old_id, new_id):
        old_str = f"{old_id:03d}"
        new_str = f"{new_id:03d}"
        cmd = f"#{old_str}PID{new_str}!"
        success_ack = f"#{new_str}P!"

        # 清空缓存
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.05)

        print(f"📤 发送: {cmd}")
        self.ser.write(cmd.encode())
        time.sleep(0.1)

        # 读响应
        if self.ser.in_waiting > 0:
            recv = self.ser.read_all().decode('ascii', errors='ignore').strip()
            print(f"📥 响应: {recv}")
            if success_ack in recv:
                print("✅ ID 修改成功！")
                return True
        print("❌ 修改失败")
        return False

    def test_move(self, servo_id):
        """测试转动：使能 → 转150° → 回中位"""
        print(f"\n🔧 测试新ID {servo_id} 转动...")

        # 使能
        en_cmd = f"#{servo_id:03d}PULR!"
        self.ser.write(en_cmd.encode())
        time.sleep(0.2)

        # 转 150°
        pwm = 1555
        move_cmd = f"#{servo_id:03d}P{pwm:04d}T0500!"
        self.ser.write(move_cmd.encode())
        time.sleep(0.7)

        # 回中位 135°
        pwm_mid = 1500
        mid_cmd = f"#{servo_id:03d}P{pwm_mid:04d}T0500!"
        self.ser.write(mid_cmd.encode())
        time.sleep(0.7)

        print("✅ 测试转动完成！")


# ==================== 主程序 ====================
if __name__ == "__main__":
    # ========== 在这里改 ID ==========
    OLD_ID = 12   # 原来的ID（新舵机一定是0）
    NEW_ID = 11    # 你要设置的新ID
    # ================================

    setter = ZLServoIDSetter()
    if not setter.connect():
        exit()

    print("\n" + "="*50)
    print("        舵机ID修改工具（纯同步无线程）")
    print("="*50)
    print("⚠️  务必只接一个舵机！")
    time.sleep(2)

    # 1. 修改ID
    ok = setter.set_id(OLD_ID, NEW_ID)
    if not ok:
        setter.disconnect()
        exit()

    time.sleep(0.5)

    # 2. 测试转动
    setter.test_move(NEW_ID)

    print("\n🎉 全部完成！")
    setter.disconnect()