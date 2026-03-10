import time
import numpy as np
from typing import List, Dict, Optional
# ==============================================
# 核心：跨文件导入你写的舵机驱动（已加自动轮询）
# ==============================================
from commu import ZLServoController


# ==============================================
# 机械狗抽象层：封装12个舵机的批量操作
# 你的RL代码只和这个类打交道，不用管底层驱动细节
# ==============================================
class HWI:
    def __init__(self, serial_port='/dev/ttyTHS1', baudrate=115200):
        # 1. 实例化底层舵机驱动（全局唯一实例，禁止重复创建）
        self.servo_driver = ZLServoController(port=serial_port, baudrate=baudrate)
        
        # ====================== 关键修正：SERVO_ID_LIST严格对齐你确认的joints字典ID顺序 ======================
        # 顺序：左前腿(髋/膝/踝) → 左后腿(髋/膝/踝) → 右前腿(髋/膝/踝) → 右后腿(髋/膝/踝)
        # 完全匹配你确认的joints字典ID映射，确保批量操作的ID和实际接线一致
        self.SERVO_ID_LIST = [
            0, 1, 2,   
            3, 4, 5,   
            6, 7, 8,  
            9, 10, 11  
        ]
        # 你确认的正确ID映射（完全保留，不做任何修改）
        self.joints = {  
            "left_front_hip_joint": 0,
            "left_front_knee_joint": 1,
            "left_front_ankle_joint": 2,
            "left_back_hip_joint": 3,
            "left_back_knee_joint": 4,
            "left_back_ankle_joint": 5,
            "right_front_hip_joint": 6,
            "right_front_knee_joint": 7,
            "right_front_ankle_joint": 8,
            "right_back_hip_joint": 9, 
            "right_back_knee_joint": 10,
            "right_back_ankle_joint": 11,
        }
        # ==================================================================================================

        # 初始位置（按你确认的关节名+ID映射，值可根据实际站立位调整）
        self.init_pos = {
            "left_front_hip_joint": 135.0,
            "left_front_knee_joint": 135.0,
            "left_front_ankle_joint": 135.0,
            "left_back_hip_joint": 135.0,
            "left_back_knee_joint": 135.0,
            "left_back_ankle_joint": 135.0,
            "right_front_hip_joint": 135.0,
            "right_front_knee_joint": 135.0,
            "right_front_ankle_joint": 135.0,
            "right_back_hip_joint": 135.0, 
            "right_back_knee_joint": 135.0,
            "right_back_ankle_joint": 135.0,
        } 

        # 理论零位（仿真基准）+ 实物偏移校准（关键：适配RL仿真→实物）
        self.zero_pos = {k: 0.0 for k in self.joints.keys()}  # 仿真零位
        self.angle_offset = {k: 135.0 for k in self.joints.keys()}  # 实物零位偏移（135°）

        self.SERVO_COUNT = len(self.SERVO_ID_LIST)  # 固定12个

        # 3. 硬件安全限制
        self.ANGLE_MIN = 0.0    # 舵机最小角度
        self.ANGLE_MAX = 270.0  # 舵机最大角度
        self.CONTROL_FREQ = 50  # 控制频率50Hz（对应20ms周期，适配总线舵机性能）
        self.CONTROL_DT = 1.0 / self.CONTROL_FREQ

        # 4. 状态变量
        self.connected = False
        self.current_angles = np.full(self.SERVO_COUNT, 135.0)  # 初始全部为135度中位
        self.last_control_time = 0.0  # 记录最后一次控制指令发送时间，防高频堆积

    def init_hardware(self):
        """初始化硬件：连接串口+启动接收线程+启动自动轮询+舵机使能+转到初始位"""
        # 防重复初始化（RL多次调用时容错）
        if self.connected:
            print("⚠️  硬件已初始化，无需重复操作")
            return True
        
        if not self.servo_driver.connect():
            print("❌ 舵机驱动初始化失败")
            return False

        self.connected = True

        # 启动50Hz自动轮询，底层持续给12个舵机发查询指令，无需手动刷新
        self.servo_driver.start_auto_query(servo_ids=self.SERVO_ID_LIST)
        time.sleep(0.1)

        # 【新增】所有舵机使能
        self.servo_enable_all()

        print("✅ 机械狗硬件初始化完成，12路舵机已就绪（自动轮询已启动，已转到初始位）")
        return True


    def deinit_hardware(self):
        """安全释放硬件：停止线程+关闭串口+舵机失能"""
        if not self.connected:
            return
        # 【新增】紧急失能所有舵机，防止断电损坏
        self.servo_break_all()
        self.servo_driver.disconnect()  # disconnect内部会自动停止自动轮询
        self.connected = False
        print("🛑 机械狗硬件已安全释放（所有舵机已失能）")


    def get_joint_angle(self, joint_name: str) -> Optional[float]:
        """【RL调试必备】单独获取某个关节的实时角度"""
        if joint_name not in self.joints:
            print(f"❌ 关节名错误：{joint_name}，可选值：{list(self.joints.keys())}")
            return None
        servo_id = self.joints[joint_name]
        angle = self.servo_driver.get_servo_pos(servo_id)
        if angle is None:
            print(f"⚠️  关节{joint_name}（ID{servo_id}）无数据，返回初始值{self.init_pos[joint_name]}")
            return self.init_pos[joint_name]
        return angle

    def set_joint_angle(self, joint_name: str, angle: float, duration_ms=20):
        """【RL调试必备】单独设置某个关节的角度"""
        if joint_name not in self.joints:
            print(f"❌ 关节名错误：{joint_name}")
            return
        if not self.connected:
            print("⚠️  硬件未连接，无法设置关节角度")
            return
        # 安全限幅
        angle = np.clip(angle, self.ANGLE_MIN, self.ANGLE_MAX)
        servo_id = self.joints[joint_name]
        self.servo_driver.set_angle(servo_id=servo_id, angle=angle, s_time=duration_ms)
        # print(f"📤 设置关节{joint_name}（ID{servo_id}）角度为{angle}°")

    def get_all_angles(self) -> np.ndarray:
        """【RL观测接口】读取12个舵机的实时角度，返回shape=(12,)的数组
        关键：适配RL仿真→实物，返回「仿真坐标系角度」（减去偏移）
        """
        if not self.connected:
            return self.current_angles
        
        for idx, servo_id in enumerate(self.SERVO_ID_LIST):
            angle = self.servo_driver.get_servo_pos(servo_id)
            if angle is not None:
                self.current_angles[idx] = angle  # 实物角度
        return self.current_angles  


    # def set_all_angles(self, target_angles: np.ndarray, duration_ms=20):
    #     """【RL执行接口】批量设置12个舵机的目标角度
    #     target_angles: shape=(12,)的数组，对应12个舵机的目标角度（单位：度）
    #     duration_ms: 舵机转动时间，RL高频控制建议设为10-20ms
    #     关键：防高频指令堆积（50Hz控制下，仅当间隔≥CONTROL_DT时发送）
    #     """
    #     if not self.connected:
    #         print("⚠️  硬件未连接，无法发送角度指令")
    #         return
        
    #     # 防高频指令堆积（RL 50Hz调用时，确保指令间隔≥20ms）
    #     current_time = time.time()
    #     if current_time - self.last_control_time < self.CONTROL_DT:
    #         return  # 跳过本次，防止串口拥堵
    #     self.last_control_time = current_time

    #     # 安全限幅，防止超出机械极限损坏舵机
    #     target_angles = np.clip(target_angles, self.ANGLE_MIN, self.ANGLE_MAX)
        
    #     # 批量给12个舵机发送角度控制指令（严格按你确认的ID顺序）
    #     for idx, servo_id in enumerate(self.SERVO_ID_LIST):
    #         target_angle = target_angles[idx]
    #         self.servo_driver.set_angle(
    #             servo_id=servo_id,
    #             angle=target_angle,
    #             s_time=duration_ms
    #         )

    def set_all_angles(self, target_angles: np.ndarray, duration_ms=20):
        """【RL执行接口】批量设置12个舵机的目标角度（叠加指令版）
        target_angles: shape=(12,)的数组，对应12个舵机的目标角度（单位：度）
        duration_ms: 舵机转动时间，RL高频控制建议设为10-20ms
        协议格式：{G0000#ID1PxxxxTxxxx!#ID2PxxxxTxxxx!#...!}
        """
        if not self.connected:
            print("⚠️  硬件未连接，无法发送角度指令")
            return
        
        # 防高频指令堆积（RL 50Hz控制下，确保指令间隔≥20ms）
        current_time = time.time()
        if current_time - self.last_control_time < self.CONTROL_DT:
            return  # 跳过本次，防止串口拥堵
        self.last_control_time = current_time

        # 安全限幅，防止超出机械极限损坏舵机
        target_angles = np.clip(target_angles, self.ANGLE_MIN, self.ANGLE_MAX)
        
        # ====================== 核心：叠加指令生成 ======================
        # 1. 叠加指令固定开头
        multi_cmd = "{G0000"
        
        # 2. 遍历12个舵机，拼接每个舵机的控制指令
        for idx, servo_id in enumerate(self.SERVO_ID_LIST):
            target_angle = target_angles[idx]
            
            # 角度转PWM（和底层驱动逻辑完全一致，保证精度）
            pwm_val = int(
                self.servo_driver.PWM_MIN 
                + (target_angle / self.servo_driver.MAX_ANGLE) 
                * (self.servo_driver.PWM_MAX - self.servo_driver.PWM_MIN)
            )
            # PWM安全限幅
            pwm_val = max(self.servo_driver.PWM_MIN, min(self.servo_driver.PWM_MAX, pwm_val))
            
            # 按照协议格式化：3位ID、4位PWM、4位时间，不足补0
            id_str = f"{servo_id:03d}"
            pwm_str = f"{pwm_val:04d}"
            time_str = f"{duration_ms:04d}"
            
            # 拼接单条舵机指令，追加到叠加指令中
            single_cmd = f"#{id_str}P{pwm_str}T{time_str}!"
            multi_cmd += single_cmd
        
        # 3. 叠加指令固定结尾
        multi_cmd += "}"
        # ==================================================================

        # 一次性发送完整的叠加指令（只发1次串口，无拥堵）
        self.servo_driver._send_cmd(multi_cmd)


    def servo_break_all(self):
        """紧急停止：所有舵机失能，释放扭力"""
        if not self.connected:
            return
        for servo_id in self.SERVO_ID_LIST:
            self.servo_driver.disable_servo(servo_id)
        print("⚠️  所有舵机已失能（紧急停止）")

    def servo_enable_all(self):
        """所有舵机使能，恢复扭力"""
        if not self.connected:
            return
        for servo_id in self.SERVO_ID_LIST:
            self.servo_driver.enable_servo(servo_id)
        print("✅ 所有舵机已使能（恢复扭力）")


if __name__ == "__main__":
    # 1. 初始化硬件
    hwi = HWI(serial_port='/dev/ttyTHS1', baudrate=115200)

    if not hwi.init_hardware():
        print("❌ 硬件初始化失败")
        exit(1)

    print("\n" + "="*80)
    print("       🤖 12路舵机联合测试 (按 Ctrl+C 随时停止)")
    print("="*80)
    print("💡 测试流程：")
    print("  1. 所有舵机回到中位 (135°)")
    print("  2. 第一波：所有舵机转到 115° (低头)")
    print("  3. 第二波：所有舵机转到 155° (抬头)")
    print("  4. 循环摆动，同时实时显示角度")
    print("="*80 + "\n")

    # 初始化目标角度数组
    target_angles = np.full(12, 135.0)
    joint_names = list(hwi.joints.keys())

    try:
        # ==================== 阶段1：回到中位 ====================
        print("⏳ 阶段1/4：所有舵机回到中位...")
        hwi.set_all_angles(target_angles, duration_ms=1000)
        time.sleep(1.5)
        
        print("✅ 准备就绪！开始摆动测试...\n")
        time.sleep(0.5)

        # ==================== 阶段2：循环摆动测试 ====================
        print("🚀 开始循环摆动 (Ctrl+C 停止)...")
        
        loop_count = 0
        wave_state = 0 # 0: 去115°, 1: 去155°
        
        while True:
            loop_count += 1
            
            # 切换目标角度
            if wave_state == 0:
                target_angles[:] = 115.0 # 所有舵机 115°
                state_str = "↓ 低位"
            else:
                target_angles[:] = 155.0 # 所有舵机 155°
                state_str = "↑ 高位"
            
            # 发送控制指令
            hwi.set_all_angles(target_angles, duration_ms=800)
            
            # 等待并实时显示数据
            wait_start = time.time()
            while time.time() - wait_start < 2.0:
                current_angles = hwi.get_all_angles()
                
                # 清屏打印 (可选，为了美观)
                print("\033c", end="")
                
                print("="*80)
                print(f"       🤖 12路舵机联合测试 | 循环次数: {loop_count} | 当前状态: {state_str}")
                print("="*80)
                print(f"{'ID':<4} {'关节名称':<25} {'当前角度(°)':<15} {'目标角度(°)':<15}")
                print("-" * 70)
                
                for idx, servo_id in enumerate(hwi.SERVO_ID_LIST):
                    curr = current_angles[idx]
                    targ = target_angles[idx]
                    name = joint_names[idx]
                    
                    # 如果接近目标，显示绿色，否则显示黄色
                    status = "✅" if abs(curr - targ) < 3.0 else "🔄"
                    
                    print(f"{servo_id:<4} {name:<25} {curr:<15.1f} {targ:<15.1f} {status}")
                
                print("-" * 70)
                print("💡 按 Ctrl+C 停止测试")
                
                time.sleep(0.1)
            
            # 切换状态
            wave_state = 1 - wave_state

    except KeyboardInterrupt:
        print("\n\n🛑 用户停止测试")
        
    finally:
        # ==================== 阶段3：安全回中位 ====================
        print("\n🏠 紧急回中位...")
        target_home = np.full(12, 135.0)
        hwi.set_all_angles(target_home, duration_ms=1000)
        time.sleep(1.2)
        
        hwi.deinit_hardware()
        print("\n✅ 测试结束，所有舵机已安全失能")