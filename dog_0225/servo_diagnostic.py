#!/usr/bin/env python3
"""
舵机诊断脚本 - 用于排查12个舵机突然停止的问题
"""
import time
import sys

# 添加项目路径
sys.path.insert(0, '/home/jetson/Desktop/dog/dog_0225')

from dog import HWI
import numpy as np

def test_servo_communication():
    """测试1：检查每个舵机的通信状态"""
    print("\n" + "="*80)
    print("测试1：检查12个舵机的通信状态")
    print("="*80)
    
    hwi = HWI(serial_port='/dev/ttyTHS1', baudrate=115200)
    if not hwi.init_hardware():
        print("❌ 硬件初始化失败")
        return False
    
    time.sleep(1.0)  # 等待自动轮询稳定
    
    # 检查每个舵机的数据更新
    failed_servos = []
    for servo_id in range(12):
        angle = hwi.servo_driver.get_servo_pos(servo_id)
        if angle is None:
            failed_servos.append(servo_id)
            print(f"❌ 舵机ID {servo_id}: 无数据")
        else:
            print(f"✅ 舵机ID {servo_id}: {angle:.1f}°")
    
    hwi.deinit_hardware()
    
    if failed_servos:
        print(f"\n⚠️  发现 {len(failed_servos)} 个舵机无响应: {failed_servos}")
        return False
    else:
        print("\n✅ 所有舵机通信正常")
        return True

def test_continuous_control():
    """测试2：持续控制测试，监控舵机是否会突然停止"""
    print("\n" + "="*80)
    print("测试2：持续控制测试（60秒）")
    print("="*80)
    
    hwi = HWI(serial_port='/dev/ttyTHS1', baudrate=115200)
    if not hwi.init_hardware():
        print("❌ 硬件初始化失败")
        return
    
    time.sleep(1.0)
    
    # 记录每个舵机的最后更新时间
    last_update_time = {i: time.time() for i in range(12)}
    update_timeout = 0.5  # 如果0.5秒没更新，认为舵机停止响应
    
    target_angles = np.full(12, 135.0)
    test_duration = 60  # 测试60秒
    start_time = time.time()
    
    print("开始测试，舵机将在115°和155°之间摆动...")
    print("监控舵机响应，如果某个舵机超过0.5秒没更新数据，将报警\n")
    
    wave_state = 0
    last_angles = hwi.get_all_angles().copy()
    
    try:
        while time.time() - start_time < test_duration:
            # 切换目标角度
            if wave_state == 0:
                target_angles[:] = 115.0
            else:
                target_angles[:] = 155.0
            
            # 发送控制指令
            hwi.set_all_angles(target_angles, duration_ms=500)
            
            # 监控数据更新
            current_time = time.time()
            current_angles = hwi.get_all_angles()
            
            for servo_id in range(12):
                # 检查角度是否有变化
                if abs(current_angles[servo_id] - last_angles[servo_id]) > 0.5:
                    last_update_time[servo_id] = current_time
                
                # 检查是否超时
                time_since_update = current_time - last_update_time[servo_id]
                if time_since_update > update_timeout:
                    print(f"⚠️  [{current_time - start_time:.1f}s] 舵机ID {servo_id} 已 {time_since_update:.2f}秒 无响应！当前角度: {current_angles[servo_id]:.1f}°")
            
            last_angles = current_angles.copy()
            
            # 每2秒切换一次
            if int((current_time - start_time) / 2) != wave_state:
                wave_state = 1 - wave_state
            
            time.sleep(0.02)  # 50Hz控制频率
            
    except KeyboardInterrupt:
        print("\n用户中断测试")
    
    finally:
        hwi.deinit_hardware()
        print("\n✅ 测试结束")

def test_query_frequency():
    """测试3：测试自动轮询的实际频率"""
    print("\n" + "="*80)
    print("测试3：测试自动轮询频率")
    print("="*80)
    
    hwi = HWI(serial_port='/dev/ttyTHS1', baudrate=115200)
    if not hwi.init_hardware():
        print("❌ 硬件初始化失败")
        return
    
    time.sleep(1.0)
    
    # 记录每个舵机的数据更新次数
    update_counts = {i: 0 for i in range(12)}
    last_angles = hwi.get_all_angles().copy()
    
    test_duration = 10  # 测试10秒
    start_time = time.time()
    
    print(f"监控{test_duration}秒内每个舵机的数据更新频率...\n")
    
    while time.time() - start_time < test_duration:
        current_angles = hwi.get_all_angles()
        
        for servo_id in range(12):
            if abs(current_angles[servo_id] - last_angles[servo_id]) > 0.1:
                update_counts[servo_id] += 1
        
        last_angles = current_angles.copy()
        time.sleep(0.01)
    
    elapsed = time.time() - start_time
    
    print("\n数据更新统计：")
    print(f"{'舵机ID':<10} {'更新次数':<15} {'平均频率(Hz)':<15}")
    print("-" * 40)
    
    for servo_id in range(12):
        count = update_counts[servo_id]
        freq = count / elapsed
        status = "✅" if freq > 10 else "⚠️"
        print(f"{servo_id:<10} {count:<15} {freq:<15.1f} {status}")
    
    hwi.deinit_hardware()

def main():
    print("\n" + "="*80)
    print("       🔍 舵机诊断工具")
    print("="*80)
    print("此工具将帮助诊断12个舵机突然停止的问题")
    print("="*80)
    
    # 测试1：通信状态
    if not test_servo_communication():
        print("\n⚠️  基础通信测试失败，请检查硬件连接")
        return
    
    time.sleep(2)
    
    # 测试2：持续控制
    test_continuous_control()
    
    time.sleep(2)
    
    # 测试3：轮询频率
    test_query_frequency()
    
    print("\n" + "="*80)
    print("诊断完成")
    print("="*80)

if __name__ == "__main__":
    main()
