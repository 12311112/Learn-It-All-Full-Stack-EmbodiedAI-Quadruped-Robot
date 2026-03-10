import serial
import time
import threading
import subprocess
from threading import Lock

class ZLServoController:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        # 硬件配置
        self.port = port
        self.baudrate = baudrate
        
        # 串口与线程安全
        self.ser = None
        self.ser_lock = Lock()
        
        # 接收线程与数据缓存
        self.received_data = bytearray()
        self.last_packet_time = 0
        self.MIN_PACKET_INTERVAL = 0.05
        self.stop_thread = False
        self.recv_thread = None

        # 舵机数据存储
        self.servo_current_pos = {}  # 存储舵机当前位置 {id: (pwm, angle)}
        # 计算135°对应的PWM值（500 + (135/270)*2000 = 1500）
        init_pwm = 1500
        init_angle = 135.0
        # 初始化ID 0-11共12个舵机的初始值
        for servo_id in range(12):
            self.servo_current_pos[servo_id] = (init_pwm, init_angle)        
        self.pos_lock = Lock()  # 专门保护 servo_current_pos 的锁  

        # 舵机协议常量
        self.PWM_MIN = 500
        self.PWM_MAX = 2500
        self.MAX_ANGLE = 270
        self.PACKET_FIXED_LEN = 10
        self.CHAR_HEADER = b'#'
        self.CHAR_TAIL = b'!'

        # ====================== 新增：自动轮询相关属性 ======================
        self.auto_query_thread = None  # 自动轮询线程
        self.auto_query_stop = False   # 轮询线程停止标记（线程安全）
        self.auto_query_freq = 50      # 自动轮询频率（50Hz，和控制频率对齐）
        self.auto_query_interval = 1.0 / self.auto_query_freq  # 轮询间隔（20ms）
        self.servo_ids_to_query = []   # 需要轮询的舵机ID列表
        self.auto_query_lock = Lock()  # 保护轮询参数的锁
        # ==================================================================

    def _set_serial_permissions(self):
        """内部方法：设置串口权限"""
        try:
            result = subprocess.run(
                ['sudo', 'chmod', '777', self.port],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            print(f"✅ 串口 {self.port} 权限设置成功")
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ 串口权限设置失败: {e.stderr}")
            return False

    def connect(self):
        """连接串口：设置权限 -> 打开串口 -> 启动接收线程"""
        # 1. 设置权限
        self._set_serial_permissions()
        time.sleep(0.1)
        
        # 2. 打开串口
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✅ 串口 {self.port} 连接成功，波特率 {self.baudrate}")
        except Exception as e:
            print(f"❌ 串口连接失败：{str(e)}")
            return False
        
        # 3. 启动接收线程
        self.stop_thread = False
        self.recv_thread = threading.Thread(target=self._receive_process_func, daemon=True)
        self.recv_thread.start()
        print("✅ 接收线程已启动")
        return True

    def disconnect(self):
        """安全断开：停止所有线程 -> 关闭串口"""
        # ====================== 新增：停止自动轮询线程 ======================
        self.stop_auto_query()
        # ==================================================================
        
        # 原有逻辑：停止接收线程
        self.stop_thread = True
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        
        # 关闭串口
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("✅ 串口已关闭")

    # ====================== 新增：自动轮询核心方法 ======================
    def _auto_query_func(self):
        """内部方法：自动轮询线程主逻辑——循环给所有舵机发查询指令"""
        while True:
            # 先检查停止信号（优先退出）
            with self.auto_query_lock:
                if self.auto_query_stop:
                    break
            
            start_time = time.time()
            # 遍历需要轮询的舵机ID，逐个发送查询指令
            with self.auto_query_lock:
                target_ids = self.servo_ids_to_query.copy()  # 拷贝避免遍历中修改
            
            for servo_id in target_ids:
                # 每次发送前再检查停止信号，避免指令堆积
                with self.auto_query_lock:
                    if self.auto_query_stop:
                        break
                # 发送查询指令（复用原有get_position方法）
                self.get_position(servo_id)
                time.sleep(0.001)  # 微延时，避免串口指令拥堵
            
            # 控制轮询频率（确保50Hz，补偿指令发送耗时）
            cost = time.time() - start_time
            sleep_time = max(0, self.auto_query_interval - cost)
            time.sleep(sleep_time)
            #print(sleep_time)
        
        print("✅ 自动轮询线程已退出")

        
    def start_auto_query(self, servo_ids):
        """
        公开方法：启动自动轮询（被调用方调用）
        :param servo_ids: list，需要实时查询的舵机ID列表，比如[0,1,2,...,11]
        """
        with self.auto_query_lock:
            # 校验参数
            if not isinstance(servo_ids, list) or len(servo_ids) == 0:
                print("❌ 自动轮询启动失败：舵机ID列表不能为空")
                return False
            # 检查线程是否已运行
            if self.auto_query_thread and self.auto_query_thread.is_alive():
                print("⚠️  自动轮询线程已在运行，无需重复启动")
                return True
            # 设置轮询ID并启动线程
            self.servo_ids_to_query = servo_ids
            self.auto_query_stop = False
        
        self.auto_query_thread = threading.Thread(target=self._auto_query_func, daemon=True)
        self.auto_query_thread.start()
        print(f"✅ 自动轮询线程启动成功 | 频率：{self.auto_query_freq}Hz | 轮询ID：{servo_ids}")
        return True

    def stop_auto_query(self):
        """公开方法：停止自动轮询（被调用方调用）"""
        with self.auto_query_lock:
            if not self.auto_query_stop:
                self.auto_query_stop = True
        
        # 等待线程退出
        if self.auto_query_thread and self.auto_query_thread.is_alive():
            self.auto_query_thread.join(timeout=1.0)
            self.auto_query_thread = None  # 清空线程对象
        print("✅ 自动轮询已停止")
    # ==================================================================

    def _process_packet(self, valid_packet_bytes):
        """内部方法：解析舵机位置返回包 #xxxPxxxx!
        注意：这里直接接收 bytes，不接收 str，防止 decode 破坏数据
        """
        try:
            # 1. 先在 bytes 层面检查格式，极其严格
            # 协议格式： b'#' + [3字节ID] + b'P' + [4字节PWM] + b'!'
            if len(valid_packet_bytes) != 10:
                return
            if valid_packet_bytes[0:1] != b'#' or valid_packet_bytes[9:10] != b'!':
                return
            if valid_packet_bytes[4:5] != b'P': # 第5个字节必须是 b'P' (索引是4)
                return

            # 2. 安全解码
            valid_packet_str = valid_packet_bytes.decode('ascii')
            
            # 3. 分割
            content = valid_packet_str[1:-1]
            id_part, pwm_part = content.split('P', maxsplit=1)

            servo_id = int(id_part)
            pwm_val = int(pwm_part)

            # 4. 额外的数值范围校验
            if not (0 <= servo_id <= 254) or not (500 <= pwm_val <= 2500):
                return

            # PWM转角度
            angle = (pwm_val - self.PWM_MIN) * self.MAX_ANGLE / (self.PWM_MAX - self.PWM_MIN)
            angle = round(angle, 2)
            
            # 更新存储
            with self.pos_lock:
                self.servo_current_pos[servo_id] = (pwm_val, angle)
            # 解析成功的打印，建议平时关掉，刷屏太厉害
            # print(f"✅ ID={servo_id}, PWM={pwm_val}, Angle={angle}°")
            
        except Exception as e:
            # 所有错误静默处理
            pass

    def _receive_process_func(self):
        """内部方法：接收线程主循环（优化版：严格切包）"""
        while not self.stop_thread:
            try:
                # 1. 线程安全地读取新数据
                with self.ser_lock:
                    if self.ser and self.ser.in_waiting > 0:
                        new_data = self.ser.read(self.ser.in_waiting)
                        self.received_data += new_data

                # 2. 处理缓存数据 (核心修改：更严谨的找包逻辑)
                while len(self.received_data) >= self.PACKET_FIXED_LEN:
                    # 找包头 '#'
                    if self.received_data[0:1] != self.CHAR_HEADER:
                        self.received_data = self.received_data[1:]
                        continue

                    # 【关键修改】预检查：第10个字节必须是 '!'，且第5个字节必须是 'P'
                    # 如果不满足，说明这个 '#' 是假的包头，扔掉
                    if self.received_data[9:10] == self.CHAR_TAIL and self.received_data[4:5] == b'P':
                        # 确认是完整包，切出来
                        valid_packet_bytes = self.received_data[:self.PACKET_FIXED_LEN]
                        self.received_data = self.received_data[self.PACKET_FIXED_LEN:]
                        
                        # 直接传 bytes 给解析函数，不要在这里 decode
                        self._process_packet(valid_packet_bytes)
                        continue

                    # 如果虽然找到了 '#'，但后面结构不对，说明是干扰，扔掉这一个字节继续找
                    self.received_data = self.received_data[1:]

            except Exception as e:
                pass
            
            time.sleep(0.001)

    def _send_cmd(self, cmd):
        """内部方法：线程安全的指令发送"""
        try:


            with self.ser_lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(cmd.encode('ascii'))
                    # print(f"📤 已发送指令：{cmd}") # 可取消注释调试
                    return True
                else:
                    print("⚠️  串口未打开，无法发送指令")
                    return False
        except Exception as e:
            print(f"⚠️  指令发送异常：{str(e)}")
            return False

    # -------------------------- 原有公开API（完全未修改） --------------------------
    def get_position(self, servo_id):
        """读取舵机当前位置 (#IDPRAD!)"""
        id_str = f"{servo_id:03d}"
        cmd = f"#{id_str}PRAD!"
        return self._send_cmd(cmd)

    def set_start_position(self, servo_id, position=1500):
        """设置舵机启动位置 (#IDPCSDXXXX!)"""
        if not 500 <= position <= 2500:
            print(f"⚠️  启动位置值非法！有效范围为500~2500，当前输入：{position}")
            return False
        
        id_str = f"{servo_id:03d}"
        pos_str = f"{position:04d}"
        cmd = f"#{id_str}PCSD{pos_str}!"
        print(f"📤 设置ID {servo_id} 启动位置为 {position}")
        return self._send_cmd(cmd)

    def disable_servo(self, servo_id):
        """舵机失能/释放扭力 (#IDPULK!)"""
        id_str = f"{servo_id:03d}"
        cmd = f"#{id_str}PULK!"
        print(f"📤 失能ID {servo_id}")
        return self._send_cmd(cmd)
    
    def enable_servo(self, servo_id):
        """舵机使能/恢复扭力 (#IDPULR!)"""
        id_str = f"{servo_id:03d}"  # 补全缺失的ID格式化
        cmd = f"#{id_str}PULR!"     # 补全缺失的指令构造
        print(f"📤 使能ID {servo_id}")  # 补全日志打印
        return self._send_cmd(cmd)

    def set_angle(self, servo_id, angle, s_time=0):
        """开环角度控制 (#IDPWM值TIME值!)"""
        if servo_id < 0 or servo_id > 254:
            print(f"❌ 角度控制失败：舵机ID {servo_id} 超出0-254范围")
            return False
        if not (0 <= angle <= 270):
            print(f"❌ 角度控制失败：角度{angle}超出0-270°范围")
            return False
        if not (0 <= s_time <= 9999):
            print(f"❌ 角度控制失败：时间{s_time}ms超出0-9999ms范围")
            return False

        # 角度转PWM
        pwm_val = int(self.PWM_MIN + (angle / self.MAX_ANGLE) * (self.PWM_MAX - self.PWM_MIN))
        pwm_val = max(self.PWM_MIN, min(self.PWM_MAX, pwm_val))

        id_str = f"{servo_id:03d}"
        pwm_str = f"{pwm_val:04d}"
        time_str = f"{s_time:04d}"
        cmd = f"#{id_str}P{pwm_str}T{time_str}!"
        time.sleep(0.001)
        return self._send_cmd(cmd)


    def get_servo_pos(self, servo_id):
        with self.pos_lock:
            pos_data = self.servo_current_pos.get(servo_id, None)
            if pos_data is not None:
                return pos_data[1]  # 有数据返回角度
            return None  # 无数据返回None，避免索引错误

    def get_servo_pwm(self, servo_id):
        """【定标专用】直接读取舵机当前PWM原始值，不做角度转换"""
        with self.pos_lock:
            pos_data = self.servo_current_pos.get(servo_id, None)
        if pos_data:
            return pos_data[0]  # 返回PWM值
        return None

    def calibrate_mid_value(self, servo_id):
        """【核心定标指令】执行#IDPSCK!，将舵机当前物理位置设为1500中值"""
        id_str = f"{servo_id:03d}"
        cmd = f"#{id_str}PSCK!"
        print(f"📐 执行ID {servo_id} 中值矫正指令：{cmd}")
        return self._send_cmd(cmd)

    def reset_factory(self, servo_id, keep_id=True):
        """【兜底功能】恢复出厂设置，keep_id=True保留当前ID，False全恢复为ID0"""
        id_str = f"{servo_id:03d}"
        if keep_id:
            cmd = f"#{id_str}PCLE0!"  # 半恢复，保留ID
            print(f"⚠️  执行ID {servo_id} 半恢复出厂（保留ID）")
        else:
            cmd = f"#{id_str}PCLE!"  # 全恢复，ID重置为0
            print(f"⚠️  执行ID {servo_id} 全恢复出厂（ID重置为0）")
        return self._send_cmd(cmd)
    
    



if __name__ == "__main__":
    pass


