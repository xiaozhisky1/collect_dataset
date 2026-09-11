"""
wifiservo.py
WiFi舵机通讯库 - 基于TCP Socket通讯
参考scserial.py的结构实现
【修复版本：增加了超时时间以适配ESP32响应速度】
"""

import time
import socket
from typing import Optional
from .scs import SCS
# 导入HLS舵机相关常量
from .hlscl import *

class WiFiSerial(SCS):
    """WiFi串行舵机TCP通信类"""

    def __init__(self, host: Optional[str] = None, port: int = 8080,
                 end: int = 1, level: int = 1, timeout: float = 20):
        """
        初始化WiFi通信

        Args:
            host: 目标主机IP地址
            port: 目标端口号
            end: 大小端模式
            level: 返回等级
            timeout: 超时时间（秒）【修复：从5秒增加到20秒】
        """
        super().__init__(end, level)
        self.io_timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.host = host
        self.port = port

        if host:
            self.connect(host, port, timeout)

    def connect(self, host: str, port: int = 8080, timeout: float = 20) -> bool:
        """
        连接到WiFi设备

        Args:
            host: 目标主机IP地址
            port: 目标端口号
            timeout: 超时时间【修复：默认增加到20秒】

        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            print(f"正在连接 {host}:{port}，超时时间：{timeout}秒...")
            self.socket.connect((host, port))
            self.host = host
            self.port = port
            self.io_timeout = timeout
            print("WiFi连接建立成功！")
            return True
        except socket.timeout:
            print(f"WiFi连接超时（{timeout}秒），请检查ESP32是否正常运行")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
        except Exception as e:
            print(f"WiFi连接失败: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    def close(self) -> None:
        """关闭WiFi连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

    def is_open(self) -> bool:
        """检查WiFi连接是否打开"""
        if self.socket is None:
            return False

        try:
            # 尝试发送空数据测试连接
            self.socket.send(b'')
            return True
        except:
            return False

    def write_scs(self, data: bytes) -> int:
        """
        通过WiFi发送数据

        Args:
            data: 要发送的数据（bytes）

        Returns:
            int: 实际发送的字节数
        """
        if not self.is_open():
            return 0

        try:
            if self.socket is not None:
                result = self.socket.send(data)
                return result if result is not None else 0
            else:
                return 0
        except Exception as e:
            print(f"WiFi发送失败: {e}")
            return 0

    def read_scs(self, length: int, timeout: Optional[float] = None) -> bytes:
        """
        从WiFi Socket读取指定长度的数据.
        【此版本实现了类似C++的动态延迟和重试逻辑，并修正了无限递归的bug】

        Args:
            length: 要读取的字节数
            timeout: 总操作的超时时间（秒），None表示使用默认

        Returns:
            bytes: 读取到的数据，失败或超时返回空bytes
        """
        if not self.is_open() or self.socket is None:
            return bytes()

        total_bytes_read = 0
        attempts = 0
        max_attempts = 100  # WiFi 可以适当增加尝试次数
        current_delay_ms = 0
        consecutive_empty_reads = 0
        data_buffer = bytearray(length)
        # 保存原始超时设置，以便后续恢复
        original_timeout = self.socket.gettimeout()

        # 在循环内部处理超时，所以这里可以设置一个很短的非阻塞超时
        self.socket.settimeout(10)  # 5ms【修复：从1ms增加到5ms】

        start_time = time.time()

        if timeout is None:
            timeout = self.io_timeout

        try:
            while total_bytes_read < length and attempts < max_attempts:
                if time.time() - start_time > timeout:
                    print("WiFi读取总超时")
                    return bytes()

                remaining = length - total_bytes_read
                try:
                    # [FIXED] 从 self.read() 改为 self.socket.recv()，解决无限递归
                    chunk = self.socket.recv(remaining)
                except (BlockingIOError, socket.timeout):
                    # 在非阻塞模式下，没读到数据是正常情况
                    chunk = None

                if chunk:
                    chunk_len = len(chunk)
                    data_buffer[total_bytes_read: total_bytes_read + chunk_len] = chunk
                    total_bytes_read += chunk_len
                    consecutive_empty_reads = 0
                    if current_delay_ms > 0:
                        current_delay_ms = max(0, current_delay_ms - 1)
                    if total_bytes_read < length and current_delay_ms > 0:
                        time.sleep(current_delay_ms / 1000.0)
                else:
                    consecutive_empty_reads += 1
                    attempts += 1
                    if consecutive_empty_reads <= 3:
                        current_delay_ms = consecutive_empty_reads
                    else:
                        current_delay_ms = min(5, consecutive_empty_reads)
                    time.sleep(current_delay_ms / 1000.0)

            if total_bytes_read == length:
                return bytes(data_buffer)
            else:
                print(f"WiFi读取不完整: 期望 {length} 字节, 实际得到 {total_bytes_read} 字节")
                return bytes()

        except Exception as e:
            print(f"WiFi读取时发生异常: {e}")
            return bytes()
        finally:
            self.socket.settimeout(original_timeout)

    def flush_read(self) -> None:
        """清空接收缓冲区"""
        if self.is_open() and self.socket is not None:
            try:
                # 设置非阻塞模式
                self.socket.setblocking(False)
                try:
                    while True:
                        data = self.socket.recv(1024)
                        if not data:
                            break
                except socket.error:
                    pass
                finally:
                    # 恢复阻塞模式
                    self.socket.setblocking(True)
            except Exception as e:
                print(f"清空WiFi接收缓冲区失败: {e}")

    def flush_write(self) -> None:
        """清空发送缓冲区"""
        # TCP Socket会自动处理发送缓冲区，这里不需要特殊操作
        pass

    def set_timeout(self, timeout: float) -> bool:
        """
        设置超时时间

        Args:
            timeout: 新的超时时间

        Returns:
            bool: 成功返回True，失败返回False
        """
        if self.is_open() and self.socket is not None:
            try:
                self.socket.settimeout(timeout)
                self.io_timeout = timeout
                return True
            except Exception as e:
                print(f"设置WiFi超时时间失败: {e}")
                return False
        return False

    def send_wifi_config(self, ssid: str, password: str) -> bool:
        """
        发送WiFi配置信息（特殊指令）

        Args:
            ssid: WiFi名称
            password: WiFi密码

        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 构造WiFi配置指令（根据实际协议调整）
            config_cmd = f"WIFI_CONFIG:{ssid}:{password}\n"
            data = config_cmd.encode('utf-8')

            sent = self.write_scs(data)
            return sent == len(data)
        except Exception as e:
            print(f"发送WiFi配置失败: {e}")
            return False

    def delete_wifi_config(self, ssid: str) -> bool:
        """
        删除WiFi配置信息（特殊指令）

        Args:
            ssid: 要删除的WiFi名称

        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 构造WiFi删除指令（根据实际协议调整）
            delete_cmd = f"WIFI_DELETE:{ssid}\n"
            data = delete_cmd.encode('utf-8')

            sent = self.write_scs(data)
            return sent == len(data)
        except Exception as e:
            print(f"删除WiFi配置失败: {e}")
            return False

    def __del__(self) -> None:
        """析构函数，确保连接被正确关闭"""
        self.close()


# WiFi版本的HLS舵机控制类
class WiFiHLSCL(WiFiSerial):
    """WiFi版本的HLS系列串行舵机控制类"""

    def __init__(self, host=None, port=8080, end=0, level=1, timeout=0.5):
        """
        初始化WiFi HLS舵机

        Args:
            host: 目标主机IP
            port: 目标端口
            end: 大小端模式，HLS默认为0（小端）
            level: 返回等级
            timeout: 超时时间【修复：从2秒改为0.5秒，与C++版本的IO超时策略一致】
        """
        super().__init__(host, port, end, level, timeout)
        # 反馈数据缓冲区
        self.mem = [0] * (HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_POSITION_L + 1)

    def write_pos_ex(self, servo_id, position, speed, acc=0, torque=0):
        """
        普通写单个舵机位置指令
        """
        # 处理负位置
        if position < 0:
            position = -position
            position |= (1 << 15)

        # 转换为字节
        pos_l, pos_h = self.host_to_scs(position)
        torque_l, torque_h = self.host_to_scs(torque)
        speed_l, speed_h = self.host_to_scs(speed)

        data = bytes([acc, pos_l, pos_h, torque_l, torque_h, speed_l, speed_h])
        return self.gen_write(servo_id, HLSCL_ACC, data)

    def sync_write_pos_ex(self, servo_ids, positions, speeds, accs=None, torques=None):
        """
        同步写多个舵机位置指令

        Args:
            servo_ids: 舵机ID列表
            positions: 位置列表（带符号）
            speeds: 速度列表
            accs: 加速度列表（可选）
            torques: 扭矩列表（必需）
        """
        if torques is None:
            raise ValueError("HLS舵机同步控制需要提供扭矩参数")

        servo_count = len(servo_ids)
        data_list = []

        for i in range(servo_count):
            position = positions[i]
            if position < 0:
                position = -position
                position |= (1 << 15)

            acc_val = accs[i] if accs else 0
            torque_val = torques[i]
            speed_val = speeds[i]

            # 转换为字节
            pos_l, pos_h = self.host_to_scs(position)
            torque_l, torque_h = self.host_to_scs(torque_val)
            speed_l, speed_h = self.host_to_scs(speed_val)

            data_list.extend([acc_val, pos_l, pos_h, torque_l, torque_h, speed_l, speed_h])

        return self.sync_write(servo_ids, HLSCL_ACC, data_list, 7)

    def servo_mode(self, servo_id):
        """Servo模式"""
        return self.write_byte(servo_id, HLSCL_MODE, 0)

    def wheel_mode(self, servo_id):
        """轮式模式"""
        return self.write_byte(servo_id, HLSCL_MODE, 1)

    def ele_mode(self, servo_id):
        """电流模式"""
        return self.write_byte(servo_id, HLSCL_MODE, 2)

    def write_spe(self, servo_id, speed, acc=0, torque=0):
        """
        轮式模式控制指令

        Args:
            servo_id: 舵机ID
            speed: 速度（带符号）
            acc: 加速度
            torque: 扭矩

        Returns:
            bool: 成功返回True，失败返回False
        """
        if speed < 0:
            speed = -speed
            speed |= (1 << 15)

        pos_l, pos_h = self.host_to_scs(0)
        torque_l, torque_h = self.host_to_scs(torque)
        speed_l, speed_h = self.host_to_scs(speed)

        data = bytes([acc, pos_l, pos_h, torque_l, torque_h, speed_l, speed_h])
        return self.gen_write(servo_id, HLSCL_ACC, data)

    def write_ele(self, servo_id, torque):
        """
        电流模式控制指令

        Args:
            servo_id: 舵机ID
            torque: 扭矩（带符号）

        Returns:
            bool: 成功返回True，失败返回False
        """
        if torque < 0:
            torque = -torque
            torque |= (1 << 15)

        return self.write_word(servo_id, HLSCL_GOAL_TORQUE_L, torque)
    
    def enable_torque(self, servo_id, enable):
        """扭力控制指令"""
        return self.write_byte(servo_id, HLSCL_TORQUE_ENABLE, enable)

    def unlock_eprom(self, servo_id):
        """
        EPROM解锁
        """
        self.enable_torque(servo_id, 0)
        return self.write_byte(servo_id, HLSCL_LOCK, 0)

    def lock_eprom(self, servo_id):
        """
        EPROM加锁
        """
        return self.write_byte(servo_id, HLSCL_LOCK, 1)

    def calibration_ofs(self, servo_id):
        """
        中位校准
        """
        self.enable_torque(servo_id, 0)
        self.unlock_eprom(servo_id)
        # The recal method is inherited from the base class and should be available
        return self.recal(servo_id)

    def servo_mode(self, servo_id):
        """Servo模式"""
        return self.write_byte(servo_id, HLSCL_MODE, 0)
    
    def feedback(self, servo_id):
        """反馈舵机信息"""
        data = self.read(servo_id, HLSCL_PRESENT_POSITION_L, len(self.mem))
        if data is None:
            return False
        
        self.mem = list(data)
        return True
    
    def read_pos(self, servo_id=-1):
        """读位置"""
        if servo_id == -1:
            pos_h_idx = HLSCL_PRESENT_POSITION_H - HLSCL_PRESENT_POSITION_L
            pos_l_idx = HLSCL_PRESENT_POSITION_L - HLSCL_PRESENT_POSITION_L
            pos = self.scs_to_host(self.mem[pos_l_idx], self.mem[pos_h_idx])
        else:
            pos = self.read_word(servo_id, HLSCL_PRESENT_POSITION_L)
        
        if pos == -1:
            return -1
        
        # 处理符号位
        if pos & (1 << 15):
            pos = -(pos & ~(1 << 15))
        
        return pos
    
    def read_load(self, servo_id=-1):
        """读输出至电机的电压百分比"""
        if servo_id == -1:
            load_h_idx = HLSCL_PRESENT_LOAD_H - HLSCL_PRESENT_POSITION_L
            load_l_idx = HLSCL_PRESENT_LOAD_L - HLSCL_PRESENT_POSITION_L
            load = self.scs_to_host(self.mem[load_l_idx], self.mem[load_h_idx])
        else:
            load = self.read_word(servo_id, HLSCL_PRESENT_LOAD_L)
        
        if load == -1:
            return -1
        
        # 处理符号位
        if load & (1 << 10):
            load = -(load & ~(1 << 10))
        
        return load
    
    def read_voltage(self, servo_id=-1):
        """读电压"""
        if servo_id == -1:
            voltage_idx = HLSCL_PRESENT_VOLTAGE - HLSCL_PRESENT_POSITION_L
            return self.mem[voltage_idx]
        else:
            return self.read_byte(servo_id, HLSCL_PRESENT_VOLTAGE)
    
    def read_temper(self, servo_id=-1):
        """读温度"""
        if servo_id == -1:
            temp_idx = HLSCL_PRESENT_TEMPERATURE - HLSCL_PRESENT_POSITION_L
            return self.mem[temp_idx]
        else:
            return self.read_byte(servo_id, HLSCL_PRESENT_TEMPERATURE)
    
    def read_move(self, servo_id=-1):
        """读移动状态"""
        if servo_id == -1:
            move_idx = HLSCL_MOVING - HLSCL_PRESENT_POSITION_L
            return self.mem[move_idx]
        else:
            return self.read_byte(servo_id, HLSCL_MOVING)
    
    def read_current(self, servo_id=-1):
        """读电流"""
        if servo_id == -1:
            current_h_idx = HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_POSITION_L
            current_l_idx = HLSCL_PRESENT_CURRENT_L - HLSCL_PRESENT_POSITION_L
            current = self.scs_to_host(self.mem[current_l_idx], self.mem[current_h_idx])
        else:
            current = self.read_word(servo_id, HLSCL_PRESENT_CURRENT_L)
        
        if current == -1:
            return -1
        
        # 处理符号位
        if current & (1 << 15):
            current = -(current & ~(1 << 15))
        
        return current