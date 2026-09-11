"""
scserial.py
飞特串行舵机硬件接口层程序
转换自 SCSerial.cpp 和 SCSerial.h
"""

import time
import serial
from typing import Optional
from .scs import SCS

class SCSerial(SCS):
    """串行舵机串口通信类"""

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200,
                 end: int = 1, level: int = 1, timeout: float = 0.01):
        """
        初始化串口通信

        Args:
            port: 串口号，如 'COM3' (Windows) 或 '/dev/ttyUSB0' (Linux)
            baudrate: 波特率
            end: 大小端模式
            level: 返回等级
            timeout: 串口超时时间（秒）
        """
        super().__init__(end, level)
        self.io_timeout = timeout
        self.serial_port: Optional[serial.Serial] = None

        if port:
            self.open(port, baudrate, timeout)

    def open(self, port: str, baudrate: int = 115200, timeout: float = 0.01) -> bool:
        """
        打开串口

        Args:
            port: 串口号
            baudrate: 波特率
            timeout: 超时时间

        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=timeout
            )
            self.io_timeout = timeout
            return True
        except Exception as e:
            print(f"打开串口失败: {e}")
            return False

    def close(self) -> None:
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None

    def is_open(self) -> bool:
        """检查串口是否打开"""
        return self.serial_port is not None and self.serial_port.is_open

    def write_scs(self, data: bytes) -> int:
        """
        写数据到串口

        Args:
            data: 要写入的数据（bytes）

        Returns:
            int: 实际写入的字节数
        """
        if not self.is_open():
            return 0

        try:
            if self.serial_port is not None:
                result = self.serial_port.write(data)
                return result if result is not None else 0
            else:
                return 0
        except Exception as e:
            print(f"串口写入失败: {e}")
            return 0

    def read_scs(self, length: int, timeout: Optional[float] = None) -> bytes:
        if not self.is_open() or self.serial_port is None:
            return bytes()

        total_bytes_read = 0
        attempts = 0
        max_attempts = 100 # 最大连续空读次数
        current_delay_ms = 0
        consecutive_empty_reads = 0
        data_buffer = bytearray(length)

        original_timeout = self.serial_port.timeout
        # 设置一个极短的单次读取超时，以实现非阻塞轮询
        self.serial_port.timeout = 0.001

        start_time = time.time()
        if timeout is None:
            timeout = self.io_timeout * length  # 根据长度调整总超时

        try:
            while total_bytes_read < length and attempts < max_attempts:
                if time.time() - start_time > timeout:
                    return bytes()

                remaining = length - total_bytes_read
                chunk = self.serial_port.read(remaining)

                if chunk:
                    # 成功读取到数据
                    chunk_len = len(chunk)
                    data_buffer[total_bytes_read: total_bytes_read + chunk_len] = chunk
                    total_bytes_read += chunk_len
                    consecutive_empty_reads = 0  # 重置空读计数

                    # 动态调整：如果读取顺利，减少延迟
                    if current_delay_ms > 0:
                        current_delay_ms = max(0, current_delay_ms - 1)
                else:
                    # 空读取 (read timed out)
                    consecutive_empty_reads += 1
                    attempts += 1

                    # 动态增加延迟：根据连续空读次数进行退避
                    if consecutive_empty_reads <= 3:
                        current_delay_ms = consecutive_empty_reads
                    else:
                        current_delay_ms = min(5, consecutive_empty_reads)

                if current_delay_ms > 0:
                    time.sleep(current_delay_ms / 1000.0)

            if total_bytes_read == length:
                return bytes(data_buffer)
            else:
                return bytes()
        except Exception as e:
            print(f"串口读取时发生异常: {e}")
            return bytes()
        finally:
            self.serial_port.timeout = original_timeout

    def flush_read(self) -> None:
        """清空接收缓冲区"""
        if self.is_open() and self.serial_port is not None:
            try:
                self.serial_port.reset_input_buffer()
            except Exception as e:
                print(f"清空接收缓冲区失败: {e}")

    def flush_write(self) -> None:
        """清空发送缓冲区"""
        if self.is_open() and self.serial_port is not None:
            try:
                self.serial_port.flush()
            except Exception as e:
                print(f"清空发送缓冲区失败: {e}")

    def set_baudrate(self, baudrate: int) -> bool:
        """
        设置波特率

        Args:
            baudrate: 新的波特率

        Returns:
            bool: 成功返回True，失败返回False
        """
        if self.is_open() and self.serial_port is not None:
            try:
                self.serial_port.baudrate = baudrate
                return True
            except Exception as e:
                print(f"设置波特率失败: {e}")
                return False
        return False

    def __del__(self) -> None:
        """析构函数，确保串口被正确关闭"""
        self.close()