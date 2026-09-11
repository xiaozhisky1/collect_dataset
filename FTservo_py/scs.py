"""
scs.py
飞特串行舵机通信层协议程序
转换自 SCS.cpp 和 SCS.h
"""

import time
from abc import ABC, abstractmethod
from typing import Optional, List, Tuple, Union
from .inst import *

class SCS(ABC):
    """串行舵机通信协议基类"""
    
    def __init__(self, end: int = 1, level: int = 1):
        """
        初始化SCS通信协议
        
        Args:
            end: 大小端模式，1=大端模式，0=小端模式
            level: 返回等级，1=除广播指令所有指令返回应答，0=无应答
        """
        self.end = end
        self.level = level
        self.status = 0
        self.error = 0
        
        # 同步读相关变量
        self.sync_read_rx_packet_index = 0
        self.sync_read_rx_packet_len = 0
        self.sync_read_rx_packet: Optional[bytes] = None
        self.sync_read_rx_buff: Optional[bytes] = None
        self.sync_read_rx_buff_len = 0
        self.sync_read_rx_buff_max = 0
        self.sync_timeout = 0
    
    @abstractmethod
    def write_scs(self, data: bytes) -> int:
        """写数据到串口（抽象方法）
        
        Args:
            data: 要写入的数据
            
        Returns:
            int: 实际写入的字节数
        """
        pass
    
    @abstractmethod
    def read_scs(self, length: int, timeout: Optional[float] = None) -> bytes:
        """从串口读数据（抽象方法）
        
        Args:
            length: 要读取的字节数
            timeout: 超时时间
            
        Returns:
            bytes: 读取到的数据
        """
        pass
    
    @abstractmethod
    def flush_read(self) -> None:
        """清空接收缓冲区（抽象方法）"""
        pass
    
    @abstractmethod
    def flush_write(self) -> None:
        """清空发送缓冲区（抽象方法）"""
        pass
    
    def host_to_scs(self, data: int) -> Tuple[int, int]:
        """
        将16位数据转换为2个8位数据
        
        Args:
            data: 16位整数
            
        Returns:
            tuple: (低位字节, 高位字节)
        """
        if self.end:  # 大端模式
            return ((data >> 8) & 0xFF, data & 0xFF)
        else:  # 小端模式
            return (data & 0xFF, (data >> 8) & 0xFF)
    
    def scs_to_host(self, data_l: int, data_h: int) -> int:
        """
        将2个8位数据合并为1个16位数据
        
        Args:
            data_l: 低位字节
            data_h: 高位字节
            
        Returns:
            int: 16位整数
        """
        if self.end:  # 大端模式
            return (data_l << 8) | data_h
        else:  # 小端模式
            return (data_h << 8) | data_l
    
    def write_buf(self, servo_id: int, mem_addr: int, data: Optional[bytes], func: int) -> None:
        """
        写入缓冲区数据
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存地址
            data: 要写入的数据（bytes或None）
            func: 功能码
        """
        msg_len = 2
        header = [0xFF, 0xFF, servo_id]
        
        if data:
            msg_len += len(data) + 1
            packet = header + [msg_len, func, mem_addr] + list(data)
        else:
            packet = header + [msg_len, func]
            
        # 计算校验和
        checksum = sum(packet[2:]) & 0xFF  # 从ID开始计算
        packet.append((~checksum) & 0xFF)
        
        self.write_scs(bytes(packet))
    
    def gen_write(self, servo_id: int, mem_addr: int, data: bytes) -> bool:
        """
        普通写指令
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存表地址
            data: 写入数据（bytes）
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.flush_read()
        self.write_buf(servo_id, mem_addr, data, INST_WRITE)
        self.flush_write()
        return self.ack(servo_id)
    
    def reg_write(self, servo_id: int, mem_addr: int, data: bytes) -> bool:
        """
        异步写指令
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存表地址
            data: 写入数据（bytes）
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.flush_read()
        self.write_buf(servo_id, mem_addr, data, INST_REG_WRITE)
        self.flush_write()
        return self.ack(servo_id)
    
    def reg_write_action(self, servo_id: int = 0xFE) -> bool:
        """
        异步写执行指令
        
        Args:
            servo_id: 舵机ID，默认为广播地址0xFE
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.flush_read()
        self.write_buf(servo_id, 0, None, INST_REG_ACTION)
        self.flush_write()
        return self.ack(servo_id)
    
    def sync_write(self, servo_ids: List[int], mem_addr: int, data_list: List[int], data_len: int) -> None:
        """
        同步写指令
        
        Args:
            servo_ids: 舵机ID列表
            mem_addr: 内存表地址
            data_list: 数据列表（每个舵机对应的数据）
            data_len: 每个舵机的数据长度
        """
        self.flush_read()
        
        id_count = len(servo_ids)
        msg_len = (data_len + 1) * id_count + 4
        
        packet = [0xFF, 0xFF, 0xFE, msg_len, INST_SYNC_WRITE, mem_addr, data_len]
        
        checksum = 0xFE + msg_len + INST_SYNC_WRITE + mem_addr + data_len
        
        for i, servo_id in enumerate(servo_ids):
            packet.append(servo_id)
            checksum += servo_id
            
            start_idx = i * data_len
            for j in range(data_len):
                data_byte = data_list[start_idx + j]
                packet.append(data_byte)
                checksum += data_byte
        
        packet.append((~checksum) & 0xFF)
        self.write_scs(bytes(packet))
        self.flush_write()
    
    def write_byte(self, servo_id: int, mem_addr: int, data: int) -> bool:
        """
        写1个字节
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存地址
            data: 要写入的字节值
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.gen_write(servo_id, mem_addr, bytes([data]))
    
    def write_word(self, servo_id: int, mem_addr: int, data: int) -> bool:
        """
        写2个字节
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存地址
            data: 要写入的16位值
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        data_l, data_h = self.host_to_scs(data)
        return self.gen_write(servo_id, mem_addr, bytes([data_l, data_h]))
    
    def read(self, servo_id: int, mem_addr: int, data_len: int) -> Optional[bytes]:
        """
        读指令
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存表地址
            data_len: 读取长度
            
        Returns:
            bytes: 读取到的数据，失败返回None
        """
        self.flush_read()
        self.write_buf(servo_id, mem_addr, bytes([data_len]), INST_READ)
        self.flush_write()
        
        self.error = 0
        # [建议新增] 在写入指令和开始读取应答之间，增加一个短暂的延迟
        # 给予舵机足够的处理和响应时间，通常1-2ms即可
        time.sleep(0.001)
        if not self.check_head():
            self.error = SCSError.ERR_NO_REPLY
            return None
        
        # 读取响应头
        response = self.read_scs(3)
        if len(response) != 3:
            self.error = SCSError.ERR_NO_REPLY
            return None
        
        resp_id, resp_len, status = response
        
        if resp_id != servo_id and servo_id != 0xFE:
            self.error = SCSError.ERR_SLAVE_ID
            return None
        
        if resp_len != (data_len + 2):
            self.error = SCSError.ERR_BUFF_LEN
            return None
        
        # 读取数据
        data = self.read_scs(data_len)
        if len(data) != data_len:
            self.error = SCSError.ERR_NO_REPLY
            return None
        
        # 读取校验和
        checksum_data = self.read_scs(1)
        if len(checksum_data) != 1:
            self.error = SCSError.ERR_NO_REPLY
            return None
        
        # 校验
        calc_sum = (resp_id + resp_len + status + sum(data)) & 0xFF
        calc_sum = (~calc_sum) & 0xFF
        
        if calc_sum != checksum_data[0]:
            self.error = SCSError.ERR_CRC_CMP
            return None
        
        self.status = status
        return data
    
    def read_byte(self, servo_id: int, mem_addr: int) -> int:
        """
        读1个字节
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存地址
            
        Returns:
            int: 读取到的字节值，失败返回-1
        """
        data = self.read(servo_id, mem_addr, 1)
        if data is None:
            return -1
        return data[0]
    
    def read_word(self, servo_id: int, mem_addr: int) -> int:
        """
        读2个字节
        
        Args:
            servo_id: 舵机ID
            mem_addr: 内存地址
            
        Returns:
            int: 读取到的16位值，失败返回-1
        """
        data = self.read(servo_id, mem_addr, 2)
        if data is None:
            return -1
        return self.scs_to_host(data[0], data[1])
    
    def ping(self, servo_id: int) -> int:
        """
        Ping指令
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            int: 舵机ID，失败返回-1
        """
        self.flush_read()
        self.write_buf(servo_id, 0, None, INST_PING)
        self.flush_write()
        
        self.status = 0
        
        if not self.check_head():
            self.error = SCSError.ERR_NO_REPLY
            return -1
        
        response = self.read_scs(4)
        if len(response) != 4:
            self.error = SCSError.ERR_NO_REPLY
            return -1
        
        resp_id, resp_len, status, checksum = response
        
        if resp_id != servo_id and servo_id != 0xFE:
            self.error = SCSError.ERR_SLAVE_ID
            return -1
        
        if resp_len != 2:
            self.error = SCSError.ERR_BUFF_LEN
            return -1
        
        calc_sum = (~(resp_id + resp_len + status)) & 0xFF
        if calc_sum != checksum:
            self.error = SCSError.ERR_CRC_CMP
            return -1
        
        self.status = status
        return resp_id
    
    def check_head(self) -> bool:
        """
        检查帧头
        
        Returns:
            bool: 找到帧头返回True，否则返回False
        """
        count = 0
        prev_byte = 0
        
        while count < 10:
            data = self.read_scs(1, timeout=0.1)
            if len(data) == 0:
                return False
            
            current_byte = data[0]
            if prev_byte == 0xFF and current_byte == 0xFF:
                return True
            
            prev_byte = current_byte
            count += 1
        
        return False
    
    def ack(self, servo_id: int) -> bool:
        """
        等待应答
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.error = 0
        
        if servo_id != 0xFE and self.level:
            if not self.check_head():
                self.error = SCSError.ERR_NO_REPLY
                return False
            
            self.status = 0
            response = self.read_scs(4)
            if len(response) != 4:
                self.error = SCSError.ERR_NO_REPLY
                return False
            
            resp_id, resp_len, status, checksum = response
            
            if resp_id != servo_id:
                self.error = SCSError.ERR_SLAVE_ID
                return False
            
            if resp_len != 2:
                self.error = SCSError.ERR_BUFF_LEN
                return False
            
            calc_sum = (~(resp_id + resp_len + status)) & 0xFF
            if calc_sum != checksum:
                self.error = SCSError.ERR_CRC_CMP
                return False
            
            self.status = status
        
        return True
    
    def reset(self, servo_id: int) -> bool:
        """
        重置舵机状态
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.flush_read()
        self.write_buf(servo_id, 0, None, INST_RESET)
        self.flush_write()
        return self.ack(servo_id)
    
    def recal(self, servo_id: int) -> bool:
        """
        重置舵机中位
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.flush_read()
        self.write_buf(servo_id, 0, None, INST_CAL)
        self.flush_write()
        return self.ack(servo_id)
    
    def get_state(self) -> int:
        """获取舵机状态"""
        return self.status
    
    def get_last_error(self) -> int:
        """获取最后的错误码"""
        return self.error