"""
hlscl.py
飞特HLS系列串行舵机应用层程序
转换自 HLSCL.cpp 和 HLSCL.h
"""

from .scserial import SCSerial

# HLSCL内存表定义
# -------EPROM(只读)--------
HLSCL_MODEL_L = 3
HLSCL_MODEL_H = 4

# -------EPROM(读写)--------
HLSCL_ID = 5
HLSCL_BAUD_RATE = 6
HLSCL_SECOND_ID = 7
HLSCL_MIN_ANGLE_LIMIT_L = 9
HLSCL_MIN_ANGLE_LIMIT_H = 10
HLSCL_MAX_ANGLE_LIMIT_L = 11
HLSCL_MAX_ANGLE_LIMIT_H = 12
HLSCL_CW_DEAD = 26
HLSCL_CCW_DEAD = 27
HLSCL_OFS_L = 31
HLSCL_OFS_H = 32
HLSCL_MODE = 33

# -------SRAM(读写)--------
HLSCL_TORQUE_ENABLE = 40
HLSCL_ACC = 41
HLSCL_GOAL_POSITION_L = 42
HLSCL_GOAL_POSITION_H = 43
HLSCL_GOAL_TORQUE_L = 44
HLSCL_GOAL_TORQUE_H = 45
HLSCL_GOAL_SPEED_L = 46
HLSCL_GOAL_SPEED_H = 47
HLSCL_TORQUE_LIMIT_L = 48
HLSCL_TORQUE_LIMIT_H = 49
HLSCL_LOCK = 55

# -------SRAM(只读)--------
HLSCL_PRESENT_POSITION_L = 56
HLSCL_PRESENT_POSITION_H = 57
HLSCL_PRESENT_SPEED_L = 58
HLSCL_PRESENT_SPEED_H = 59
HLSCL_PRESENT_LOAD_L = 60
HLSCL_PRESENT_LOAD_H = 61
HLSCL_PRESENT_VOLTAGE = 62
HLSCL_PRESENT_TEMPERATURE = 63
HLSCL_MOVING = 66
HLSCL_PRESENT_CURRENT_L = 69
HLSCL_PRESENT_CURRENT_H = 70

class HLSCL(SCSerial):
    """HLS系列串行舵机控制类"""
    
    def __init__(self, port=None, baudrate=115200, end=0, level=1, timeout=0.01):
        """
        初始化HLS舵机
        
        Args:
            port: 串口号
            baudrate: 波特率
            end: 大小端模式，HLS默认为0（小端）
            level: 返回等级
            timeout: 超时时间
        """
        super().__init__(port, baudrate, end, level, timeout)
        # 反馈数据缓冲区
        self.mem = [0] * (HLSCL_PRESENT_CURRENT_H - HLSCL_PRESENT_POSITION_L + 1)
    
    def write_pos_ex(self, servo_id, position, speed, acc=0, torque=0):
        """
        普通写单个舵机位置指令
        
        Args:
            servo_id: 舵机ID
            position: 目标位置（带符号）
            speed: 执行速度
            acc: 加速度，默认为0
            torque: 扭矩，默认为0
            
        Returns:
            bool: 成功返回True，失败返回False
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
    
    def reg_write_pos_ex(self, servo_id, position, speed, acc=0, torque=0):
        """
        异步写单个舵机位置指令（RegWriteAction生效）
        
        Args:
            servo_id: 舵机ID
            position: 目标位置（带符号）
            speed: 执行速度
            acc: 加速度
            torque: 扭矩
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        if position < 0:
            position = -position
            position |= (1 << 15)
        
        pos_l, pos_h = self.host_to_scs(position)
        torque_l, torque_h = self.host_to_scs(torque)
        speed_l, speed_h = self.host_to_scs(speed)
        
        data = bytes([acc, pos_l, pos_h, torque_l, torque_h, speed_l, speed_h])
        return self.reg_write(servo_id, HLSCL_ACC, data)
    
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
        
        self.sync_write(servo_ids, HLSCL_ACC, data_list, 7)
    
    def sync_write_spe(self, servo_ids, speeds, accs=None, torques=None):
        """
        同步写多个舵机速度指令
        
        Args:
            servo_ids: 舵机ID列表
            speeds: 速度列表（带符号）
            accs: 加速度列表（可选）
            torques: 扭矩列表（必需）
        """
        if torques is None:
            raise ValueError("HLS舵机同步控制需要提供扭矩参数")
            
        servo_count = len(servo_ids)
        data_list = []
        
        for i in range(servo_count):
            speed = speeds[i]
            if speed < 0:
                speed = -speed
                speed |= (1 << 15)
            
            acc_val = accs[i] if accs else 0
            torque_val = torques[i]
            
            # 转换为字节
            pos_l, pos_h = self.host_to_scs(0)
            torque_l, torque_h = self.host_to_scs(torque_val)
            speed_l, speed_h = self.host_to_scs(speed)
            
            data_list.extend([acc_val, pos_l, pos_h, torque_l, torque_h, speed_l, speed_h])
        
        self.sync_write(servo_ids, HLSCL_ACC, data_list, 7)
    
    def wheel_mode(self, servo_id):
        """
        恒速模式
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, HLSCL_MODE, 1)
    
    def ele_mode(self, servo_id):
        """
        恒力模式
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, HLSCL_MODE, 2)
    
    def write_spe(self, servo_id, speed, acc=0, torque=0):
        """
        恒速模式控制指令
        
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
        恒力模式控制指令
        
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
        """
        扭力控制指令
        
        Args:
            servo_id: 舵机ID
            enable: 1=使能扭矩，0=关闭扭矩
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, HLSCL_TORQUE_ENABLE, enable)
    
    def unlock_eprom(self, servo_id):
        """
        EPROM解锁
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.enable_torque(servo_id, 0)
        return self.write_byte(servo_id, HLSCL_LOCK, 0)
    
    def lock_eprom(self, servo_id):
        """
        EPROM加锁
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, HLSCL_LOCK, 1)
    
    def calibration_ofs(self, servo_id):
        """
        中位校准
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        self.enable_torque(servo_id, 0)
        self.unlock_eprom(servo_id)
        return self.recal(servo_id)
    
    def feedback(self, servo_id):
        """
        反馈舵机信息
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        data = self.read(servo_id, HLSCL_PRESENT_POSITION_L, len(self.mem))
        if data is None:
            return False
        
        self.mem = list(data)
        return True
    
    def read_pos(self, servo_id=-1):
        """
        读位置
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 位置值（带符号），失败返回-1
        """
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
    
    def read_speed(self, servo_id=-1):
        """
        读速度
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 速度值（带符号），失败返回-1
        """
        if servo_id == -1:
            speed_h_idx = HLSCL_PRESENT_SPEED_H - HLSCL_PRESENT_POSITION_L
            speed_l_idx = HLSCL_PRESENT_SPEED_L - HLSCL_PRESENT_POSITION_L
            speed = self.scs_to_host(self.mem[speed_l_idx], self.mem[speed_h_idx])
        else:
            speed = self.read_word(servo_id, HLSCL_PRESENT_SPEED_L)
        
        if speed == -1:
            return -1
        
        # 处理符号位
        if speed & (1 << 15):
            speed = -(speed & ~(1 << 15))
        
        return speed
    
    def read_load(self, servo_id=-1):
        """
        读输出至电机的电压百分比(0~1000)
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 负载值（带符号），失败返回-1
        """
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
        """
        读电压
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 电压值，失败返回-1
        """
        if servo_id == -1:
            voltage_idx = HLSCL_PRESENT_VOLTAGE - HLSCL_PRESENT_POSITION_L
            return self.mem[voltage_idx]
        else:
            return self.read_byte(servo_id, HLSCL_PRESENT_VOLTAGE)
    
    def read_temper(self, servo_id=-1):
        """
        读温度
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 温度值，失败返回-1
        """
        if servo_id == -1:
            temp_idx = HLSCL_PRESENT_TEMPERATURE - HLSCL_PRESENT_POSITION_L
            return self.mem[temp_idx]
        else:
            return self.read_byte(servo_id, HLSCL_PRESENT_TEMPERATURE)
    
    def read_move(self, servo_id=-1):
        """
        读移动状态
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 移动状态，0=停止，1=移动中，失败返回-1
        """
        if servo_id == -1:
            move_idx = HLSCL_MOVING - HLSCL_PRESENT_POSITION_L
            return self.mem[move_idx]
        else:
            return self.read_byte(servo_id, HLSCL_MOVING)
    
    def read_current(self, servo_id=-1):
        """
        读电流
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 电流值（带符号），失败返回-1
        """
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
    
    def servo_mode(self, servo_id):
        """
        Servo模式
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, HLSCL_MODE, 0)