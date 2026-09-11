"""
scscl.py
飞特SCSCL系列串行舵机应用层程序
转换自 SCSCL.cpp 和 SCSCL.h
"""

from .scserial import SCSerial

# SCSCL内存表定义
# -------EPROM(只读)--------
SCSCL_VERSION_L = 3
SCSCL_VERSION_H = 4

# -------EPROM(读写)--------
SCSCL_ID = 5
SCSCL_BAUD_RATE = 6
SCSCL_MIN_ANGLE_LIMIT_L = 9
SCSCL_MIN_ANGLE_LIMIT_H = 10
SCSCL_MAX_ANGLE_LIMIT_L = 11
SCSCL_MAX_ANGLE_LIMIT_H = 12
SCSCL_CW_DEAD = 26
SCSCL_CCW_DEAD = 27

# -------SRAM(读写)--------
SCSCL_TORQUE_ENABLE = 40
SCSCL_GOAL_POSITION_L = 42
SCSCL_GOAL_POSITION_H = 43
SCSCL_GOAL_TIME_L = 44
SCSCL_GOAL_TIME_H = 45
SCSCL_GOAL_SPEED_L = 46
SCSCL_GOAL_SPEED_H = 47
SCSCL_LOCK = 48

# -------SRAM(只读)--------
SCSCL_PRESENT_POSITION_L = 56
SCSCL_PRESENT_POSITION_H = 57
SCSCL_PRESENT_SPEED_L = 58
SCSCL_PRESENT_SPEED_H = 59
SCSCL_PRESENT_LOAD_L = 60
SCSCL_PRESENT_LOAD_H = 61
SCSCL_PRESENT_VOLTAGE = 62
SCSCL_PRESENT_TEMPERATURE = 63
SCSCL_MOVING = 66
SCSCL_PRESENT_CURRENT_L = 69
SCSCL_PRESENT_CURRENT_H = 70

class SCSCL(SCSerial):
    """SCSCL系列串行舵机控制类"""
    
    def __init__(self, port=None, baudrate=115200, end=1, level=1, timeout=0.01):
        """
        初始化SCSCL舵机
        
        Args:
            port: 串口号
            baudrate: 波特率
            end: 大小端模式
            level: 返回等级
            timeout: 超时时间
        """
        super().__init__(port, baudrate, end, level, timeout)
        # 反馈数据缓冲区
        self.mem = [0] * (SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L + 1)
    
    def write_pos(self, servo_id, position, time=0, speed=0):
        """
        普通写单个舵机位置指令
        
        Args:
            servo_id: 舵机ID
            position: 目标位置 (0-1023)
            time: 执行时间 (单位：毫秒，0表示不限制)
            speed: 执行速度 (0-1023，0表示不限制)
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        # 将数据转换为字节
        pos_l, pos_h = self.host_to_scs(position)
        time_l, time_h = self.host_to_scs(time)
        speed_l, speed_h = self.host_to_scs(speed)
        
        data = bytes([pos_l, pos_h, time_l, time_h, speed_l, speed_h])
        return self.gen_write(servo_id, SCSCL_GOAL_POSITION_L, data)
    
    def reg_write_pos(self, servo_id, position, time=0, speed=0):
        """
        异步写单个舵机位置指令（RegWriteAction生效）
        
        Args:
            servo_id: 舵机ID
            position: 目标位置
            time: 执行时间
            speed: 执行速度
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        pos_l, pos_h = self.host_to_scs(position)
        time_l, time_h = self.host_to_scs(time)
        speed_l, speed_h = self.host_to_scs(speed)
        
        data = bytes([pos_l, pos_h, time_l, time_h, speed_l, speed_h])
        return self.reg_write(servo_id, SCSCL_GOAL_POSITION_L, data)
    
    def sync_write_pos(self, servo_ids, positions, times=None, speeds=None):
        """
        同步写多个舵机位置指令
        
        Args:
            servo_ids: 舵机ID列表
            positions: 位置列表
            times: 时间列表（可选）
            speeds: 速度列表（可选）
        """
        servo_count = len(servo_ids)
        data_list = []
        
        for i in range(servo_count):
            # 获取时间和速度值
            time_val = times[i] if times else 0
            speed_val = speeds[i] if speeds else 0
            
            # 转换为字节
            pos_l, pos_h = self.host_to_scs(positions[i])
            time_l, time_h = self.host_to_scs(time_val)
            speed_l, speed_h = self.host_to_scs(speed_val)
            
            data_list.extend([pos_l, pos_h, time_l, time_h, speed_l, speed_h])
        
        self.sync_write(servo_ids, SCSCL_GOAL_POSITION_L, data_list, 6)
    
    def enable_torque(self, servo_id, enable):
        """
        扭矩控制指令
        
        Args:
            servo_id: 舵机ID
            enable: 1=使能扭矩，0=关闭扭矩
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, SCSCL_TORQUE_ENABLE, enable)
    
    def unlock_eprom(self, servo_id):
        """
        EPROM解锁
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, SCSCL_LOCK, 0)
    
    def lock_eprom(self, servo_id):
        """
        EPROM加锁
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.write_byte(servo_id, SCSCL_LOCK, 1)
    
    def feedback(self, servo_id):
        """
        反馈舵机信息
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        data = self.read(servo_id, SCSCL_PRESENT_POSITION_L, len(self.mem))
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
            int: 位置值，失败返回-1
        """
        if servo_id == -1:
            # 从缓存读取
            pos_l_idx = SCSCL_PRESENT_POSITION_L - SCSCL_PRESENT_POSITION_L
            pos_h_idx = SCSCL_PRESENT_POSITION_H - SCSCL_PRESENT_POSITION_L
            return self.scs_to_host(self.mem[pos_l_idx], self.mem[pos_h_idx])
        else:
            return self.read_word(servo_id, SCSCL_PRESENT_POSITION_L)
    
    def read_speed(self, servo_id=-1):
        """
        读速度
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 速度值（带符号），失败返回-1
        """
        if servo_id == -1:
            speed_l_idx = SCSCL_PRESENT_SPEED_L - SCSCL_PRESENT_POSITION_L
            speed_h_idx = SCSCL_PRESENT_SPEED_H - SCSCL_PRESENT_POSITION_L
            speed = self.scs_to_host(self.mem[speed_l_idx], self.mem[speed_h_idx])
        else:
            speed = self.read_word(servo_id, SCSCL_PRESENT_SPEED_L)
        
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
            load_l_idx = SCSCL_PRESENT_LOAD_L - SCSCL_PRESENT_POSITION_L
            load_h_idx = SCSCL_PRESENT_LOAD_H - SCSCL_PRESENT_POSITION_L
            load = self.scs_to_host(self.mem[load_l_idx], self.mem[load_h_idx])
        else:
            load = self.read_word(servo_id, SCSCL_PRESENT_LOAD_L)
        
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
            voltage_idx = SCSCL_PRESENT_VOLTAGE - SCSCL_PRESENT_POSITION_L
            return self.mem[voltage_idx]
        else:
            return self.read_byte(servo_id, SCSCL_PRESENT_VOLTAGE)
    
    def read_temper(self, servo_id=-1):
        """
        读温度
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 温度值，失败返回-1
        """
        if servo_id == -1:
            temp_idx = SCSCL_PRESENT_TEMPERATURE - SCSCL_PRESENT_POSITION_L
            return self.mem[temp_idx]
        else:
            return self.read_byte(servo_id, SCSCL_PRESENT_TEMPERATURE)
    
    def read_move(self, servo_id=-1):
        """
        读移动状态
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 移动状态，0=停止，1=移动中，失败返回-1
        """
        if servo_id == -1:
            move_idx = SCSCL_MOVING - SCSCL_PRESENT_POSITION_L
            return self.mem[move_idx]
        else:
            return self.read_byte(servo_id, SCSCL_MOVING)
    
    def read_current(self, servo_id=-1):
        """
        读电流
        
        Args:
            servo_id: 舵机ID，-1表示从缓存读取
            
        Returns:
            int: 电流值（带符号），失败返回-1
        """
        if servo_id == -1:
            current_l_idx = SCSCL_PRESENT_CURRENT_L - SCSCL_PRESENT_POSITION_L
            current_h_idx = SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L
            current = self.scs_to_host(self.mem[current_l_idx], self.mem[current_h_idx])
        else:
            current = self.read_word(servo_id, SCSCL_PRESENT_CURRENT_L)
        
        if current == -1:
            return -1
        
        # 处理符号位
        if current & (1 << 15):
            current = -(current & ~(1 << 15))
        
        return current
    
    def pwm_mode(self, servo_id):
        """
        PWM模式
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        data = bytes([0, 0, 0, 0])
        return self.gen_write(servo_id, SCSCL_MIN_ANGLE_LIMIT_L, data)
    
    def write_pwm(self, servo_id, pwm_out):
        """
        PWM输出模式指令
        
        Args:
            servo_id: 舵机ID
            pwm_out: PWM输出值（带符号）
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        if pwm_out < 0:
            pwm_out = -pwm_out
            pwm_out |= (1 << 10)
        
        pwm_l, pwm_h = self.host_to_scs(pwm_out)
        data = bytes([pwm_l, pwm_h])
        return self.gen_write(servo_id, SCSCL_GOAL_TIME_L, data)
        