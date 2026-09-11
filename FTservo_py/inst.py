"""
inst.py
飞特串行舵机协议指令定义
转换自 INST.h
"""

from enum import IntEnum

# 数据类型定义 (Python中不需要显式定义，但保留注释)
# s8 = signed 8-bit integer
# u8 = unsigned 8-bit integer  
# s16 = signed 16-bit integer
# u16 = unsigned 16-bit integer
# s32 = signed 32-bit integer
# u32 = unsigned 32-bit integer

class SCSError(IntEnum):
    """SCS错误码枚举"""
    ERR_NO_REPLY = 1    # 无应答
    ERR_CRC_CMP = 2     # CRC校验错误
    ERR_SLAVE_ID = 3    # 从机ID错误
    ERR_BUFF_LEN = 4    # 缓冲区长度错误

# 指令定义
INST_PING = 0x01        # Ping指令
INST_READ = 0x02        # 读指令
INST_WRITE = 0x03       # 写指令
INST_REG_WRITE = 0x04   # 寄存器写指令
INST_REG_ACTION = 0x05  # 寄存器执行指令
INST_SYNC_READ = 0x82   # 同步读指令
INST_SYNC_WRITE = 0x83  # 同步写指令
INST_RECOVERY = 0x06    # 恢复指令
INST_RESET = 0x0A       # 重置指令
INST_CAL = 0x0B         # 校准指令

# 波特率定义
class BaudRate(IntEnum):
    """波特率枚举"""
    _1M = 0         # 1000000
    _0_5M = 1       # 500000
    _250K = 2       # 250000
    _128K = 3       # 128000
    _115200 = 4     # 115200
    _76800 = 5      # 76800
    _57600 = 6      # 57600
    _38400 = 7      # 38400
    _19200 = 8      # 19200
    _14400 = 9      # 14400
    _9600 = 10      # 9600
    _4800 = 11      # 4800

# 波特率映射表
BAUD_RATE_MAP = {
    BaudRate._1M: 1000000,
    BaudRate._0_5M: 500000,
    BaudRate._250K: 250000,
    BaudRate._128K: 128000,
    BaudRate._115200: 115200,
    BaudRate._76800: 76800,
    BaudRate._57600: 57600,
    BaudRate._38400: 38400,
    BaudRate._19200: 19200,
    BaudRate._14400: 14400,
    BaudRate._9600: 9600,
    BaudRate._4800: 4800
}