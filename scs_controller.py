#!/usr/bin/env python3
"""
SCSCL 多功能舵机控制程序 (仅支持串口连接)

结构:
1. BasicServoController (基础父类):
   - 提供最底层的、基于舵机原始位置值 (position) 的操作。
   - 兼容串口 (SCSCL) 实例。

2. MultiAngleServoController (高级子类):
   - 继承 BasicServoController，提供更高级的、基于角度 (degree) 的控制。
   - 增加了角度/位置转换、多圈支持、时间速度计算和交互式监控功能。

作者: Gemini LLM (基于用户提供的代码整合与优化)
"""

import time
import threading
import math
from typing import Union, List, Dict, Any
from FTservo_py import SCSCL  # 只导入 SCSCL，WiFi 相关的已移除

CommTool = SCSCL  # 使用 SCSCL 进行串口通信


# ====================================================================
# CLASS 1: THE GENERIC BASE CONTROLLER (Position-Based)
# ====================================================================

import time
from typing import List, Dict, Any, Union


class BasicServoController:
    """
    通用的舵机控制系统.
    封装了单个或多个舵机的移动、读取状态、模式切换等基础功能。
    """

    def __init__(self, communication_tool):
        """
        初始化舵机控制器。

        Args:
            communication_tool: 用于舵机通信的工具（例如：SCSCL实例）。
        """
        self.sc = communication_tool
        print("BasicServoController (通用舵机控制器) 已初始化。")

    # --------------------------------------------------------------------
    # 1. 核心移动功能 (阻塞式 - 等待完成)
    # --------------------------------------------------------------------

    def move_servo_wait(self, servo_id: int, target_position: int, speed: int, acc: int = 50, torque: int = 300,
                        tolerance: int = 50, max_attempts: int = 30, wait_time: float = 0.1) -> bool:
        """
        移动单个舵机到指定位置，并阻塞直到到达或超时。

        Args:
            servo_id: 舵机 ID
            target_position: 目标位置
            speed: 移动速度
            acc: 加速度（不再使用，但可为后续扩展保留）
            torque: 扭矩
            tolerance: 目标位置的误差范围
            max_attempts: 最大重试次数
            wait_time: 每次重试的等待时间

        Returns:
            bool: 是否成功到达目标位置
        """
        print(f"  [检测] 正在移动舵机 {servo_id} 到 {target_position}...")

        # 初始指令
        self.sc.write_pos(servo_id, target_position, 0, speed)

        for attempt in range(max_attempts):
            time.sleep(wait_time)
            actual_pos = self.sc.read_pos(servo_id)

            if actual_pos == -1:
                print(f"    [尝试 {attempt + 1}/{max_attempts}] 读取位置失败，重发指令...")
                self.sc.write_pos(servo_id, target_position, 0, speed)
                continue

            if abs(actual_pos - target_position) <= tolerance:
                print(f"  [成功] 舵机 {servo_id} 已到达位置 {actual_pos} (目标: {target_position}).")
                return True
            else:
                print(
                    f"    [尝试 {attempt + 1}/{max_attempts}] 当前: {actual_pos}, 目标: {target_position}. 重发指令...")

        final_pos = self.sc.read_pos(servo_id)
        print(f"  [警告] 舵机 {servo_id} 超时未到达。最终位置: {final_pos}, 目标: {target_position}.")
        return False

    def sync_move_servos_wait(self, servo_ids: List[int], target_positions: List[int], speeds: List[int],
                              accs: List[int], torques: List[int], tolerance: int = 50, max_attempts: int = 10,
                              wait_time: float = 0.1) -> bool:
        """
        同步移动多个舵机，并阻塞直到全部到达或超时。

        Args:
            servo_ids: 舵机 ID 列表
            target_positions: 目标位置列表
            speeds: 速度列表
            accs: 加速度列表
            torques: 扭矩列表
            tolerance: 目标位置的误差范围
            max_attempts: 最大重试次数
            wait_time: 每次重试的等待时间

        Returns:
            bool: 是否成功同步完成移动
        """
        print(f"  [同步检测] 正在移动舵机 {servo_ids} 到 {target_positions}...")
        num_servos = len(servo_ids)
        reached = [False] * num_servos

        # 初始同步写入指令
        self.sc.sync_write_pos(servo_ids, target_positions, speeds, accs, torques)

        for attempt in range(max_attempts):
            if all(reached):
                break

            time.sleep(wait_time)
            all_reached_this_cycle = True

            for i in range(num_servos):
                if reached[i]:
                    continue

                sid = servo_ids[i]
                target_pos = target_positions[i]
                actual_pos = self.sc.read_pos(sid)

                if actual_pos != -1 and abs(actual_pos - target_pos) <= tolerance:
                    reached[i] = True
                    print(f"    [成功] 舵机 {sid} 已到达 {actual_pos}.")
                else:
                    all_reached_this_cycle = False
                    if actual_pos == -1:
                        print(f"    [尝试 {attempt + 1}] 读取舵机 {sid} 位置失败...")
                    else:
                        print(f"    [尝试 {attempt + 1}] 舵机 {sid} 当前: {actual_pos}, 目标: {target_pos}...")

            if not all(reached):
                print(f"    [重发] 重新发送同步移动指令...")
                self.sc.sync_write_pos(servo_ids, target_positions, speeds, accs, torques)

        if all(reached):
            print("  [成功] 所有舵机均已到达目标位置。")
            return True
        else:
            not_reached_servos = [servo_ids[i] for i, r in enumerate(reached) if not r]
            print(f"  [警告] 超时！以下舵机未到达目标位置: {not_reached_servos}")
            return False

    # --------------------------------------------------------------------
    # 2. 核心移动功能 (非阻塞式 - 立即返回)
    # --------------------------------------------------------------------

    def move_servo(self, servo_id: int, target_position: int, speed: int, torque: int = 500):
        """
        非阻塞地移动单个舵机到指定位置。

        Args:
            servo_id: 舵机 ID
            target_position: 目标位置值
            speed: 移动速度
            torque: 扭矩（可选，默认为 500）

        Returns:
            bool: 返回是否成功执行
        """
        print(f"  [指令] 移动舵机 {servo_id} -> {target_position} (速度: {speed}, 扭矩: {torque})")
        return self.sc.write_pos(servo_id, target_position, 0, speed)

    def sync_move_servos(self, servo_ids: List[int], positions: List[int], speeds: List[int], accs: List[int],
                         torques: List[int]):
        """
        非阻塞地同步移动多个舵机。

        Args:
            servo_ids: 舵机 ID 列表
            positions: 目标位置列表
            speeds: 速度列表
            accs: 加速度列表
            torques: 扭矩列表

        Returns:
            bool: 返回是否成功执行
        """
        print(f"  [同步指令] 移动舵机 {servo_ids} -> {positions}")
        return self.sc.sync_write_pos(servo_ids, positions, speeds, accs, torques)

    # --------------------------------------------------------------------
    # 3. 状态读取功能
    # --------------------------------------------------------------------

    def get_full_status(self, servo_id: int) -> Union[Dict[str, Any], None]:
        """
        读取单个舵机的完整当前状态（支持多圈）。

        Args:
            servo_id: 舵机 ID

        Returns:
            dict: 包含舵机位置、负载、电压、温度、是否移动、电流等信息的字典
        """
        # 读取反馈信息
        if not self.sc.feedback(servo_id):
            print(f"  [错误] 舵机 {servo_id} 反馈失败")
            return None

        # 从缓存读取各项数据
        current_position = self.sc.read_pos(-1)  # -1表示从缓存读取
        current_load = self.sc.read_load(-1)
        current_voltage = self.sc.read_voltage(-1)
        current_temp = self.sc.read_temper(-1)
        is_moving = self.sc.read_move(-1)
        current_current = self.sc.read_current(-1)

        if current_position == -1:
            print(f"  [错误] 从舵机 {servo_id} 缓存中读取位置失败")
            return None

        return {
            'servo_id': servo_id,
            'position': current_position,
            'load': current_load,
            'voltage': current_voltage,
            'temperature': current_temp,
            'moving': is_moving,
            'current': current_current
        }

    def get_position(self, servo_id: int) -> int:
        """读取舵机当前位置。"""
        return self.sc.read_pos(servo_id)

    # --------------------------------------------------------------------
    # 4. 配置和模式功能
    # --------------------------------------------------------------------

    def enable_torque(self, servo_id: int, enable: bool):
        """
        使能或禁用舵机的扭矩。

        Args:
            servo_id: 舵机 ID
            enable: 1=使能扭矩，0=关闭扭矩

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        val = 1 if enable else 0
        status = "使能" if enable else "禁用"
        print(f"  [配置] {status}舵机 {servo_id} 的扭矩")
        return self.sc.enable_torque(servo_id, val)

    def calibrate_zero_offset(self, servo_id: int):
        """
        标定舵机的零位偏移量（将其当前位置设为零点）。

        Args:
            servo_id: 舵机 ID
        """
        print(f"  [配置] 正在标定舵机 {servo_id} 的零位偏移...")
        self.sc.calibration_ofs(servo_id)
        time.sleep(0.01)
        print(f"  [配置] 舵机 {servo_id} 标定完成")

    # --------------------------------------------------------------------
    # 5. 总线工具
    # --------------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """
        Ping 一个舵机，检查它是否存在。

        Args:
            servo_id: 舵机 ID

        Returns:
            bool: 返回是否存在
        """
        if hasattr(self.sc, 'ping'):
            result = self.sc.ping(servo_id)
        else:
            result = self.sc.read_pos(servo_id)

        return result != -1

    def scan_bus(self, start_id: int = 1, end_id: int = 20) -> List[int]:
        """
        扫描总线上的舵机。

        Args:
            start_id: 扫描的开始 ID
            end_id: 扫描的结束 ID

        Returns:
            List[int]: 发现的舵机 ID 列表
        """
        print(f"\n=== 扫描舵机总线 (ID {start_id} - {end_id}) ===")
        found_servos = []

        for servo_id in range(start_id, end_id + 1):
            try:
                if self.ping(servo_id):
                    found_servos.append(servo_id)
                    print(f"发现舵机ID: {servo_id}")
                else:
                    print(f". (ID {servo_id} 未响应)")

                time.sleep(0.005)  # 小延迟避免通信过快

            except Exception as e:
                print(f"扫描ID {servo_id} 时出错: {e}")
                continue

        print("\n=== 扫描结果 ===")
        if not found_servos:
            print("未发现舵机。请检查连接和电源。")
        else:
            print(f"发现 {len(found_servos)} 个舵机，ID: {found_servos}")
        print("==================")
        return found_servos

    def move_servo_with_load(self, servo_id: int, target_position: int, speed: int, torque: int = 500, max_load: int = 1000, wait_time: float = 0.1) -> bool:
        """
        带负载控制的舵机移动函数。

        Args:
            servo_id: 舵机 ID
            target_position: 目标位置
            speed: 移动速度
            torque: 扭矩
            max_load: 最大允许负载，超过则停止
            wait_time: 每次检测间隔时间

        Returns:
            bool: 是否成功执行
        """
        print(f"  [指令] 正在移动舵机 {servo_id} 到位置 {target_position} (速度: {speed}, 扭矩: {torque})")

        # 开始移动舵机
        self.sc.write_pos(servo_id, target_position, 0, speed)

        # 监测负载，直到舵机到达目标位置或超负载
        while True:
            # 获取舵机的当前位置
            current_position = self.sc.read_pos(servo_id)
            if current_position == -1:
                print(f"  [错误] 无法读取舵机 {servo_id} 当前位置！")
                return False

            # 获取舵机的负载
            current_load = self.sc.read_load(servo_id)

            # 如果负载超过最大负载限制，则停止舵机
            if current_load > max_load:
                print(f"  [警告] 舵机 {servo_id} 超过负载限制 {max_load}, 当前负载: {current_load}. 停止舵机。")
                self.sc.write_pos(servo_id, current_position, 0, 0)  # 停止舵机
                return False

            # 检查是否达到目标位置（可加上容错范围）
            if abs(current_position - target_position) <= 10:  # tolerance can be adjusted
                print(f"  [成功] 舵机 {servo_id} 已到达目标位置 {current_position}.")
                return True

            # 等待一段时间，继续检测
            time.sleep(wait_time)

    # 其他方法
    def read_pos(self, servo_id: int) -> int:
        return self.sc.read_pos(servo_id)

    def read_load(self, servo_id: int) -> int:
        return self.sc.read_load(servo_id)

    def write_pos(self, servo_id: int, position: int, time: int, speed: int):
        return self.sc.write_pos(servo_id, position, time, speed)

class MultiAngleServoController(BasicServoController):
    """
    高级多SCSCL舵机控制器 (继承自 BasicServoController)
    增加了角度控制、多圈支持、时间控制和交互菜单。
    """

    def __init__(self, comm_address: str, comm_port: int, servo_ids=[1, 2]):
        """
        初始化多SCSCL舵机控制器

        Args:
            comm_address (str): 串口号 (如 'COM3') 或 IP地址 (如 '192.168.1.1')
            comm_port (int): 串口波特率 (如 1000000)
            servo_ids: 舵机ID列表
        """
        print(f"初始化 串口连接: Port={comm_address}, Baudrate={comm_port}")
        comm_tool = SCSCL(comm_address, comm_port)

        # 调用父类的 __init__ 方法，将通信工具传递给它
        super().__init__(comm_tool)

        # 初始化变量
        self.servo_ids = servo_ids
        self.SERVO_MAX_SPEED = 500  # 默认最大速度（SCSCL速度单位）
        self.SERVO_DEFAULT_ACC = 200  # 默认加速度（SCSCL加速度单位）
        self.SERVO_DEFAULT_TORQUE = 500  # 默认扭矩

        # 角度范围常量（支持多圈旋转舵机）
        self.INPUT_MIN_ANGLE = -2520.0  # 输入最小角度(-7*360)
        self.INPUT_MAX_ANGLE = 2520.0  # 输入最大角度(7*360)

        # 单位转换常量
        self.POSITION_TO_DEGREE = 360.0 / 4096.0  # 位置单位到角度的转换比例
        self.DEGREE_TO_POSITION = 4096.0 / 360.0  # 角度到位置单位的转换比例
        self.HLSCL_MIN_POSITION = -28672  # 最小位置值（精确-7圈）
        self.HLSCL_MAX_POSITION = 28672  # 最大位置值（精确+7圈）

        # 初始化监控线程相关属性
        self.monitor_thread = None
        self.is_monitoring = False

    # --------------------------------------------------------------------
    # 1. 角度/位置/速度 转换
    # --------------------------------------------------------------------

    def angle_to_position(self, angle: float) -> int:
        """
        将角度转换为位置值（支持多圈）

        Args:
            angle: 目标角度

        Returns:
            int: 对应的舵机位置值
        """
        angle = max(self.INPUT_MIN_ANGLE, min(self.INPUT_MAX_ANGLE, angle))
        position = int(round(angle * self.DEGREE_TO_POSITION))
        return max(self.HLSCL_MIN_POSITION, min(self.HLSCL_MAX_POSITION, position))

    def position_to_angle(self, position: int) -> float:
        """
        将位置值转换为角度（支持多圈）

        Args:
            position: 舵机位置值

        Returns:
            float: 对应的角度
        """
        return position * self.POSITION_TO_DEGREE

    def calculate_speed(self, angle_diff: float, time_seconds: float) -> int:
        """
        基于时间计算SCSCL速度值

        Args:
            angle_diff: 目标角度与当前位置的差值
            time_seconds: 目标时间

        Returns:
            int: 计算出的速度值
        """
        if time_seconds <= 0:
            time_seconds = 0.1
        if angle_diff <= 0:
            return 1

        print(f" 速度计算: 角度差={angle_diff:.1f}°, 时间={time_seconds:.1f}s")

        # 计算物理最大加速度 (度/秒²)
        amax_physical = self.SERVO_DEFAULT_ACC * 8.7  # SCSCL加速度转换为度/秒²

        # 简化计算：假设梯形速度曲线，加速时间为总时间的30%
        accel_time = time_seconds * 0.3
        const_time = time_seconds * 0.4

        # 恒速阶段的距离
        const_distance = angle_diff - 0.5 * amax_physical * accel_time * accel_time * 2

        if const_distance <= 0:
            # 纯三角速度曲线
            max_velocity = angle_diff / time_seconds  # 简化计算，实际公式更复杂
        else:
            # 梯形速度曲线
            max_velocity = const_distance / const_time + amax_physical * accel_time

        # 转换为SCSCL速度值
        scscl_speed = int(round(max_velocity * 0.732 * 6.0))  # 转换为度/秒
        scscl_speed = max(1, min(self.SERVO_MAX_SPEED, scscl_speed))

        print(f"   转换后SCSCL速度: {scscl_speed}")

        return scscl_speed

    # --------------------------------------------------------------------
    # 2. 高级连接和移动
    # --------------------------------------------------------------------

    def connect(self) -> bool:
        """连接所有舵机并进行初始化"""
        if not self.sc.is_open():
            print("通信连接失败！请检查参数或线路。")
            return False

        print("正在连接舵机...")

        # 连接并初始化每个舵机
        connected_servos = []
        for servo_id in self.servo_ids:
            if self.ping(servo_id):
                print(f" 舵机ID {servo_id} 连接成功！")
                connected_servos.append(servo_id)

                # 如果没有定义 set_servo_mode，就去掉这行
                # if not self.set_servo_mode(servo_id):
                #     print(f" 舵机ID {servo_id} 设置Servo模式失败，但继续运行...")

                if not self.enable_torque(servo_id, True):
                    print(f" 舵机ID {servo_id} 使能扭矩失败，但继续运行...")
            else:
                print(f" 舵机ID {servo_id} 无响应！")

        if not connected_servos:
            print(" 没有舵机响应！请检查连接和ID设置。")
            return False

        self.servo_ids = connected_servos
        print(f" 成功连接 {len(self.servo_ids)} 个舵机: {self.servo_ids}")
        return True

    def move_single_servo(self, servo_id: int, target_angle: float, speed: int = None, torque: int = 500) -> bool:
        """
        移动单个舵机到指定角度（角度控制）

        Args:
            servo_id: 舵机 ID
            target_angle: 目标角度
            speed: 移动速度
            torque: 扭矩（可选，默认为 500）

        Returns:
            bool: 是否成功执行
        """
        if speed is None:
            speed = self.SERVO_MAX_SPEED  # 默认最大速度

        # 计算目标位置
        target_position = self.angle_to_position(target_angle)
        print(f"  [指令] 移动舵机 {servo_id} -> {target_position} (速度: {speed}, 扭矩: {torque})")

        # 调用父类的 move_servo 方法
        return self.move_servo(servo_id, target_position, speed, torque)

    def move_single_servo_with_time(self, servo_id: int, target_angle: float, time_seconds: float, torque: int = 500) -> bool:
        """
        基于时间控制单个舵机运动（角度控制）

        Args:
            servo_id: 舵机 ID
            target_angle: 目标角度
            time_seconds: 运动时间
            torque: 扭矩（可选，默认为 500）

        Returns:
            bool: 是否成功执行
        """
        if target_angle < self.INPUT_MIN_ANGLE or target_angle > self.INPUT_MAX_ANGLE:
            print(f"角度超出范围！支持范围: {self.INPUT_MIN_ANGLE}° ~ {self.INPUT_MAX_ANGLE}°")
            return False

        # 计算角度差
        current_status = self.read_servo_status(servo_id)
        if not current_status:
            print(f" 无法读取舵机{servo_id}当前位置！")
            return False

        current_angle = current_status['angle']
        angle_diff = abs(target_angle - current_angle)

        # 计算所需速度
        calculated_speed = self.calculate_speed(angle_diff, time_seconds)

        print(f" 舵机{servo_id}: {current_angle:.1f}° -> {target_angle:.1f}° "
              f"(差值: {angle_diff:.1f}°, 时间: {time_seconds:.1f}s, 计算速度: {calculated_speed})")

        # 调用 move_single_servo 来执行
        return self.move_single_servo(servo_id, target_angle, calculated_speed, torque)

    def move_single_servo_with_load(
        self,
        servo_id: int,
        target_angle: float,
        speed: int = None,
        torque: int = 500,
        max_load: int = 990,
        timeout_s: float = 3.0,
        stall_timeout_s: float = 0.8,
    ) -> bool:
        """
        移动单个舵机到指定角度，并进行负载控制。

        Args:
            servo_id: 舵机 ID
            target_angle: 目标角度
            speed: 移动速度
            torque: 扭矩（可选，默认为 500）
            max_load: 最大负载限制，超载时停止舵机（可选，默认为 1000）

        Returns:
            bool: 是否成功执行
        """
        if speed is None:
            speed = 1500  # 默认最大速度

        # 计算目标位置
        target_position = self.angle_to_position(target_angle)
        print(
            f"  [指令] 移动舵机 {servo_id} -> {target_position} (速度: {speed}, 扭矩: {torque}, 最大负载: {max_load})")

        # 开始移动舵机
        self.sc.write_pos(servo_id, target_position, 0, speed)

        # 监测负载，直到舵机到达目标位置、超负载、超时或卡住无进展
        start_time = time.time()
        last_progress_time = start_time
        last_position = None
        position_tolerance = 50
        min_progress_delta = 3

        while True:
            now = time.time()

            # 获取舵机的当前位置
            current_position = self.sc.read_pos(servo_id)
            if current_position == -1:
                print(f"  [错误] 无法读取舵机 {servo_id} 当前位置！")
                return False

            # 获取舵机的负载
            current_load = self.sc.read_load(servo_id)

            # 检查负载的绝对值是否超过最大负载限制
            if abs(current_load) > max_load:
                print(f"  [警告] 舵机 {servo_id} 超过负载限制 {max_load}, 当前负载: {current_load}. 停止舵机。")
                self.sc.write_pos(servo_id, current_position, 0, 0)  # 停止舵机
                return True

            # 检查是否达到目标位置（可加上容错范围）
            if abs(current_position - target_position) <= position_tolerance:
                print(f"  [成功] 舵机 {servo_id} 已到达目标位置 {current_position}.")
                return True

            # 检测位置是否在推进，避免“已经停了但线程不退出”
            if last_position is None or abs(current_position - last_position) >= min_progress_delta:
                last_progress_time = now
            last_position = current_position

            if now - start_time > timeout_s:
                print(
                    f"  [超时] 舵机 {servo_id} 在 {timeout_s:.1f}s 内未到达目标，当前位置: {current_position}, 目标: {target_position}"
                )
                self.sc.write_pos(servo_id, current_position, 0, 0)
                return False

            if now - last_progress_time > stall_timeout_s:
                print(
                    f"  [卡住] 舵机 {servo_id} 连续 {stall_timeout_s:.1f}s 无位置变化，当前位置: {current_position}, 目标: {target_position}"
                )
                self.sc.write_pos(servo_id, current_position, 0, 0)
                return False

            # 等待一段时间，继续检测
            time.sleep(0.1)

    # --------------------------------------------------------------------
    # 3. 状态读取和监控
    # --------------------------------------------------------------------

    def read_servo_status(self, servo_id: int) -> Union[Dict[str, Any], None]:
        """
        读取单个舵机的当前状态（添加角度转换）

        Args:
            servo_id: 舵机 ID

        Returns:
            dict: 包含舵机状态信息（位置、负载、电压、电流等）
        """
        status_dict = super().get_full_status(servo_id)

        if status_dict and status_dict['position'] != -1:
            status_dict['angle'] = self.position_to_angle(status_dict['position'])
            return status_dict
        else:
            return None

    def read_all_servos_status(self) -> Dict[int, Dict[str, Any]]:
        """
        读取所有舵机的状态信息

        Returns:
            dict: 包含所有舵机状态的字典
        """
        all_status = {}
        for servo_id in self.servo_ids:
            status = self.read_servo_status(servo_id)
            if status:
                all_status[servo_id] = status
        return all_status

    def start_multi_servo_monitor(self):
        """启动多舵机监控线程"""
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._multi_servo_monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_monitor(self):
        """停止监控"""
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=0.2)

    def _multi_servo_monitor_loop(self):
        """多舵机监控循环（在后台线程中运行）"""
        movement_stopped_counts = {servo_id: 0 for servo_id in self.servo_ids}

        while self.is_monitoring:
            all_status = self.read_all_servos_status()

            if all_status:
                status_line = " "
                all_stopped = True

                for servo_id in self.servo_ids:
                    if servo_id in all_status:
                        status = all_status[servo_id]
                        status_line += f"舵机{servo_id}[{status['angle']:7.1f}°,负载:{status['load']:4d},移动:{'是' if status['moving'] else '否'}] "

                        if not status['moving']:
                            movement_stopped_counts[servo_id] += 1
                        else:
                            movement_stopped_counts[servo_id] = 0
                            all_stopped = False
                    else:
                        status_line += f"舵机{servo_id}[读取失败] "
                        all_stopped = False

                print(status_line)

                if all_stopped and all(count >= 3 for count in movement_stopped_counts.values()):
                    print("所有舵机运动完成！状态监控已停止。")
                    self.is_monitoring = False
                    break
            else:
                print(" 读取状态失败")

            time.sleep(0.1)

    def show_control_menu(self):
        """显示控制菜单"""
        print("\n" + "=" * 80)
        print("多SCSCL舵机交互式角度控制（支持多圈±7圈）")
        print("=" * 80)
        print(f" 已连接舵机: {self.servo_ids}")
        print(f" 角度范围: {self.INPUT_MIN_ANGLE}° ~ {self.INPUT_MAX_ANGLE}° (±7圈)")
        print(f" 速度范围: 1 ~ {self.SERVO_MAX_SPEED}")
        print("\n控制选项:")
        print("单独控制 - 输入格式: servo_id angle  (例: 1 90)")
        print("单独时间控制 - 输入格式: servo_id angle time  (例: 1 720 5.0)")
        print("状态查询 - 输入: status")
        print("退出程序 - 输入: q")
        print(" 支持多圈角度，如720°=2圈，-720°=-2圈")
        print("-" * 80)

    def run_interactive_control(self):
        """运行交互式控制"""
        self.show_control_menu()

        print(" 读取当前状态...")
        all_status = self.read_all_servos_status()
        for servo_id, status in all_status.items():
            if status:
                print(f" 舵机{servo_id}: 角度{status['angle']:.1f}°, 负载{status['load']}, "
                      f"电压{status['voltage'] / 10:.1f}V, 温度{status['temperature']}°C")

        print("\n 开始交互式控制...")

        while True:
            try:
                user_input = input(f"\n请输入指令 (舵机数:{len(self.servo_ids)}): ").strip()

                if user_input.lower() == 'q':
                    print(" 退出程序...")
                    break

                if user_input.lower() == 'status':
                    all_status = self.read_all_servos_status()
                    print("\n 当前状态:")
                    for servo_id, status in all_status.items():
                        if status:
                            turns = status['angle'] / 360.0
                            print(f"   舵机{servo_id}: {status['angle']:.1f}° ({turns:+.2f}圈), "
                                  f"负载:{status['load']}, 电流:{status['current']}mA, "
                                  f"移动:{'是' if status['moving'] else '否'}")
                    continue

                parts = user_input.split()

                if len(parts) == 2:
                    try:
                        servo_id = int(parts[0])
                        angle = float(parts[1])

                        if servo_id not in self.servo_ids:
                            print(f" 舵机ID {servo_id} 未连接！可用ID: {self.servo_ids}")
                            continue

                        self.stop_monitor()
                        success = self.move_single_servo_with_load(servo_id, angle)
                        if success:
                            self.start_multi_servo_monitor()
                            if self.monitor_thread:
                                self.monitor_thread.join()
                            print(" 可以输入下一个指令了！")

                    except ValueError:
                        print(" 单独控制格式错误！正确格式: servo_id angle (例: 1 720)")

                elif len(parts) == 3:
                    try:
                        servo_id = int(parts[0])
                        angle = float(parts[1])
                        time_seconds = float(parts[2])

                        if servo_id not in self.servo_ids:
                            print(f" 舵机ID {servo_id} 未连接！可用ID: {self.servo_ids}")
                            continue

                        if time_seconds <= 0:
                            print(" 时间必须大于0秒！")
                            continue

                        self.stop_monitor()
                        success = self.move_single_servo_with_time(servo_id, angle, time_seconds)
                        if success:
                            self.start_multi_servo_monitor()
                            if self.monitor_thread:
                                self.monitor_thread.join()
                            print(" 可以输入下一个指令了！")

                    except ValueError:
                        print(" 单独时间控制格式错误！正确格式: servo_id angle time (例: 1 720 5.0)")

                else:
                    print(f" 输入格式错误！请参考菜单。")

            except KeyboardInterrupt:
                print("\n程序被中断，正在退出...")
                break
            except Exception as e:
                print(f" 发生错误: {e}")

    def disconnect(self):
        """断开连接"""
        print(" 正在断开连接...")
        self.stop_monitor()

        for servo_id in self.servo_ids:
            try:
                self.enable_torque(servo_id, False)
                print(f" 舵机{servo_id} 已关闭扭矩")
            except:
                pass

        self.sc.close()
        print(" 串口已关闭")

# ====================================================================
# MAIN EXECUTION
# ====================================================================

def main():
    """主函数"""
    print("多SCSCL舵机角度控制程序启动（支持多圈±7圈）")

    # ====================================================================
    # --- 配置参数 (请选择串口连接配置并修改参数) ---
    # ====================================================================

    # 串口连接配置
    PORT = '/dev/ttyUSB0'  # Windows下如 'COM3', Linux下如 '/dev/ttyUSB0'
    BAUDRATE = 1000000

    SERVO_IDS = [1]  # 要控制的舵机ID列表

    # ====================================================================

    # 初始化 MultiAngleServoController
    controller = MultiAngleServoController(
        comm_address=PORT,
        comm_port=BAUDRATE,
        servo_ids=SERVO_IDS
    )

    try:
        # 连接舵机并进行初始化
        if not controller.connect():
            return

        # 运行交互式控制
        controller.run_interactive_control()

    except Exception as e:
        print(f" 程序异常: {e}")

    finally:
        # 程序结束时断开与舵机的连接
        controller.disconnect()


if __name__ == "__main__":
    main()

# 1 90 闭合
# 1 30 打开
