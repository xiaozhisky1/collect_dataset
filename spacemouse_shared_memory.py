import multiprocessing as mp
import numpy as np
import time
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent # spnav 模块处理空间鼠标的事件
from shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer # 导入自定义的共享内存环形缓冲区类
# --------Spacemouse 类实现了一个从空间鼠标接收输入并将其状态更新到共享内存的完整流程-----------
class Spacemouse(mp.Process):# Spacemouse 继承自 mp.Process，使其能够在独立进程中运行
    def __init__(self, 
            shm_manager,
            get_max_k=30,
            frequency=200,
            max_value=500, 
            deadzone=(0,0,0,0,0,0), 
            dtype=np.float32,
            n_buttons=2,
            ): # 初始化参数包括共享内存管理器、最大缓冲区大小、频率、最大值、死区、数据类型和按钮数量。
        """
        Continuously listen to 3D connection space naviagtor events
        and update the latest state.

        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
        
        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        super().__init__()
        if np.issubdtype(type(deadzone), np.number): # 根据 deadzone 的类型设置为一个包含六个元素的数组，确保所有死区值为非负数
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        # copied variables
        self.frequency = frequency
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.n_buttons = n_buttons
        # self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        # self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=dtype)

        example = { # 定义一个示例数据结构，用于初始化环形缓冲区
            # 3 translation, 3 rotation, 1 period
            'motion_event': np.zeros((7,), dtype=np.int64),
            # left and right button
            'button_state': np.zeros((n_buttons,), dtype=bool),
            'receive_timestamp': time.time()
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples( # 使用 SharedMemoryRingBuffer 创建一个共享内存环形缓冲区，配置其参数
            shm_manager=shm_manager, 
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        # shared variables 初始化两个事件：ready_event 用于表示类是否准备就绪，stop_event 用于控制进程的停止。
        self.ready_event = mp.Event()
        self.stop_event = mp.Event()
        self.ring_buffer = ring_buffer

    # ======= get state APIs ==========

    def get_motion_state(self): # get_motion_state 方法从环形缓冲区获取运动状态，将其归一化并应用死区处理
        state = self.ring_buffer.get()
        state = np.array(state['motion_event'][:6], 
            dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state
    
    def get_motion_state_transformed(self): # 将运动状态转换为右手坐标系
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        return tf_state

    def get_button_state(self): # 获取按钮状态
        state = self.ring_buffer.get()
        return state['button_state']
    
    def is_button_pressed(self, button_id):
        return self.get_button_state()[button_id]
    
    #========== start stop API ===========

    def start(self, wait=True): # 启动进程，并可选择等待进程准备就绪
        super().start()
        if wait:
            self.ready_event.wait()
    
    def stop(self, wait=True): # 设置停止事件，并可选择等待进程结束
        self.stop_event.set()
        if wait:
            self.join()
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= main loop ==========
    def run(self):
        spnav_open() # 首先打开空间鼠标设备
        try:
            motion_event = np.zeros((7,), dtype=np.int64)
            button_state = np.zeros((self.n_buttons,), dtype=bool)
            # send one message immediately so client can start reading
            self.ring_buffer.put({
                'motion_event': motion_event,
                'button_state': button_state,
                'receive_timestamp': time.time()
            })
            self.ready_event.set()
            # 循环中使用 spnav_poll_event() 获取鼠标事件，更新运动状态和按钮状态，并将这些状态存入环形缓冲区。
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                receive_timestamp = time.time()
                if isinstance(event, SpnavMotionEvent):
                    motion_event[:3] = event.translation
                    motion_event[3:6] = event.rotation
                    motion_event[6] = event.period
                elif isinstance(event, SpnavButtonEvent):
                    button_state[event.bnum] = event.press
                else:
                    # finish integrating this round of events
                    # before sending over
                    self.ring_buffer.put({
                        'motion_event': motion_event,
                        'button_state': button_state,
                        'receive_timestamp': receive_timestamp
                    })
                    time.sleep(1/self.frequency)
        finally:
            spnav_close()
