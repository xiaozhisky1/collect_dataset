from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from threading import Thread, Event
from collections import defaultdict
import numpy as np
import time


class Spacemouse(Thread):
    def __init__(self, max_value=500, deadzone=(0,0,0,0,0,0), dtype=np.float32):
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
        """ # max_value：控制输入的最大值。值为300表示有线设备，500表示无线设备。 deadzone：用于消除小幅度抖动的“死区”，可定义为单个值或长度为6的数组，每个轴小于该值的输入将被设为零。 dtype：指定数值类型，默认为np.float32。
        if np.issubdtype(type(deadzone), np.number):# 如果deadzone是单一数值，则扩展为6个轴的数组；否则将其转换为数组格式。
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        super().__init__()
        self.stop_event = Event() # stop_event用于线程控制，决定是否停止循环。
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)# motion_event用于保存SpaceMouse的平移和旋转信息。
        self.button_state = defaultdict(lambda: False)# button_state使用defaultdict初始化按键状态。
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=dtype)

    def get_motion_state(self):#将motion_event中的平移和旋转数据按max_value归一化到[-1, 1]范围。#应用deadzone消除小幅度抖动，将接近零的值置为零。
        me = self.motion_event
        state = np.array(me.translation + me.rotation, 
            dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state
    
    def get_motion_state_transformed(self):# 使用tx_zup_spnav矩阵将坐标转换为右手坐标系，以方便进行后续操作。
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
        tf_state[0]=-tf_state[0]
        tf_state[1]=-tf_state[1]
        return tf_state

    def is_button_pressed(self, button_id):# 检查特定按键（button_id）是否被按下，返回布尔值。
        return self.button_state[button_id]

    def stop(self):# 设置stop_event为True以停止线程，并调用join等待线程结束。
        self.stop_event.set()
        self.join()
# 定义上下文管理器__enter__和__exit__，使得Spacemouse可以用with语句进行资源管理，自动启动和停止。
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():# 主事件循环，调用spnav_poll_event获取SpaceMouse的事件，直到stop_event被设置为True。
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent): # 如果事件是SpnavMotionEvent类型，更新平移和旋转信息；
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent): # 如果是SpnavButtonEvent类型，更新按键状态。
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1/200) # 循环等待1/200秒以减轻CPU负担。
        finally:# 在finally块中调用spnav_close确保无论如何都会正确关闭连接。
            spnav_close()


def test():
    with Spacemouse(deadzone=0.3) as sm: # 测试函数在上下文管理器中启动Spacemouse实例，设置deadzone为0.3。
        for i in range(2000): # 每隔1/100秒获取并打印当前的平移/旋转状态（经过坐标转换）和按键状态，持续约20秒。
            # print(sm.get_motion_state())
            print(sm.get_motion_state_transformed())
            print(sm.is_button_pressed(0))
            print(sm.is_button_pressed(1))
            time.sleep(1/100)

if __name__ == '__main__':
    test()
