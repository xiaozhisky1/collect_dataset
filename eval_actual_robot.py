import pyrealsense2 as rs
import asyncio
import websockets
import numpy as np
import time
import msgpack_numpy
import jkrc


async def run_client():
    uri = "ws://localhost:8880"
    task_name = "Hhhhhhh"
    max_episodes = 100
    max_steps = 200
    batch_size = 1
    max_size = 2 ** 26  # 64MB 以防止消息截断
    packer = msgpack_numpy.Packer()

    robot = jkrc.RC("10.5.5.100")  # 返回一个机器人对象
    ret = robot.login()  # 登录

    pipeline1 = rs.pipeline()
    pipeline2 = rs.pipeline()

    config1 = rs.config()
    config2 = rs.config()

    ctx = rs.context()
    if len(ctx.devices) < 2:
        print("需要至少两个RealSense相机")
        return

    serial1 = ctx.devices[0].get_info(rs.camera_info.serial_number)
    serial2 = ctx.devices[1].get_info(rs.camera_info.serial_number)

    # 启用设备序列号和RGB流
    config1.enable_device(serial1)
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config2.enable_device(serial2)
    config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline1.start(config1)
    pipeline2.start(config2)

    try:
        async with websockets.connect(uri, compression=None, max_size=max_size, ping_interval=None,
                                      ping_timeout=None) as ws:
            for ep in range(max_episodes):
                arm_pose = robot.get_tcp_position()[1]

                frames1 = pipeline1.wait_for_frames()
                frames2 = pipeline2.wait_for_frames()

                color_frame1 = frames1.get_color_frame()
                color_frame2 = frames2.get_color_frame()

                color_image1 = np.asanyarray(color_frame1.get_data())
                color_image2 = np.asanyarray(color_frame2.get_data())

                try:
                    obs = {
                        "task": task_name,
                        "wrist_image": color_image1,
                        "front_image": color_image2,
                        "state": arm_pose,
                    }
                    packed_obs = packer.pack(obs)
                    await ws.send(packed_obs)
                    # print("[DEBUG] 初始观测数据发送成功")
                except websockets.exceptions.ConnectionClosed as e:
                    print(f"[ERROR] WebSocket 在发送初始观测数据时连接关闭: {e}")
                    # break
                except Exception as e:
                    print(f"[ERROR] 发送初始观测数据失败: {e}")
                    # break


                try:
                    action_bin = await ws.recv()
                    # print(f"[DEBUG] 收到动作消息: 长度={len(action_bin)} 字节")
                    action_msg = msgpack_numpy.unpackb(action_bin, raw=False)

                    # 强制将其中的 numpy 数组变为可写副本
                    if isinstance(action_msg, dict) and "action" in action_msg:
                        action_msg["action"] = np.array(action_msg["action"]).copy()

                    action_msg = action_msg.copy()

                    #
                    if not isinstance(action_msg, dict) or "action" not in action_msg:
                        print("[ERROR] 动作消息格式错误: 期望包含 'action' 键的字典")
                        break
                    action = np.asarray(action_msg["action"])
                    if action.shape != (batch_size, 6) or action.dtype != np.float32:
                        print(f"[ERROR] 动作无效: 形状 {action.shape}, 类型 {action.dtype}")
                        break
                    action = action[0]
                except websockets.exceptions.ConnectionClosed as e:
                    print(f"[ERROR] WebSocket 在接收动作时连接关闭: {e}")
                    # break
                except Exception as e:
                    print(f"[ERROR] 接收或解析动作失败: {e}")
                    # break

                print(action)
                # 四元数归一化
                return









    except websockets.exceptions.ConnectionClosed as e:
        print(f"[ERROR] WebSocket 意外关闭: {e}")
    except Exception as e:
        print(f"[ERROR] 意外错误: {e}")
    finally:
        pass


if __name__ == "__main__":
    asyncio.run(run_client())