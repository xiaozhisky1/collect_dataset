
import asyncio
import websockets
import numpy as np
import time
import msgpack_numpy


async def run_client():
    uri = "ws://localhost:8000"
    task_name = "pick_up_cup"
    robot_name = "panda"
    max_episodes = 100
    max_steps = 200
    batch_size = 1
    max_size = 2**26  # 64MB 以防止消息截断
    episode_returns = []
    highest_rewards = []
    packer = msgpack_numpy.Packer()

    robot = jkrc.RC("10.5.5.100")  # 返回一个机器人对象
    ret = robot.login()  # 登录
    tcp_pose = robot.get_tcp_position()[1]


    try:
        async with websockets.connect(uri, compression=None, max_size=max_size, ping_interval=None, ping_timeout=None) as ws:
            for ep in range(max_episodes):



                arm_class, gripper_class, _ = SUPPORTED_ROBOTS[
                    robot_setup]
                arm, gripper = arm_class(), gripper_class()
                arm_pose = arm.get_pose()
                # 验证 ts_obs 字段
                required_fields = ["wrist_rgb", "front_rgb", "gripper_pose"]
                for field in required_fields:
                    if not hasattr(ts_obs, field):
                        print(f"[ERROR] ts_obs 缺少字段: {field}")
                        break
                # 发送初始 observation
                try:
                    state_np = np.append(ts_obs.gripper_pose, ts_obs.gripper_open).astype(np.float32)[np.newaxis]
                    # print(f"[DEBUG] wrist_rgb: shape={ts_obs.wrist_rgb.shape}, dtype={ts_obs.wrist_rgb.dtype}")

                    obs = {
                        "task": task_name,
                        "wrist_image": ts_obs.wrist_rgb[np.newaxis],
                        "front_image": ts_obs.front_rgb[np.newaxis],
                        "state": state_np,
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

                while  not done and t < max_steps:
                    try:
                        action_bin = await ws.recv()
                        # print(f"[DEBUG] 收到动作消息: 长度={len(action_bin)} 字节")
                        action_msg = msgpack_numpy.unpackb(action_bin, raw=False)
                        
                        # 强制将其中的 numpy 数组变为可写副本
                        if isinstance(action_msg, dict) and "action" in action_msg:
                            action_msg["action"] = np.array(action_msg["action"]).copy()

                        action_msg = action_msg.copy()

                        # print(f"[DEBUG] 解包后的动作消息: {action_msg}")
                        if not isinstance(action_msg, dict) or "action" not in action_msg:
                            print("[ERROR] 动作消息格式错误: 期望包含 'action' 键的字典")
                            break
                        action = np.asarray(action_msg["action"])
                        if action.shape != (batch_size, 8) or action.dtype != np.float32:
                            print(f"[ERROR] 动作无效: 形状 {action.shape}, 类型 {action.dtype}")
                            break
                        action = action[0]
                    except websockets.exceptions.ConnectionClosed as e:
                        print(f"[ERROR] WebSocket 在接收动作时连接关闭: {e}")
                        # break
                    except Exception as e:
                        print(f"[ERROR] 接收或解析动作失败: {e}")
                        # break
                    action[7] = 1.0 if action[7] > 0.95 else 0.0
                    print(action)
                    # 四元数归一化
                    quaternion_raw = action[3:7]
                    norm = np.linalg.norm(quaternion_raw)
                    if norm > 0:
                        quaternion_normalized = quaternion_raw / norm
                    else:
                        quaternion_normalized = np.array([1.0, 0.0, 0.0, 0.0])
                    action[3:7] = quaternion_normalized

                    # 提取 gripper 控制值
                    gripper = action[7]

                    # 相对动作到绝对动作转换（前 7 维: xyz + 四元数）
                    action_world = compute_pose_in_world(arm_pose, action[:7])

                    # 拼接 gripper 值，构成完整的 8 维动作
                    action = np.concatenate([action_world, [gripper]])

                    reward = 0
                    done = False
                    success = 0
                    try:
                        ts_obs, reward, done = env.step(np.concatenate([action[:7], [action[7]]]))
                        # print(f"[DEBUG] 环境步进: 奖励={reward}, 完成={done}")
                        # print(f"[DEBUG] Step {t + 1}: ts_obs.wrist_rgb mean={ts_obs.wrist_rgb.mean()}, gripper_pose={ts_obs.gripper_pose}, reward={reward}, done={done}")

                        if not isinstance(reward, (int, float, np.floating)):
                            print(f"[ERROR] 奖励类型无效: 期望标量，实际 {type(reward)}")

                        if not isinstance(done, (bool, np.bool_)):
                            print(f"[ERROR] 完成状态类型无效: 期望布尔值，实际 {type(done)}")

                    except Exception as e:
                        print(f"[ERROR] 环境步进失败: {e}")
                        # break
                    if t == max_steps - 1:
                        done = True
                    
                    success = reward == 1.0 and np.isclose(action[7], 1.0)
                    feedback = {
                        "observation": {
                            "task": task_name,
                            "wrist_image": ts_obs.wrist_rgb[np.newaxis].copy(),
                            "front_image": ts_obs.front_rgb[np.newaxis].copy(),
                            "state": np.append(ts_obs.gripper_pose, ts_obs.gripper_open).astype(np.float32)[np.newaxis].copy(),
                        },
                        "reward": float(reward),
                        "terminated": bool(done),
                        "success": bool(success),
                    }
                    # 打印 feedback 中的 state
                    # print(f"[DEBUG] feedback['observation']['state']: value={feedback['observation']['state']}, "
                    #     f"shape={feedback['observation']['state'].shape}, dtype={feedback['observation']['state'].dtype}")
                    # print(feedback)
                    # print("-------------------")
                    # print(obs["wrist_image"] == feedback["observation"]["wrist_image"] )
                    # print(f"[DEBUG] feedback['observation']['wrist_image']: type={type(feedback['observation']['wrist_image'])}, shape={feedback['observation']['wrist_image'].shape}")
                    try:
                        transition = (feedback["observation"], feedback["reward"], feedback["terminated"])
                        packed_feedback = packer.pack(transition)
                        await ws.send(packed_feedback)
                        print("[DEBUG] 反馈数据发送成功")
                    except websockets.exceptions.ConnectionClosed as e:
                        print(f"[ERROR] WebSocket 在发送反馈数据时连接关闭: {e}")
                    except Exception as e:
                        print(f"[ERROR] 发送反馈数据失败: {e}")

                    rewards.append(reward)
                    print(f"Step {t + 1} - 奖励: {reward}, 成功: {success}")
                    if success:
                        done = True
                    t += 1
                    print(t)

                episode_return = np.sum(rewards)
                episode_returns.append(episode_return)
                highest_rewards.append(np.max(rewards) if rewards else 0.0)

            success_rate = np.mean(np.array(highest_rewards) == 1.0)
            avg_return = np.mean(episode_returns)
            print(f"\n成功率: {success_rate * 100:.2f}%")
            print(f"平均回报: {avg_return:.2f}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"[ERROR] WebSocket 意外关闭: {e}")
    except Exception as e:
        print(f"[ERROR] 意外错误: {e}")
    finally:
        pass

if __name__ == "__main__":
    asyncio.run(run_client())