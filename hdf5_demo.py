import h5py
import numpy as np
import time
import jkrc

# === 初始位姿与回零函数 ===
HOME_JOINTS = [
    0.1356431053774459,
    1.3770658289898887,
    -0.9458763740625533,
    1.1441266389982259,
    1.574945146395447,
    0.8625550837790502,
]
def move_robot_to_home(robot, speed=0.2):
    """
    在发起数据采集前调用，使机械臂回到既定关节初始位姿。
    - 为避免与 servo_p 冲突：回零前临时关闭伺服模式，回零后再打开。
    - speed: 关节移动速度（请按控制器标准设定）
    """
    try:
        # 关闭伺服以避免与 joint_move 冲突
        try:
            robot.servo_move_enable(False)
        except Exception:
            pass

        # coord=0(基坐标)，is_block=True(阻塞到位)，speed 为关节空间速度
        ret = robot.joint_move(HOME_JOINTS, 0, True, speed)
        # 兼容 jkrc 返回 (errcode, ...) 的形式
        if isinstance(ret, (list, tuple)) and len(ret) > 0 and ret[0] != 0:
            raise Exception(f"joint_move 返回错误码: {ret[0]}")
    finally:
        # 恢复伺服用于后续 servo_p 微动
        try:
            robot.servo_move_enable(True)
            time.sleep(5)
        except Exception:
            pass
def initialize_robot(ip="10.5.5.100"):
    robot = jkrc.RC(ip)
    ret = robot.login()
    robot.power_on()
    robot.enable_robot()
    robot.servo_move_enable(True)
    if ret[0] != 0:
        raise Exception(f"Robot login failed, error code: {ret[0]}")
    return robot
# 机器人
robot = initialize_robot()

last_grip = None  # 模型输出的 0/1 标志；0=打开，1=闭合

with h5py.File(f"data/real_stack_block_5/real_stack_block_5.hdf5") as file:
    # count total steps
    demos = file['data']  # 打开 HDF5 文件并读取数据
    for i in range(50):
        move_robot_to_home(robot)
        abort_variation = False
        demo = demos[f'demo_{i}']
        actions = demo['action']
        print(f"当前Demo{i}的动作步数为：" + str(actions.shape[0]))
        # for action in actions:
        #     # action_world = compute_pose_in_world(arm_pose, action[:-1])
        #     # action_world = np.concatenate([action_world, np.array([action[-1]])])
        #     print(action)
        #     try:
        #     except Exception as e:
        #         print(f"运动出错")
        #         continue
        # 逐步下发（最多 24 步以稳定周期）
        for t_idx, a in enumerate(actions):
            # if t_idx >= 24:
            #     continue
            if a.shape[0] < 7:
                print(f"[ERROR] 第 {t_idx} 步动作维度为 {a.shape[0]}，需要 7；跳过该步")
                continue

            # 前 6 维：绝对末端位姿（欧拉角）——不做单位转换，直接下发
            target_pose = [float(v) for v in a[:6]]

            # 第 7 维：夹爪（1=闭合，0=打开）
            grip_flag = int(round(float(a[6])))  # -> 0 或 1

            # 先控制夹爪（只在状态变化时下发 IO）
            if last_grip is None or grip_flag != last_grip:
                try:
                    robot.set_digital_output(0, 0, 1 if grip_flag == 1 else 0)
                except Exception as e:
                    print(f"[WARN] gripper IO failed at t={t_idx}: {e}")
                last_grip = grip_flag
                # 同步显示与 qpos 语义：open=True 表示张开
                gripper_open = (grip_flag == 0)

            # 下发该时间步位姿
            try:
                robot.servo_p(target_pose, 0)
            except Exception as e:
                print(f"[WARN] servo_p failed at t={t_idx}: {e}")
                break
            time.sleep(0.1)

# data = [
#     [0.65195715, -0.08369011, 0.5814738, 0.61454225, 0.06168066, -0.78498507, -0.04828935, 1.0],
#     [0.11426067, -0.15031841, 0.65105075, 0.0602768, 0.70499605, -0.06015408, -0.70408016, 1.0],
#     [0.05141068, -0.15029666, 0.6509042, 0.05914127, 0.70523185, -0.05875754, -0.7040582, 1.0],
#     [0.05104983, -0.15033394, 0.6511717, 0.05904227, 0.7049376, -0.05885811, -0.70435286, 0.0],
#     [0.30919266, -0.15131807, 0.6464823, 0.02027337, 0.69994813, -0.02196239, -0.713568, 0.0]
# ]
#
# # 转换为 numpy 数组
# actions = np.array(data)
# env.reset()
# i=0
# time.sleep(5)
# for action in actions:
#     print(i)
#     i += 1
#     action_world = compute_pose_in_world(arm_pose, action[:-1])
#     action_world = np.concatenate([action_world, np.array([action[-1]])])
#     try:
#         ts_obs, reward, terminate = env.step(action_world)
#     except Exception as e:
#         continue
