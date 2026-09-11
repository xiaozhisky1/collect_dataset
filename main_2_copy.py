import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
from datetime import datetime
import jkrc
import time
import argparse
from spacemouse import Spacemouse
import shutil

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
        robot.set_digital_output(0, 0, 0)
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
    return True

def initialize_camera(serial_number, width=640, height=480, fps=30, manual_exposure=None):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipeline.start(config)
    try:
        color_sensor = profile.get_device().first_color_sensor()
        if manual_exposure is not None:
            color_sensor.set_option(rs.option.enable_auto_exposure, 0)
            color_sensor.set_option(rs.option.exposure, float(manual_exposure))
            print(f"[Camera {serial_number}] 手动曝光已设置为 {manual_exposure}")
        else:
            # 显式恢复自动曝光，避免沿用设备上一次手动曝光值
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            print(f"[Camera {serial_number}] 已启用自动曝光")
    except Exception as e:
        print(f"[Camera {serial_number}] 设置曝光模式失败：{e}")
    return pipeline, profile

def save_camera_parameters(profile, serial_number, output_folder):
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_stream.get_intrinsics()

    params = {
        "serial_number": serial_number,
        "intrinsics": {
            "width": intrinsics.width,
            "height": intrinsics.height,
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "ppx": intrinsics.ppx,
            "ppy": intrinsics.ppy,
            "model": str(intrinsics.model),
            "distortion_coefficients": list(intrinsics.coeffs) if intrinsics.coeffs else []
        },
        "extrinsics": {
            "translation": [0.0, 0.0, 0.0],
            "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        }
    }
    json_path = os.path.join(output_folder, "parameters", f"camera_{serial_number}_params.json")
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, 'w') as f:
        json.dump(params, f, indent=4)

def initialize_robot(ip="10.5.5.100"):
    robot = jkrc.RC(ip)
    ret = robot.login()
    robot.power_on()
    robot.enable_robot()
    robot.servo_move_enable(True)
    if ret[0] != 0:
        raise Exception(f"Robot login failed, error code: {ret[0]}")
    return robot

def get_next_episode_number(output_folder):
    if not os.path.exists(output_folder):
        return 1
    existing_episodes = [d for d in os.listdir(output_folder) if d.startswith("episode_")]
    if not existing_episodes:
        return 1
    episode_numbers = [int(d.split("_")[1]) for d in existing_episodes]
    return max(episode_numbers) + 1
def clear_episode_folder(dataset_root, episode_num):
    """
    删除当前 episode 文件夹（含 images 与 robot_data），用于重新采集。
    """
    episode_folder = os.path.join(dataset_root, f"episode_{episode_num}")
    try:
        if os.path.exists(episode_folder):
            shutil.rmtree(episode_folder, ignore_errors=True)
            # 若你希望立即重建空文件夹，也可以取消下面两行注释
            # os.makedirs(os.path.join(episode_folder, "images"), exist_ok=True)
            # os.makedirs(os.path.join(episode_folder, "robot_data"), exist_ok=True)
            print(f"[Redo] 已清空 {episode_folder}")
        else:
            print(f"[Redo] {episode_folder} 不存在，无需清理")
    except Exception as e:
        print(f"[Redo][WARN] 清理 {episode_folder} 失败：{e}")
def main(output_folder="data", control_hz=20, save_hz=10, continue_getdata=False,
         manual_exposure=None, manual_exposure_1=None, manual_exposure_2=None):
    program_start_time = time.time()

    # 目录初始化
    if not continue_getdata:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_folder = os.path.join(output_folder, timestamp)
    else:
        base_folder = output_folder
    param_folder = os.path.join(base_folder, "parameters")
    os.makedirs(param_folder, exist_ok=True)

    # 相机
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) < 2:
        print("Error: Two RealSense cameras are required.")
        return
    serial_1 = devices[0].get_info(rs.camera_info.serial_number)
    serial_2 = devices[1].get_info(rs.camera_info.serial_number)
    print(f"Camera 1 Serial: {serial_1}")
    print(f"Camera 2 Serial: {serial_2}")

    cam1_exposure = manual_exposure_1 if manual_exposure_1 is not None else manual_exposure
    cam2_exposure = manual_exposure_2 if manual_exposure_2 is not None else manual_exposure

    pipeline_1, profile_1 = initialize_camera(serial_1, manual_exposure=cam1_exposure)
    pipeline_2, profile_2 = initialize_camera(serial_2, manual_exposure=cam2_exposure)

    save_camera_parameters(profile_1, serial_1, base_folder)
    save_camera_parameters(profile_2, serial_2, base_folder)

    # 机器人
    robot = initialize_robot()

    # 在开始采集第 1 条轨迹之前回到初始位姿
    move_robot_to_home(robot)

    # SpaceMouse
    spacemouse = Spacemouse(deadzone=0.3)
    spacemouse.start()

    colorizer = rs.colorizer()

    # 频率控制
    target_interval = 1.0 / float(control_hz)
    assert control_hz >= save_hz and control_hz % save_hz == 0, \
        f"control_hz({control_hz}) 必须是 save_hz({save_hz}) 的整数倍"
    save_interval = int(control_hz // save_hz)  # 例如：20/5 = 4

    # 数据集目录
    dataset_root = os.path.join(base_folder, "dataset")
    os.makedirs(dataset_root, exist_ok=True)
    max_episode_num = get_next_episode_number(dataset_root)

    # 夹爪状态机（统一约定：IO 0=张开，1=闭合）
    gripper_open = True  # True=张开, False=闭合
    last_btn0 = False
    last_btn1 = False
    rotation_enabled = False

    try:
        # 初始：张开 => IO 输出 0
        robot.set_digital_output(0, 0, 0 if gripper_open else 1)
    except Exception as e:
        print(f"[WARN] 初始化抓夹IO失败：{e}")

    # 保存去重需要的“上一已保存状态”
    last_saved_tcp = None          # list/tuple of 6
    last_saved_gripper = None      # 0=open, 1=close
    first_saved_in_episode = False # 本 episode 是否已保存过第一帧

    # 统一窗口创建
    cv2.namedWindow('Dual RealSense D435i Feeds', cv2.WINDOW_NORMAL)

    # 计数器
    frame_count = 0
    save_count = 0

    try:
        while True:
            start_time = time.time()

            # === 获取双相机帧 ===
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            depth_frame_1 = frames_1.get_depth_frame()

            frames_2 = pipeline_2.wait_for_frames()
            color_frame_2 = frames_2.get_color_frame()
            depth_frame_2 = frames_2.get_depth_frame()

            if not (color_frame_1 and depth_frame_1 and color_frame_2 and depth_frame_2):
                elapsed_time = time.time() - start_time
                sleep_time = max(0, target_interval - elapsed_time)
                time.sleep(sleep_time)
                continue

            # === SpaceMouse 输入 ===
            motion = spacemouse.get_motion_state_transformed()
            translation = motion[:3] * 8.0

            # —— 旋转模式：按键切换（SpaceMouse Btn1 或键盘 R）
            btn1 = spacemouse.is_button_pressed(1)
            # 上升沿触发切换
            if btn1 and not last_btn1:
                rotation_enabled = not rotation_enabled
                print(f"[Rotation] {'ENABLED' if rotation_enabled else 'LOCKED'} (toggled by Btn1)")
            last_btn1 = btn1

            # 根据开关决定是否应用旋转
            if rotation_enabled:
                # 取出原始旋转量
                rx, ry, rz = motion[3:]

                # === 对 (rx, ry) 做 -45° 坐标旋转，解耦控制 ===
                angle = np.deg2rad(-45)
                rot_matrix = np.array([
                    [np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]
                ])
                rx_new, ry_new = rot_matrix @ np.array([rx, ry])

                rotation = np.array([rx_new, ry_new, rz]) * 0.04
            else:
                rotation = np.zeros(3, dtype=np.float32)

            # 读取当前 TCP 位姿
            result = robot.get_tcp_position()
            if result[0] != 0:
                print("Failed to get TCP position")
                elapsed_time = time.time() - start_time
                sleep_time = max(0, target_interval - elapsed_time)
                time.sleep(sleep_time)
                continue

            tcp_pose = result[1]  # 长度6
            new_pose = None

            # === 仅当增量非零才下发运动指令 ===
            if not (np.all(translation == 0) and np.all(rotation == 0)):
                new_pose = list(tcp_pose)
                for i in range(3):
                    new_pose[i] += float(translation[i])
                    new_pose[i + 3] += float(rotation[i])
                robot.servo_p(new_pose, 0)

            # else: 增量为0，不调用 servo_p

            # Btn0 → 切换抓夹（IO：开=0，关=1）
            btn0 = spacemouse.is_button_pressed(0)
            if btn0 and not last_btn0:
                gripper_open = not gripper_open
                try:
                    robot.set_digital_output(0, 0, 0 if gripper_open else 1)
                    print("[Gripper]", "打开" if gripper_open else "关闭")
                except Exception as e:
                    print(f"[ERROR] 抓夹IO切换失败：{e}")
            last_btn0 = btn0

            # 机器人状态（用于保存）
            ret = robot.get_joint_position()
            if ret[0] != 0:
                print(f"Failed to get joint position, error code: {ret[0]}")
                elapsed_time = time.time() - start_time
                sleep_time = max(0, target_interval - elapsed_time)
                time.sleep(sleep_time)
                continue
            joint_angles = ret[1]
            tcp_pose = robot.get_tcp_position()
            if new_pose is not None:
                print("save_count:", save_count)
                print(tcp_pose[1])
                print(new_pose)
            tcp_data = tcp_pose[1] if tcp_pose[0] == 0 else None

            # 转 numpy & 可视化深度
            color_image_1 = np.asanyarray(color_frame_1.get_data())
            depth_image_1 = np.asanyarray(colorizer.colorize(depth_frame_1).get_data())
            color_image_2 = np.asanyarray(color_frame_2.get_data())
            depth_image_2 = np.asanyarray(colorizer.colorize(depth_frame_2).get_data())

            # 显示（原分辨率的拼图，仅用于观察）
            top_row = np.hstack((color_image_1, depth_image_1))
            bottom_row = np.hstack((color_image_2, depth_image_2))
            combined_image = np.vstack((top_row, bottom_row))
            cv2.putText(combined_image, f"Camera 1 Color (Serial: {serial_1})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined_image, f"Camera 1 Depth", (650, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined_image, f"Camera 2 Color (Serial: {serial_2})", (10, 510),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined_image, f"Camera 2 Depth", (650, 510),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 叠加状态提示：抓夹与旋转使能
            cv2.putText(combined_image, f"Gripper: {'OPEN(0)' if gripper_open else 'CLOSED(1)'} (Btn0 toggle)",
                        (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_image, f"Rotation: {'ENABLED' if rotation_enabled else 'LOCKED'} (toggle Btn1/R)",
                        (10, 445), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow('Dual RealSense D435i Feeds', combined_image)

            # 计数（控制频率计数）
            frame_count += 1

            # === 保存逻辑（save_hz）：每 save_interval 帧尝试保存一次 ===
            if (frame_count % save_interval) == 0:
                # 预处理与缩放（保存为 256x256）
                RGB_SIZE = (256, 256)
                DEPTH_SIZE = (256, 256)
                c1_rgb_256 = cv2.resize(color_image_1, RGB_SIZE, interpolation=cv2.INTER_AREA)
                d1_vis_256 = cv2.resize(depth_image_1, DEPTH_SIZE, interpolation=cv2.INTER_NEAREST)
                c2_rgb_256 = cv2.resize(color_image_2, RGB_SIZE, interpolation=cv2.INTER_AREA)
                d2_vis_256 = cv2.resize(depth_image_2, DEPTH_SIZE, interpolation=cv2.INTER_NEAREST)

                current_gripper_int = 0 if gripper_open else 1

                # === 修改后的保存条件 ===
                if not first_saved_in_episode:
                    # episode 第一次保存 → 只有 motion 全零时才跳过
                    if np.all(translation == 0) and np.all(rotation == 0):
                        need_save = False
                    else:
                        need_save = True
                else:
                    # 其他情况 → 一律保存
                    need_save = True

                if need_save:
                    save_count += 1
                    episode_folder = os.path.join(dataset_root, f"episode_{max_episode_num}")
                    dataset_folder = os.path.join(episode_folder, "images")
                    robot_data_folder = os.path.join(episode_folder, "robot_data")
                    os.makedirs(dataset_folder, exist_ok=True)
                    os.makedirs(robot_data_folder, exist_ok=True)

                    timestep_folder = os.path.join(dataset_folder, str(save_count))
                    os.makedirs(timestep_folder, exist_ok=True)

                    cv2.imwrite(os.path.join(timestep_folder, "camera1_rgb.png"), c1_rgb_256)
                    cv2.imwrite(os.path.join(timestep_folder, "camera1_depth.png"), d1_vis_256)
                    cv2.imwrite(os.path.join(timestep_folder, "camera2_rgb.png"), c2_rgb_256)
                    cv2.imwrite(os.path.join(timestep_folder, "camera2_depth.png"), d2_vis_256)

                    robot_subfolder = os.path.join(robot_data_folder, str(save_count))
                    os.makedirs(robot_subfolder, exist_ok=True)
                    robot_data = {
                        "timestamp": datetime.now().isoformat(),
                        "joint_angles": joint_angles,
                        "tcp_pose": tcp_data,
                        # 统一保存语义：0=张开，1=闭合
                        "gripper": current_gripper_int,
                    }
                    with open(os.path.join(robot_subfolder, "robot_data.json"), 'w') as f:
                        json.dump(robot_data, f, indent=4)

                    # 更新“上一已保存状态”
                    last_saved_tcp = tcp_data[:] if tcp_data is not None else None
                    last_saved_gripper = current_gripper_int
                    first_saved_in_episode = True
                # else: 跳过保存

            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("收到退出(q)，停止采集。")
                break
            if key == ord('c'):
                # 新开一条演示/轨迹（递增 episode 号）
                max_episode_num = get_next_episode_number(dataset_root)
                frame_count = 0
                save_count = 0
                # 重置去重状态
                last_saved_tcp = None
                last_saved_gripper = None
                first_saved_in_episode = False
                print(f"正在采集第 {max_episode_num} 条轨迹（已重置计数）")

                # 开始新一条采集前，回到初始位姿
                gripper_open = move_robot_to_home(robot)

            if key == ord('r'):
                # 重新收集当前这条轨迹（不递增 episode 号）：清空当前 episode 已采集的数据并复位
                try:
                    clear_episode_folder(dataset_root, max_episode_num)
                except Exception as e:
                    print(f"[Redo][ERROR] 清理当前 episode 失败：{e}")

                # 计数与去重状态全部复位
                frame_count = 0
                save_count = 0
                last_saved_tcp = None
                last_saved_gripper = None
                first_saved_in_episode = False

                # 回到初始位姿，重新开始本 episode
                try:
                    gripper_open = move_robot_to_home(robot)
                except Exception as e:
                    print(f"[Redo][WARN] 回初始位姿失败：{e}")

                print(f"[Redo] 已重置并准备重新采集第 {max_episode_num} 条轨迹")


            # 控制频率：睡眠补偿
            elapsed_time = time.time() - start_time
            sleep_time = max(0, target_interval - elapsed_time)
            time.sleep(sleep_time)

    finally:
        program_end_time = time.time()
        total_runtime = program_end_time - program_start_time
        actual_control_hz = (frame_count / total_runtime) if total_runtime > 0 else 0.0
        actual_save_hz = (save_count / total_runtime) if total_runtime > 0 else 0.0

        print(f"Total runtime: {total_runtime:.2f} seconds")
        print(f"Control ticks: {frame_count}  (~{actual_control_hz:.2f} Hz)")
        print(f"Saved samples: {save_count}  (~{actual_save_hz:.2f} Hz)")

        metrics = {
            "total_runtime_seconds": total_runtime,
            "control_ticks": frame_count,
            "saved_samples": save_count,
            "actual_control_hz": actual_control_hz,
            "actual_save_hz": actual_save_hz,
            "target_control_hz": control_hz,
            "target_save_hz": save_hz
        }
        metrics_path = os.path.join(param_folder, "capture_metrics.json")
        os.makedirs(os.path.dirname(metrics_path), exist_ok=True)
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=4)

        try:
            spacemouse.stop()
        except Exception:
            pass
        try:
            pipeline_1.stop()
            pipeline_2.stop()
        except Exception:
            pass
        try:
            robot.logout()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_folder", help="文件存储地址", default="data")

    # 兼容旧参数：--target_fps 等价于 --control_hz
    parser.add_argument("--target_fps", type=int, help="【兼容参数】控制频率（等价于 --control_hz）", default=None)

    parser.add_argument("--control_hz", type=int, help="控制频率（Hz）", default=20)
    parser.add_argument("--save_hz", type=int, help="保存频率（Hz）", default=10)
    parser.add_argument("--manual_exposure", type=float, default=None,
                        help="两路相机统一手动曝光值（兼容参数）")
    parser.add_argument("--manual_exposure_1", type=float, default=None,
                        help="相机1手动曝光值（优先级高于 --manual_exposure）120/400-450")
    parser.add_argument("--manual_exposure_2", type=float, default=None,
                        help="相机2手动曝光值（优先级高于 --manual_exposure）80-100/250")

    # 为了更稳妥地接收布尔类型，这里将字符串 true/false 映射为布尔
    def str2bool(v):
        if isinstance(v, bool):
            return v
        return v.lower() in ('1', 'true', 't', 'yes', 'y')
    parser.add_argument("--continue_getdata", type=str2bool, nargs='?', const=True, default=False,
                        help="是否继续获取数据（True/False）")

    args = parser.parse_args()

    # 若提供了旧参数，则覆盖 control_hz
    if args.target_fps is not None:
        args.control_hz = args.target_fps

    main(output_folder=args.output_folder,
         control_hz=args.control_hz,
         save_hz=args.save_hz,
         continue_getdata=args.continue_getdata,
         manual_exposure=args.manual_exposure,
         manual_exposure_1=args.manual_exposure_1,
         manual_exposure_2=args.manual_exposure_2)
