#
# import os
# import json
# import h5py
# import cv2
# import numpy as np
# from tqdm import tqdm
# import open3d as o3d
# import matplotlib.pyplot as plt
#
# # ===== 配置 =====
# TARGET_SIZE = (320, 320)     # 统一到 256x256
# DEPTH_SCALE_MM = 1000.0      # 原始深度单位 mm -> 除以 1000 变成米
#
# def load_intrinsics(json_path):
#     with open(json_path, "r") as f:
#         params = json.load(f)
#     intr = params.get("intrinsics", None)
#     if intr is None:
#         raise KeyError(f"{json_path} 缺少 'intrinsics' 字段")
#
#     # === 根据 TARGET_SIZE 缩放内参 ===
#     scale_x = TARGET_SIZE[0] / intr["width"]
#     scale_y = TARGET_SIZE[1] / intr["height"]
#
#     return {
#         "width":  TARGET_SIZE[0],
#         "height": TARGET_SIZE[1],
#         "fx":     float(intr["fx"]) * scale_x,
#         "fy":     float(intr["fy"]) * scale_y,
#         "ppx":    float(intr["ppx"]) * scale_x,
#         "ppy":    float(intr["ppy"]) * scale_y,
#     }
#
# def depth_to_pointcloud(depth_mm, intr):
#     """输入: depth_mm (H,W) float32 毫米；输出: (H,W,3) float32 米"""
#     H, W = depth_mm.shape
#     u = np.arange(W, dtype=np.float32)
#     v = np.arange(H, dtype=np.float32)
#     uu, vv = np.meshgrid(u, v)
#
#     z = depth_mm / DEPTH_SCALE_MM  # 转米
#     fx, fy, cx, cy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]
#
#     x = (uu - cx) * z / fx
#     y = (vv - cy) * z / fy
#     return np.stack([x, y, z], axis=-1).astype(np.float32)
#
# def read_depth_uint16(depth_path):
#     depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
#     if depth is None:
#         raise FileNotFoundError(f"读取失败: {depth_path}")
#     if depth.ndim != 2 or depth.dtype != np.uint16:
#         raise ValueError(f"{depth_path} 不是 uint16 单通道原始深度 (shape={depth.shape}, dtype={depth.dtype})")
#     return depth.astype(np.float32)
#
# def resize_depth_mm(depth_mm, size):
#     return cv2.resize(depth_mm, size, interpolation=cv2.INTER_NEAREST)
#
# def read_rgb(path):
#     img = cv2.imread(path, cv2.IMREAD_COLOR)
#     if img is None:
#         raise FileNotFoundError(f"读取失败: {path}")
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     img = cv2.resize(img, TARGET_SIZE, interpolation=cv2.INTER_AREA)
#     return img.astype(np.uint8)
#
# def collect_steps(ep_dir):
#     img_root = os.path.join(ep_dir, "images")
#     rob_root = os.path.join(ep_dir, "robot_data")
#     if not (os.path.isdir(img_root) and os.path.isdir(rob_root)):
#         return []
#     steps = [s for s in os.listdir(img_root) if s.isdigit()]
#     steps.sort(key=lambda x: int(x))
#     return [(os.path.join(img_root, s), os.path.join(rob_root, s)) for s in steps]
#
# def build_h5(dataset_root, camera_param, output_h5, save_rgb=True):
#     intr = load_intrinsics(camera_param)
#     episodes = sorted(
#         [d for d in os.listdir(dataset_root) if d.startswith("episode_")],
#         key=lambda x: int(x.split("_")[1])
#     )
#     if not episodes:
#         raise RuntimeError(f"{dataset_root} 下没有 episode_*")
#
#     os.makedirs(os.path.dirname(output_h5) or ".", exist_ok=True)
#     with h5py.File(output_h5, "w") as f:
#         root = f.create_group("data")
#
#         for di, ep in enumerate(tqdm(episodes, desc="Converting -> HDF5 pointcloud")):
#             ep_dir = os.path.join(dataset_root, ep)
#             pairs = collect_steps(ep_dir)
#             if not pairs:
#                 continue
#
#             pcs, eefs, rgbs = [], [], []
#             for img_dir, rob_dir in pairs:
#                 depth_path = os.path.join(img_dir, "camera1_depth.png")
#                 depth_mm = resize_depth_mm(read_depth_uint16(depth_path), TARGET_SIZE)
#                 pcs.append(depth_to_pointcloud(depth_mm, intr))
#
#                 with open(os.path.join(rob_dir, "robot_data.json"), "r") as jf:
#                     d = json.load(jf)
#                 tcp, grip = d.get("tcp_pose"), d.get("gripper")
#                 eefs.append(np.array(tcp + [grip], dtype=np.float32))
#
#                 if save_rgb:
#                     rgbs.append(read_rgb(os.path.join(img_dir, "camera1_rgb.png")))
#
#             pcs  = np.stack(pcs,  axis=0).astype(np.float32)
#             eefs = np.stack(eefs, axis=0).astype(np.float32)
#             if save_rgb:
#                 rgbs = np.stack(rgbs, axis=0).astype(np.uint8)
#
#             demo = root.create_group(f"demo_{di}")
#             demo.create_dataset("pointcloud", data=pcs,  dtype="float32", chunks=(1, *TARGET_SIZE, 3))
#             demo.create_dataset("eef",        data=eefs, dtype="float32", chunks=(min(64, len(eefs)), eefs.shape[1]))
#             if save_rgb:
#                 demo.create_dataset("rgb",     data=rgbs, dtype="uint8",  chunks=(1, *TARGET_SIZE, 3))
#             demo.attrs["episode_name"] = ep
#             demo.attrs["length"] = int(pcs.shape[0])
#
#     print(f"[OK] 写入完成: {output_h5}")
#
# def visualize_demo_frame(h5_path, demo_idx=0, frame_idx=0):
#     with h5py.File(h5_path, "r") as f:
#         demo_name = f"data/demo_{demo_idx}"
#         if demo_name not in f:
#             print(f"[ERR] {demo_name} 不存在")
#             return
#         xyz = f[demo_name]["pointcloud"][frame_idx]
#         rgb = f[demo_name]["rgb"][frame_idx] if "rgb" in f[demo_name] else None
#
#     H, W, _ = xyz.shape
#     xyz_flat = xyz.reshape(-1, 3)
#     mask = np.isfinite(xyz_flat).all(axis=1) & (xyz_flat[:,2] > 0)
#     xyz_valid = xyz_flat[mask]
#
#     rgb_valid = None
#     if rgb is not None:
#         rgb_flat = rgb.reshape(-1, 3)
#         rgb_valid = rgb_flat[mask].astype(np.float32) / 255.0
#
#     print(f"[Stats] demo_{demo_idx}, frame {frame_idx}")
#     print(f"  total pixels: {H*W}, valid points: {xyz_valid.shape[0]}")
#     print(f"  z range: {xyz_valid[:,2].min():.3f} ~ {xyz_valid[:,2].max():.3f} m")
#
#     # 深度热力图
#     z_map = xyz[:,:,2]
#     plt.imshow(z_map, cmap="viridis")
#     plt.colorbar(label="Depth (m)")
#     plt.title(f"Demo {demo_idx} Frame {frame_idx} Depth Map")
#     plt.show()
#
#     # 彩色点云可视化
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(xyz_valid)
#     if rgb_valid is not None:
#         pcd.colors = o3d.utility.Vector3dVector(rgb_valid)
#
#     print("[INFO] 打开 Open3D 窗口 (ESC 关闭)")
#     o3d.visualization.draw_geometries([pcd])
#
# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--dataset_root", default="/home/rookie/collect/data/fruit_wxy/dataset")
#     parser.add_argument("--camera_param", default="/home/rookie/collect/data/fruit_wxy/parameters/camera_937622072591_params.json")
#     parser.add_argument("--output_h5", default="pointcloud_front.hdf5")
#     parser.add_argument("--no_rgb", action="store_true", help="不把 RGB 一并写入 HDF5")
#     parser.add_argument("--viz_demo", type=int, default=0, help="可视化 demo 索引")
#     parser.add_argument("--viz_frame", type=int, default=50, help="可视化帧索引")
#     args = parser.parse_args()
#
#     build_h5(
#         dataset_root=args.dataset_root,
#         camera_param=args.camera_param,
#         output_h5=args.output_h5,
#         save_rgb=not args.no_rgb
#     )
#
#     visualize_demo_frame(args.output_h5, args.viz_demo, args.viz_frame)
# import os
# import json
# import h5py
# import cv2
# import numpy as np
# from tqdm import tqdm
# import open3d as o3d
# import matplotlib.pyplot as plt
#
# DEPTH_SCALE_MM = 1000.0  # 深度单位：mm → m
#
# def load_intrinsics(json_path):
#     with open(json_path, "r") as f:
#         params = json.load(f)
#     intr = params["intrinsics"]
#     return {
#         "width":  intr["width"],
#         "height": intr["height"],
#         "fx":     float(intr["fx"]),
#         "fy":     float(intr["fy"]),
#         "ppx":    float(intr["ppx"]),
#         "ppy":    float(intr["ppy"]),
#     }
#
# def depth_to_pointcloud(depth_mm, intr):
#     """深度图 (H,W) mm → 点云 (H,W,3) m"""
#     H, W = depth_mm.shape
#     u, v = np.meshgrid(np.arange(W, dtype=np.float32),
#                        np.arange(H, dtype=np.float32))
#     z = depth_mm / DEPTH_SCALE_MM  # 转米
#     x = (u - intr["ppx"]) * z / intr["fx"]
#     y = (v - intr["ppy"]) * z / intr["fy"]
#     return np.stack([x, y, z], axis=-1).astype(np.float32)
#
# def read_depth_uint16(depth_path):
#     depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
#     if depth is None:
#         raise FileNotFoundError(f"读取失败: {depth_path}")
#     if depth.ndim != 2 or depth.dtype != np.uint16:
#         raise ValueError(f"{depth_path} 不是单通道 uint16 原始深度")
#     return depth.astype(np.float32)  # 保持 mm
#
# def read_rgb(path, target_size=None):
#     img = cv2.imread(path, cv2.IMREAD_COLOR)
#     if img is None:
#         raise FileNotFoundError(f"读取失败: {path}")
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     if target_size is not None:
#         img = cv2.resize(img, target_size, interpolation=cv2.INTER_AREA)
#     return img.astype(np.uint8)
#
# def collect_steps(ep_dir):
#     img_root = os.path.join(ep_dir, "images")
#     rob_root = os.path.join(ep_dir, "robot_data")
#     if not (os.path.isdir(img_root) and os.path.isdir(rob_root)):
#         return []
#     steps = [s for s in os.listdir(img_root) if s.isdigit()]
#     steps.sort(key=lambda x: int(x))
#     return [(os.path.join(img_root, s), os.path.join(rob_root, s)) for s in steps]
#
# def build_h5(dataset_root, camera_param, output_h5, save_rgb=True):
#     intr = load_intrinsics(camera_param)
#
#     episodes = sorted(
#         [d for d in os.listdir(dataset_root) if d.startswith("episode_")],
#         key=lambda x: int(x.split("_")[1])
#     )
#     if not episodes:
#         raise RuntimeError(f"{dataset_root} 下没有 episode_*")
#
#     os.makedirs(os.path.dirname(output_h5) or ".", exist_ok=True)
#     with h5py.File(output_h5, "w") as f:
#         root = f.create_group("data")
#
#         for di, ep in enumerate(tqdm(episodes, desc="Converting -> HDF5 pointcloud")):
#             ep_dir = os.path.join(dataset_root, ep)
#             pairs = collect_steps(ep_dir)
#             if not pairs:
#                 continue
#
#             pcs, eefs, rgbs = [], [], []
#
#             for img_dir, rob_dir in pairs:
#                 depth_path = os.path.join(img_dir, "camera1_depth.png")
#                 depth_mm = read_depth_uint16(depth_path)
#                 H, W = depth_mm.shape
#
#                 xyz = depth_to_pointcloud(depth_mm, intr)  # (H,W,3)
#                 pcs.append(xyz)
#
#                 robot_json = os.path.join(rob_dir, "robot_data.json")
#                 with open(robot_json, "r") as jf:
#                     d = json.load(jf)
#                 tcp, grip = d["tcp_pose"], d["gripper"]
#                 eefs.append(np.array(tcp + [grip], dtype=np.float32))
#
#                 if save_rgb:
#                     rgb_path = os.path.join(img_dir, "camera1_rgb.png")
#                     rgbs.append(read_rgb(rgb_path, target_size=(W, H)))
#
#             pcs  = np.stack(pcs, axis=0).astype(np.float32)  # (T,H,W,3)
#             eefs = np.stack(eefs, axis=0).astype(np.float32)  # (T,7)
#             if save_rgb:
#                 rgbs = np.stack(rgbs, axis=0).astype(np.uint8)  # (T,H,W,3)
#
#             H, W = pcs.shape[1:3]
#             demo = root.create_group(f"demo_{di}")
#             demo.create_dataset("pointcloud", data=pcs,  dtype="float32", chunks=(1, H, W, 3))
#             demo.create_dataset("eef",        data=eefs, dtype="float32", chunks=(min(64,len(eefs)), eefs.shape[1]))
#             if save_rgb:
#                 demo.create_dataset("rgb",    data=rgbs, dtype="uint8",  chunks=(1, H, W, 3))
#             demo.attrs["episode_name"] = ep
#             demo.attrs["length"] = int(pcs.shape[0])
#
#     print(f"[OK] 写入完成: {output_h5}")
#
# def visualize_demo_frame(h5_path, demo_idx=0, frame_idx=0):
#     with h5py.File(h5_path, "r") as f:
#         demo_name = f"data/demo_{demo_idx}"
#         if demo_name not in f:
#             print(f"[ERR] {demo_name} 不存在")
#             return
#         xyz = f[demo_name]["pointcloud"][frame_idx]
#         rgb = f[demo_name]["rgb"][frame_idx] if "rgb" in f[demo_name] else None
#
#     H, W, _ = xyz.shape
#     xyz_flat = xyz.reshape(-1, 3)
#     mask = np.isfinite(xyz_flat).all(axis=1) & (xyz_flat[:,2] > 0)
#     xyz_valid = xyz_flat[mask]
#
#     rgb_valid = None
#     if rgb is not None:
#         rgb_flat = rgb.reshape(-1, 3)
#         rgb_valid = rgb_flat[mask].astype(np.float32) / 255.0
#
#     print(f"[Stats] demo_{demo_idx}, frame {frame_idx}")
#     print(f"  total pixels: {H*W}, valid points: {xyz_valid.shape[0]}")
#     print(f"  z range: {xyz_valid[:,2].min():.3f} ~ {xyz_valid[:,2].max():.3f} m")
#
#     z_map = xyz[:,:,2]
#     plt.imshow(z_map, cmap="viridis")
#     plt.colorbar(label="Depth (m)")
#     plt.title(f"Demo {demo_idx} Frame {frame_idx} Depth Map")
#     plt.show()
#
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(xyz_valid)
#     if rgb_valid is not None:
#         pcd.colors = o3d.utility.Vector3dVector(rgb_valid)
#
#     print("[INFO] 打开 Open3D 窗口 (ESC 关闭)")
#     o3d.visualization.draw_geometries([pcd])
#
# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--dataset_root", required=True)
#     parser.add_argument("--camera_param", required=True)
#     parser.add_argument("--output_h5", default="pointcloud_front.hdf5")
#     parser.add_argument("--no_rgb", action="store_true", help="不把 RGB 写入 HDF5")
#     parser.add_argument("--viz_demo", type=int, default=0)
#     parser.add_argument("--viz_frame", type=int, default=0)
#     args = parser.parse_args()
#
#     build_h5(args.dataset_root, args.camera_param, args.output_h5, save_rgb=not args.no_rgb)
#     visualize_demo_frame(args.output_h5, args.viz_demo, args.viz_frame)
# import os
# import json
# import h5py
# import cv2
# import numpy as np
# from tqdm import tqdm
# import open3d as o3d
# import matplotlib.pyplot as plt
#
# # ============ 配置 ============
# DEPTH_SCALE_MM = 1000.0       # 深度单位 mm -> m
# NEAR_CLIP = 0.2               # 裁剪近距离 (米)
# FAR_CLIP = 1.5                # 裁剪远距离 (米)
# NUM_POINTS = 8192             # 最远点采样数量
#
# # ============ 相机内参 ============
# def load_intrinsics(json_path, sample_depth_path=None):
#     """
#     加载内参，如果提供 sample_depth_path，会根据图像实际尺寸对内参缩放
#     """
#     with open(json_path, "r") as f:
#         params = json.load(f)
#     intr = params.get("intrinsics", None)
#     if intr is None:
#         raise KeyError(f"{json_path} 缺少 'intrinsics' 字段")
#
#     fx, fy, cx, cy = float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"])
#     w0, h0 = intr["width"], intr["height"]
#
#     if sample_depth_path is not None:
#         depth = cv2.imread(sample_depth_path, cv2.IMREAD_UNCHANGED)
#         if depth is not None:
#             H, W = depth.shape[:2]
#             if (W, H) != (w0, h0):
#                 sx, sy = W / w0, H / h0
#                 fx, fy = fx * sx, fy * sy
#                 cx, cy = cx * sx, cy * sy
#                 print(f"[INFO] 内参已缩放: 原始({w0}x{h0}) -> 实际({W}x{H}), scale=({sx:.3f},{sy:.3f})")
#
#     return {
#         "fx": fx, "fy": fy, "ppx": cx, "ppy": cy
#     }
#
# # ============ 最远点采样 ============
# def farthest_point_sampling(points, num_points):
#     N, _ = points.shape
#     if N <= num_points:
#         return np.arange(N)
#
#     sampled_idx = np.zeros(num_points, dtype=np.int64)
#     distances = np.ones(N) * 1e10
#
#     farthest = np.random.randint(0, N)
#     for i in range(num_points):
#         sampled_idx[i] = farthest
#         centroid = points[farthest, :]
#         dist = np.sum((points - centroid) ** 2, axis=1)
#         distances = np.minimum(distances, dist)
#         farthest = np.argmax(distances)
#
#     return sampled_idx
#
# # ============ 深度 → 彩色点云 ============
# # def create_colored_point_cloud(depth_mm, rgb, intr, near=NEAR_CLIP, far=FAR_CLIP, num_points=NUM_POINTS):
# #     H, W = depth_mm.shape
# #     u, v = np.meshgrid(np.arange(W), np.arange(H))
# #     z = depth_mm / DEPTH_SCALE_MM
# #     x = (u - intr["ppx"]) * z / intr["fx"]
# #     y = (v - intr["ppy"]) * z / intr["fy"]
# #
# #     xyz = np.stack([x, y, z], axis=-1).reshape(-1, 3)
# #     rgb_flat = rgb.reshape(-1, 3) / 255.0
# #
# #     # Step1: 裁剪无效点
# #     mask = (xyz[:,2] > near) & (xyz[:,2] < far) & np.isfinite(xyz).all(axis=1)
# #     xyz = xyz[mask]
# #     rgb_flat = rgb_flat[mask]
# #
# #     # Step2: 最远点采样
# #     if xyz.shape[0] > num_points:
# #         idx = farthest_point_sampling(xyz, num_points)
# #         xyz = xyz[idx]
# #         rgb_flat = rgb_flat[idx]
# #     elif xyz.shape[0] < num_points:
# #         pad = np.zeros((num_points, 6), dtype=np.float32)
# #         pad[:xyz.shape[0],:3] = xyz
# #         pad[:xyz.shape[0],3:] = rgb_flat
# #         return pad
# #
# #     return np.concatenate([xyz, rgb_flat], axis=-1).astype(np.float32)
#
# def create_colored_point_cloud(
#     depth_mm, rgb, intr,
#     near=NEAR_CLIP, far=FAR_CLIP,
#     num_points=NUM_POINTS,
#     stride=2, voxel_size=0.005,
#     x_range=(-0.36, 0.36),
#     y_range=(-1, 0.1),
#     z_range=(0.55, 2.0),
# ):
#     """
#     深度+RGB -> 彩色点云 (带范围裁剪版)
#     1) stride 下采样深度图
#     2) Open3D 生成点云
#     3) 裁剪 z 范围 + xyz 范围
#     4) voxel 下采样
#     5) 随机采样/补零到固定 num_points
#     """
#
#     # --- Step 1: 图像下采样 ---
#     depth_mm = depth_mm[::stride, ::stride]
#     rgb = rgb[::stride, ::stride]
#
#     H, W = depth_mm.shape
#     depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))
#     rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
#
#     # --- Step 2: 构建 RGBD 图像 ---
#     rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#         rgb_o3d, depth_o3d,
#         depth_scale=DEPTH_SCALE_MM,
#         depth_trunc=far,
#         convert_rgb_to_intensity=False
#     )
#
#     intr_o3d = o3d.camera.PinholeCameraIntrinsic(
#         W, H, intr["fx"]/stride, intr["fy"]/stride, intr["ppx"]/stride, intr["ppy"]/stride
#     )
#
#     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr_o3d)
#
#     # --- Step 3: 裁剪范围 ---
#     points = np.asarray(pcd.points)
#     colors = np.asarray(pcd.colors)
#
#     mask = (
#         (points[:, 2] > near) & (points[:, 2] < far) &
#         (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
#         (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
#         (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
#     )
#     points, colors = points[mask], colors[mask]
#
#     if points.shape[0] == 0:
#         return np.zeros((num_points, 6), dtype=np.float32)
#
#     # --- Step 4: voxel 下采样 ---
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
#
#     points = np.asarray(pcd.points)
#     colors = np.asarray(pcd.colors)
#
#     # --- Step 5: 固定点数 ---
#     N = points.shape[0]
#     if N >= num_points:
#         idx = np.random.choice(N, num_points, replace=False)
#         xyz, rgb = points[idx], colors[idx]
#     else:
#         pad = np.zeros((num_points, 6), dtype=np.float32)
#         pad[:N, :3] = points
#         pad[:N, 3:] = colors
#         return pad
#
#     return np.concatenate([xyz, rgb], axis=-1).astype(np.float32)
#
#
# # ============ 数据处理 ============
# def read_depth_uint16(depth_path):
#     depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
#     if depth is None:
#         raise FileNotFoundError(f"读取失败: {depth_path}")
#     if depth.ndim != 2 or depth.dtype != np.uint16:
#         raise ValueError(f"{depth_path} 不是单通道 uint16 原始深度 (shape={depth.shape}, dtype={depth.dtype})")
#     return depth.astype(np.float32)
#
# def read_rgb(path):
#     img = cv2.imread(path, cv2.IMREAD_COLOR)
#     if img is None:
#         raise FileNotFoundError(f"读取失败: {path}")
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     return img.astype(np.uint8)
#
# def collect_steps(ep_dir):
#     img_root = os.path.join(ep_dir, "images")
#     rob_root = os.path.join(ep_dir, "robot_data")
#     if not (os.path.isdir(img_root) and os.path.isdir(rob_root)):
#         return []
#     steps = [s for s in os.listdir(img_root) if s.isdigit()]
#     steps.sort(key=lambda x: int(x))
#     return [(os.path.join(img_root, s), os.path.join(rob_root, s)) for s in steps]
#
# # ============ HDF5 构建 ============
# def build_h5(dataset_root, camera_param, output_h5):
#     # 取第一帧深度图用于缩放内参
#     first_ep = sorted([d for d in os.listdir(dataset_root) if d.startswith("episode_")])[0]
#     first_depth = os.path.join(dataset_root, first_ep, "images", "1", "camera1_depth.png")
#     intr = load_intrinsics(camera_param, first_depth)
#
#     episodes = sorted(
#         [d for d in os.listdir(dataset_root) if d.startswith("episode_")],
#         key=lambda x: int(x.split("_")[1])
#     )
#     if not episodes:
#         raise RuntimeError(f"{dataset_root} 下没有 episode_*")
#
#     os.makedirs(os.path.dirname(output_h5) or ".", exist_ok=True)
#     with h5py.File(output_h5, "w") as f:
#         root = f.create_group("data")
#
#         for di, ep in enumerate(tqdm(episodes, desc="Converting -> HDF5 pointcloud")):
#             ep_dir = os.path.join(dataset_root, ep)
#             pairs = collect_steps(ep_dir)
#             if not pairs:
#                 continue
#
#             pcs, eefs = [], []
#
#             for img_dir, rob_dir in pairs:
#                 depth_path = os.path.join(img_dir, "camera1_depth.png")
#                 rgb_path = os.path.join(img_dir, "camera1_rgb.png")
#
#                 depth_mm = read_depth_uint16(depth_path)
#                 rgb = read_rgb(rgb_path)
#
#                 pc = create_colored_point_cloud(depth_mm, rgb, intr)
#                 pcs.append(pc)
#
#                 with open(os.path.join(rob_dir, "robot_data.json"), "r") as jf:
#                     d = json.load(jf)
#                 tcp, grip = d["tcp_pose"], d["gripper"]
#                 eefs.append(np.array(tcp + [grip], dtype=np.float32))
#
#             pcs = np.stack(pcs, axis=0).astype(np.float32)
#             eefs = np.stack(eefs, axis=0).astype(np.float32)
#
#             demo = root.create_group(f"demo_{di}")
#             demo.create_dataset("pointcloud", data=pcs, dtype="float32")
#             demo.create_dataset("eef", data=eefs, dtype="float32")
#             demo.attrs["episode_name"] = ep
#             demo.attrs["length"] = int(pcs.shape[0])
#
#     print(f"[OK] 写入完成: {output_h5}")
#
# # ============ 可视化 ============
# def visualize_demo_frame(h5_path, demo_idx=0, frame_idx=0):
#     with h5py.File(h5_path, "r") as f:
#         demo_name = f"data/demo_{demo_idx}"
#         if demo_name not in f:
#             print(f"[ERR] {demo_name} 不存在")
#             return
#         pc = f[demo_name]["pointcloud"][frame_idx]  # (N,6)
#
#     xyz, rgb = pc[:,:3], pc[:,3:]
#
#     print(f"[Stats] demo_{demo_idx}, frame {frame_idx}")
#     print(f"  有效点数: {xyz.shape[0]}")
#     print(f"  z range: {xyz[:,2].min():.3f} ~ {xyz[:,2].max():.3f} m")
#
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(xyz)
#     pcd.colors = o3d.utility.Vector3dVector(rgb)
#
#     print("[INFO] 打开 Open3D 窗口 (ESC 关闭)")
#     o3d.visualization.draw_geometries([pcd])
#
# # ============ 主函数 ============
# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--dataset_root", required=True)
#     parser.add_argument("--camera_param", required=True)
#     parser.add_argument("--output_h5", default="pointcloud_front.hdf5")
#     parser.add_argument("--viz_demo", type=int, default=None)
#     parser.add_argument("--viz_frame", type=int, default=0)
#     args = parser.parse_args()
#
#     build_h5(args.dataset_root, args.camera_param, args.output_h5)
#
#     if args.viz_demo is not None:
#         visualize_demo_frame(args.output_h5, args.viz_demo, args.viz_frame)
#
import os
import json
import h5py
import cv2
import numpy as np
from tqdm import tqdm
import open3d as o3d

# ============ 配置 ============
DEPTH_SCALE_MM = 1000.0   # 深度单位 mm -> m
NEAR_CLIP = 0.2           # 裁剪近距离 (米)
FAR_CLIP = 1.5            # 裁剪远距离 (米)
NUM_POINTS =4096         # 每帧点云保留点数

# ============ 相机内参 ============
def load_intrinsics(json_path, sample_depth_path=None):
    with open(json_path, "r") as f:
        params = json.load(f)
    intr = params["intrinsics"]
    fx, fy, cx, cy = float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"])
    w0, h0 = intr["width"], intr["height"]

    if sample_depth_path is not None:
        depth = cv2.imread(sample_depth_path, cv2.IMREAD_UNCHANGED)
        if depth is not None:
            H, W = depth.shape[:2]
            if (W, H) != (w0, h0):
                sx, sy = W / w0, H / h0
                fx, fy = fx * sx, fy * sy
                cx, cy = cx * sx, cy * sy
                print(f"[INFO] 内参已缩放: ({w0}x{h0}) -> ({W}x{H}), scale=({sx:.3f},{sy:.3f})")

    return {"fx": fx, "fy": fy, "ppx": cx, "ppy": cy}

# ============ 深度 → 点云 ============
def create_colored_point_cloud(
    depth_mm, rgb, intr,
    near=NEAR_CLIP, far=FAR_CLIP,
    num_points=NUM_POINTS,
    stride=2, voxel_size=0.005,
    x_range=(-0.36, 0.36),
    y_range=(-1.0, 0.1),
    z_range=(0.55, 2.0),
):
    """深度+RGB -> 裁剪范围的彩色点云"""
    # --- Step1: 下采样 ---
    depth_mm = depth_mm[::stride, ::stride]
    rgb = rgb[::stride, ::stride]
    H, W = depth_mm.shape

    depth_o3d = o3d.geometry.Image(depth_mm.astype(np.uint16))
    rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d,
        depth_scale=DEPTH_SCALE_MM,
        depth_trunc=far,
        convert_rgb_to_intensity=False
    )
    intr_o3d = o3d.camera.PinholeCameraIntrinsic(
        W, H, intr["fx"]/stride, intr["fy"]/stride, intr["ppx"]/stride, intr["ppy"]/stride
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr_o3d)

    # --- Step2: 裁剪范围 ---
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    mask = (
        (points[:, 2] > near) & (points[:, 2] < far) &
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )
    points, colors = points[mask], colors[mask]

    if points.shape[0] == 0:
        return np.zeros((num_points, 6), dtype=np.float32)

    # --- Step3: voxel 下采样 ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # --- Step4: 固定点数 ---
    N = points.shape[0]
    if N >= num_points:
        idx = np.random.choice(N, num_points, replace=False)
        xyz, rgb = points[idx], colors[idx]
    else:
        pad = np.zeros((num_points, 6), dtype=np.float32)
        pad[:N, :3] = points
        pad[:N, 3:] = colors
        return pad

    return np.concatenate([xyz, rgb], axis=-1).astype(np.float32)

# ============ 工具函数 ============
def read_depth_uint16(path):
    depth = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(path)
    return depth.astype(np.float32)

def read_rgb(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def collect_steps(ep_dir):
    img_root = os.path.join(ep_dir, "images")
    rob_root = os.path.join(ep_dir, "robot_data")
    steps = [s for s in os.listdir(img_root) if s.isdigit()]
    steps.sort(key=lambda x: int(x))
    return [(os.path.join(img_root, s), os.path.join(rob_root, s)) for s in steps]

# ============ HDF5 构建 ============
def build_h5(dataset_root, camera_param, output_h5):
    first_ep = sorted([d for d in os.listdir(dataset_root) if d.startswith("episode_")])[0]
    first_depth = os.path.join(dataset_root, first_ep, "images", "1", "camera1_depth.png")
    intr = load_intrinsics(camera_param, first_depth)

    episodes = sorted([d for d in os.listdir(dataset_root) if d.startswith("episode_")],
                      key=lambda x: int(x.split("_")[1]))
    if not episodes:
        raise RuntimeError("没有找到 episode_*")

    os.makedirs(os.path.dirname(output_h5) or ".", exist_ok=True)
    with h5py.File(output_h5, "w") as f:
        root = f.create_group("data")

        for di, ep in enumerate(tqdm(episodes, desc="Converting -> HDF5 pointcloud")):
            ep_dir = os.path.join(dataset_root, ep)
            pairs = collect_steps(ep_dir)
            if not pairs:
                continue

            pcs, qposes = [], []
            for img_dir, rob_dir in pairs:
                depth_mm = read_depth_uint16(os.path.join(img_dir, "camera1_depth.png"))
                rgb = read_rgb(os.path.join(img_dir, "camera1_rgb.png"))
                pc = create_colored_point_cloud(depth_mm, rgb, intr)
                pcs.append(pc)

                with open(os.path.join(rob_dir, "robot_data.json"), "r") as jf:
                    d = json.load(jf)
                tcp, grip = d["tcp_pose"], d["gripper"]
                qposes.append(np.array(tcp + [grip], dtype=np.float32))

            pcs = np.stack(pcs, axis=0).astype(np.float32)     # (T, N, 6)
            qposes = np.stack(qposes, axis=0).astype(np.float32)  # (T, 7)

            # 构建 action: 下一个 qpos
            T = qposes.shape[0]
            actions = np.empty_like(qposes)
            if T == 1:
                actions[0] = qposes[0]
            else:
                actions[:-1] = qposes[1:]
                actions[-1] = actions[-2]

            demo = root.create_group(f"demo_{di}")
            obs = demo.create_group("obs")
            obs.create_dataset("front_pc", data=pcs, dtype="float32")
            obs.create_dataset("qpos", data=qposes, dtype="float32")
            demo.create_dataset("action", data=actions, dtype="float32")
            demo.attrs["episode_name"] = ep
            demo.attrs["length"] = int(T)

    print(f"[OK] 写入完成: {output_h5}")

# ============ 可视化 ============
def visualize_demo_frame(h5_path, demo_idx=0, frame_idx=0):
    with h5py.File(h5_path, "r") as f:
        demo_name = f"data/demo_{demo_idx}"
        pc = f[demo_name]["obs"]["front_pc"][frame_idx]  # (N,6)

    xyz, rgb = pc[:, :3], pc[:, 3:]
    print(f"[Stats] demo_{demo_idx}, frame {frame_idx}")
    print(f"  点数: {xyz.shape[0]}, z范围: {xyz[:,2].min():.3f} ~ {xyz[:,2].max():.3f} m")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    o3d.visualization.draw_geometries([pcd])

# ============ 主函数 ============
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_root", required=True)
    parser.add_argument("--camera_param", required=True)
    parser.add_argument("--output_h5", default="pointcloud_dataset.hdf5")
    parser.add_argument("--viz_demo", type=int, default=None)
    parser.add_argument("--viz_frame", type=int, default=0)
    args = parser.parse_args()

    build_h5(args.dataset_root, args.camera_param, args.output_h5)

    if args.viz_demo is not None:
        visualize_demo_frame(args.output_h5, args.viz_demo, args.viz_frame)


