import os
import json
import h5py
import cv2
import numpy as np
import argparse
from tqdm import tqdm

TARGET_SIZE = (256, 256)   # 统一图像尺寸

def pad_last_dim_to(x: np.ndarray, target_dim: int, pad_value: float = 0.0) -> np.ndarray:
    if x.ndim == 0:
        raise ValueError("pad_last_dim_to expects an array with at least 1 dimension")
    last_dim = x.shape[-1]
    if last_dim == target_dim:
        return x
    if last_dim > target_dim:
        raise ValueError(f"Cannot pad last dim from {last_dim} to smaller target_dim={target_dim}")
    pad_width = [(0, 0)] * x.ndim
    pad_width[-1] = (0, target_dim - last_dim)
    return np.pad(x, pad_width=pad_width, mode="constant", constant_values=pad_value)

def list_steps(episode_dir):
    img_root = os.path.join(episode_dir, "images")
    rob_root = os.path.join(episode_dir, "robot_data")
    if not (os.path.isdir(img_root) and os.path.isdir(rob_root)):
        return []
    steps = [int(d) for d in os.listdir(img_root) if d.isdigit() and os.path.isdir(os.path.join(img_root, d))]
    steps.sort()
    return [(str(s), os.path.join(img_root, str(s)), os.path.join(rob_root, str(s))) for s in steps]

def read_rgb(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {path}")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # 这里统一 resize 为 224×224
    img = cv2.resize(img, TARGET_SIZE, interpolation=cv2.INTER_AREA)
    return img

def read_robot_json(path):
    with open(path, "r") as f:
        d = json.load(f)
    tcp = d.get("tcp_pose")   # [x,y,z,rx,ry,rz]
    grip = d.get("gripper")   # 0/1
    if tcp is None or grip is None:
        raise RuntimeError(f"Missing tcp_pose/gripper in {path}")
    tcp = np.asarray(tcp, dtype=np.float32)
    grip = np.asarray([float(grip)], dtype=np.float32)
    return np.concatenate([tcp, grip], axis=0).astype(np.float32)  # (7,)

def load_episode(ep_dir, pad: bool = False):
    steps = list_steps(ep_dir)
    if not steps:
        raise RuntimeError(f"No steps in {ep_dir}")

    fronts, wrists, eefs = [], [], []

    for _, img_dir, rob_dir in steps:
        fronts.append(read_rgb(os.path.join(img_dir, "camera1_rgb.png")))  # 相机1 → front
        wrists.append(read_rgb(os.path.join(img_dir, "camera2_rgb.png")))  # 相机2 → wrist
        eefs.append(read_robot_json(os.path.join(rob_dir, "robot_data.json")))

    front = np.stack(fronts, axis=0).astype(np.uint8)
    wrist = np.stack(wrists, axis=0).astype(np.uint8)
    qpos  = np.stack(eefs,   axis=0).astype(np.float32)

    # action[t] = qpos[t+1]，最后一步复制前一步的 action
    T = qpos.shape[0]
    action = np.empty_like(qpos)
    if T == 1:
        action[0] = qpos[0]
    else:
        action[:-1] = qpos[1:]
        action[-1]  = action[-2]

    if pad:
        qpos = pad_last_dim_to(qpos, 8, pad_value=0.0).astype(np.float32, copy=False)
        action = pad_last_dim_to(action, 8, pad_value=0.0).astype(np.float32, copy=False)

    return front, wrist, qpos, action

def main(dataset_root, output_h5, pad: bool = False):
    episodes = sorted([d for d in os.listdir(dataset_root) if d.startswith("episode_")],
                      key=lambda x: int(x.split("_")[1]))
    if not episodes:
        raise RuntimeError(f"No episode_* under {dataset_root}")

    os.makedirs(os.path.dirname(output_h5) or ".", exist_ok=True)
    with h5py.File(output_h5, "w") as f:
        data_group = f.create_group("data")

        for idx, ep in enumerate(tqdm(episodes, desc="Converting")):
            ep_dir = os.path.join(dataset_root, ep)
            front, wrist, qpos, action = load_episode(ep_dir, pad=pad)

            demo = data_group.create_group(f"demo_{idx}")
            obs  = demo.create_group("obs")

            T, H, W, _ = front.shape
            obs.create_dataset("front",  data=front, dtype="uint8",  chunks=(1, H, W, 3))
            obs.create_dataset("wrist",  data=wrist, dtype="uint8",  chunks=(1, H, W, 3))
            obs.create_dataset("qpos",   data=qpos,  dtype="float32", chunks=(min(64, T), qpos.shape[1]))
            demo.create_dataset("action", data=action, dtype="float32", chunks=(min(64, T), action.shape[1]))

            demo.attrs["episode_name"] = ep
            demo.attrs["length"] = int(T)

    print(f"Done. Wrote HDF5 to: {output_h5}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_root", default="data/real_long_new_gripper_1/dataset",
                        help="例如 data/20250825_xxxx/dataset")
    parser.add_argument("--output_h5", default="data/real_long_new_gripper_1/real_long_new_gripper_1.hdf5")
    parser.add_argument("--pad", action="store_true",
                        help="是否把 qpos/action 的最后一维 padding 到 8（不足补 0）")
    args = parser.parse_args()
    main(args.dataset_root, args.output_h5, pad=args.pad)
