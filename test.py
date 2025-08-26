#!/usr/bin/env python3
# h5tree.py
# 用树状结构列出 HDF5 文件内容，支持显示属性/压缩/深度限制/链接类型

import argparse
import os
import sys
from typing import Any
import h5py
import numpy as np

BRANCH_MID = "├── "
BRANCH_END = "└── "
PIPE      = "│   "
EMPTY     = "    "

def trunc(s: str, maxlen: int = 120) -> str:
    s = str(s)
    return s if len(s) <= maxlen else s[:maxlen] + "…"

def fmt_attr_value(v: Any) -> str:
    # 尽量把属性值友好地显示出来
    try:
        if isinstance(v, (bytes, bytearray)):
            try:
                return repr(v.decode("utf-8"))
            except Exception:
                return repr(v)
        if isinstance(v, np.ndarray):
            if v.size > 8:
                return f"array(shape={v.shape}, dtype={v.dtype})"
            return np.array2string(v, threshold=8)
        return repr(v)
    except Exception:
        return "<unprintable>"

def print_attrs(obj, prefix: str, show_attrs: bool, max_attrs: int):
    if not show_attrs:
        return
    try:
        keys = sorted(list(obj.attrs.keys()))
    except Exception:
        return
    if not keys:
        return
    shown = 0
    for i, k in enumerate(keys):
        if shown >= max_attrs:
            print(prefix + f"… ({len(keys) - shown} more attrs)")
            break
        try:
            v = obj.attrs[k]
        except Exception:
            v = "<error reading attr>"
        print(prefix + f"@{k} = {trunc(fmt_attr_value(v))}")
        shown += 1

def fmt_filters(dset: h5py.Dataset) -> str:
    parts = []
    try:
        if dset.compression is not None:
            parts.append(f"compression={dset.compression}({dset.compression_opts})")
        if dset.shuffle:
            parts.append("shuffle")
        if dset.fletcher32:
            parts.append("fletcher32")
        if dset.scaleoffset is not None:
            parts.append(f"scaleoffset={dset.scaleoffset}")
        if dset.chunks is not None:
            parts.append(f"chunks={dset.chunks}")
        if dset.fillvalue is not None:
            parts.append(f"fillvalue={dset.fillvalue}")
    except Exception:
        parts.append("filters=<error>")
    return ", ".join(parts) if parts else "no-filters"

def dump_group(
    g: h5py.Group,
    name: str,
    prefix: str,
    depth: int,
    max_depth: int,
    show_attrs: bool,
    max_attrs: int,
    follow_links: bool,
):
    # 打印当前组
    header = f"[group] {name if name else '/'}"
    print(prefix + header)
    print_attrs(g, prefix + "    ", show_attrs, max_attrs)

    if max_depth is not None and depth >= max_depth:
        print(prefix + "    " + "… (max depth reached)")
        return

    # 列出子项
    try:
        names = sorted(list(g.keys()))
    except Exception as e:
        print(prefix + "    " + f"<error listing children: {e}>")
        return

    for idx, child_name in enumerate(names):
        is_last = (idx == len(names) - 1)
        branch = BRANCH_END if is_last else BRANCH_MID
        child_prefix = prefix + (EMPTY if is_last else PIPE)

        # 先看链接类型
        try:
            link = g.get(child_name, getlink=True)
        except Exception as e:
            print(prefix + branch + f"{child_name} <error getting link: {e}>")
            continue

        if isinstance(link, h5py.SoftLink):
            print(prefix + branch + f"{child_name} -> [softlink] {link.path}")
            if not follow_links:
                continue
        elif isinstance(link, h5py.ExternalLink):
            print(prefix + branch + f"{child_name} -> [externallink] {link.filename}:{link.path}")
            if not follow_links:
                continue
        # HardLink 或未知时尝试打开对象
        try:
            obj = g[child_name]
        except Exception as e:
            print(prefix + branch + f"{child_name} <error opening object: {e}>")
            continue

        if isinstance(obj, h5py.Group):
            print(child_prefix[:-4] + (BRANCH_END if is_last else BRANCH_MID) + f"[group] {child_name}")
            print_attrs(obj, child_prefix + "    ", show_attrs, max_attrs)
            dump_group(
                obj,
                child_name,
                child_prefix,
                depth + 1,
                max_depth,
                show_attrs,
                max_attrs,
                follow_links,
            )
        elif isinstance(obj, h5py.Dataset):
            try:
                shape = obj.shape
                dtype = obj.dtype
            except Exception:
                shape = "<?>"
                dtype = "<?>"
            line = f"[dataset] {child_name} shape={shape}, dtype={dtype}"
            # 过滤器/压缩信息
            line += " | " + fmt_filters(obj)
            print(prefix + branch + line)
            print_attrs(obj, child_prefix + "    ", show_attrs, max_attrs)
        else:
            print(prefix + branch + f"{child_name} [unknown type: {type(obj)}]")

def main():
    p = argparse.ArgumentParser(
        description="以树状结构查看 HDF5 文件内容（组/数据集/属性/压缩/分块等）"
    )
    p.add_argument("file", help="HDF5 文件路径（.h5/.hdf5）")
    p.add_argument("-a", "--attrs", action="store_true", help="显示属性（默认不显示）")
    p.add_argument("--max-attrs", type=int, default=50, help="每个对象最多显示多少个属性（默认 50）")
    p.add_argument("-d", "--max-depth", type=int, default=None, help="最大递归深度（默认不限制）")
    p.add_argument("--no-follow-links", action="store_true", help="不跟随软链接/外部链接（只显示链接信息）")

    args = p.parse_args()

    if not os.path.exists(args.file):
        print(f"文件不存在: {args.file}", file=sys.stderr)
        sys.exit(1)

    try:
        with h5py.File(args.file, "r") as f:
            print(f"# HDF5 file: {args.file}")
            dump_group(
                f,
                name="",
                prefix="",
                depth=0,
                max_depth=args.max_depth,
                show_attrs=args.attrs,
                max_attrs=args.max_attrs,
                follow_links=not args.no_follow_links,
            )
    except OSError as e:
        print(f"无法打开 HDF5 文件: {e}", file=sys.stderr)
        sys.exit(2)

if __name__ == "__main__":
    main()
