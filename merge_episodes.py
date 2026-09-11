#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator


EP_RE = re.compile(r"^episode_(\d+)$")


@dataclass(frozen=True)
class Episode:
    num: int
    path: Path


def _episode_num_from_name(name: str) -> int | None:
    m = EP_RE.match(name)
    if not m:
        return None
    try:
        return int(m.group(1))
    except ValueError:
        return None


def _as_dataset_dir(p: Path) -> Path:
    p = p.resolve()
    if p.name == "dataset":
        return p
    if (p / "dataset").is_dir():
        return (p / "dataset").resolve()
    raise FileNotFoundError(f"not a dataset dir (and no dataset/ inside): {p}")


def _iter_episode_dirs(dataset_dir: Path) -> list[Episode]:
    episodes: list[Episode] = []
    try:
        with os.scandir(dataset_dir) as it:
            for entry in it:
                if not entry.is_dir(follow_symlinks=False):
                    continue
                num = _episode_num_from_name(entry.name)
                if num is None:
                    continue
                episodes.append(Episode(num=num, path=Path(entry.path)))
    except FileNotFoundError:
        raise

    episodes.sort(key=lambda e: e.num)
    return episodes


def _max_existing_episode_num(dataset_dir: Path) -> int:
    if not dataset_dir.exists():
        return 0
    if not dataset_dir.is_dir():
        raise NotADirectoryError(f"dest dataset is not a directory: {dataset_dir}")
    episodes = _iter_episode_dirs(dataset_dir)
    return max((e.num for e in episodes), default=0)


def _discover_dataset_dirs(src_root: Path) -> list[Path]:
    src_root = src_root.resolve()
    if not src_root.is_dir():
        raise NotADirectoryError(f"--src-root is not a directory: {src_root}")

    out: list[Path] = []
    for child in sorted(src_root.iterdir(), key=lambda p: p.name):
        if not child.is_dir():
            continue
        ds = child / "dataset"
        if ds.is_dir():
            out.append(ds.resolve())
    return out


def _format_mapping(src: Path, dst: Path) -> str:
    return f"{src}  ->  {dst}"


def _copy_episode_dir(src: Path, dst: Path) -> None:
    # symlinks=True preserves symlinks as-is (common for dataset caching setups)
    shutil.copytree(src, dst, symlinks=True)


def _move_episode_dir(src: Path, dst: Path) -> None:
    shutil.move(str(src), str(dst))


def _validate_no_overlap(src_dataset_dirs: Iterable[Path], dest_dataset_dir: Path) -> None:
    dest_dataset_dir = dest_dataset_dir.resolve()
    for ds in src_dataset_dirs:
        ds = ds.resolve()
        if ds == dest_dataset_dir:
            raise ValueError(f"source dataset equals dest dataset: {ds}")
        try:
            dest_dataset_dir.relative_to(ds)
        except ValueError:
            pass
        else:
            raise ValueError(f"dest dataset is inside a source dataset: dest={dest_dataset_dir} src={ds}")
        try:
            ds.relative_to(dest_dataset_dir)
        except ValueError:
            pass
        else:
            raise ValueError(f"a source dataset is inside dest dataset: dest={dest_dataset_dir} src={ds}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Merge multiple dataset/episode_* folders into a single dataset directory, "
            "renumbering episodes continuously."
        )
    )
    p.add_argument(
        "src",
        nargs="*",
        type=Path,
        help="Source dataset dirs (either .../dataset or a parent containing dataset/).",
    )
    p.add_argument(
        "--src-root",
        type=Path,
        default=None,
        help="Auto-discover sources under this root (finds <child>/dataset).",
    )
    p.add_argument(
        "--dest",
        required=True,
        type=Path,
        help="Destination dataset dir (either .../dataset or a parent to create/use dataset/).",
    )
    p.add_argument(
        "--mode",
        choices=("copy", "move"),
        default="copy",
        help="How to integrate: copy keeps originals; move relocates originals.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned operations without creating/moving/copying anything.",
    )
    p.add_argument(
        "--start-from",
        type=int,
        default=None,
        help="Force the first new episode number (overrides auto-detect from dest).",
    )
    return p.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    dest = args.dest
    if dest.name != "dataset":
        dest = dest / "dataset"
    dest = dest.resolve()

    src_dataset_dirs: list[Path] = []
    if args.src_root is not None:
        src_dataset_dirs.extend(_discover_dataset_dirs(args.src_root))
    for s in args.src:
        src_dataset_dirs.append(_as_dataset_dir(s))

    # De-dup while preserving order
    seen: set[Path] = set()
    uniq_src: list[Path] = []
    for ds in src_dataset_dirs:
        ds = ds.resolve()
        if ds in seen:
            continue
        seen.add(ds)
        uniq_src.append(ds)
    src_dataset_dirs = uniq_src

    if not src_dataset_dirs:
        raise SystemExit("no sources found; pass src dirs or --src-root")

    _validate_no_overlap(src_dataset_dirs, dest)

    existing_max = _max_existing_episode_num(dest)
    next_num = args.start_from if args.start_from is not None else existing_max + 1
    if next_num <= 0:
        raise SystemExit("--start-from must be >= 1")

    copier = _copy_episode_dir if args.mode == "copy" else _move_episode_dir

    planned: list[tuple[Path, Path]] = []
    for ds in src_dataset_dirs:
        for ep in _iter_episode_dirs(ds):
            planned.append((ep.path, dest / f"episode_{next_num}"))
            next_num += 1

    if not planned:
        print("No episode_* folders found in sources.", file=sys.stderr)
        return 2

    try:
        for src_path, dst_path in planned:
            print(_format_mapping(src_path, dst_path))
    except BrokenPipeError:
        # e.g. piping to `head` or similar; mimic common CLI behavior
        try:
            sys.stdout.close()
        finally:
            return 0

    if args.dry_run:
        print(f"[dry-run] {len(planned)} episode(s) planned; nothing changed.", file=sys.stderr)
        return 0

    dest.mkdir(parents=True, exist_ok=True)
    for _, dst_path in planned:
        if dst_path.exists():
            raise SystemExit(f"destination already exists: {dst_path}")

    for src_path, dst_path in planned:
        copier(src_path, dst_path)

    print(f"Done. Merged {len(planned)} episode(s) into {dest}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
