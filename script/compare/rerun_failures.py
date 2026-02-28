#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parent))

from common import repo_root


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Rerun failed scenes from a failures.csv file."
    )
    parser.add_argument(
        "--failures_csv",
        help="Path to failures.csv",
    )
    parser.add_argument(
        "failures_csv_pos",
        nargs="?",
        help="Path to failures.csv (positional)",
    )
    parser.add_argument(
        "--timeout-sec",
        type=int,
        default=2800,
        help="Override timeout for the rerun",
    )
    args, extra_args = parser.parse_known_args()

    csv_path_str = args.failures_csv or args.failures_csv_pos
    if not csv_path_str:
        parser.error("failures_csv argument is missing. Please provide it.")
    csv_path = Path(csv_path_str)
    if not csv_path.is_absolute():
        csv_path = repo_root() / csv_path

    if not csv_path.exists():
        print(f"Error: {csv_path} does not exist", file=sys.stderr)
        return 1

    scene_paths = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("status") not in {"COMPARABLE", "SKIP_OUT_OF_SCOPE"}:
                scene_paths.append(row["scene_path"])

    if not scene_paths:
        print("No failed scenes found in the CSV.")
        return 0

    print(f"Found {len(scene_paths)} failed scenes to rerun.")

    tmp_list = csv_path.parent / "rerun_scene_list.txt"
    with tmp_list.open("w", encoding="utf-8") as f:
        for p in scene_paths:
            f.write(f"{p}\n")

    print(f"Created temporary scene list at {tmp_list}")

    pipeline_script = Path(__file__).resolve().parent / "run_compare_pipeline.py"

    cmd = [
        sys.executable,
        str(pipeline_script),
        "--scene-list",
        str(tmp_list),
        "--timeout-sec",
        str(args.timeout_sec),
    ] + extra_args

    print(f"Running command: {' '.join(cmd)}")

    result = subprocess.run(cmd, cwd=str(repo_root()))

    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
