#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

if __package__ is None or __package__ == "":
    import sys

    sys.path.append(str(Path(__file__).resolve().parent))

from common import (
    classify_pbpt_error,
    now_utc_iso,
    repo_root,
    run_subprocess,
    write_text,
)


def run_pbpt(scene_path: Path, out_exr: Path, spp: int, timeout_sec: int, pbpt_binary: Path) -> dict[str, Any]:
    out_exr.parent.mkdir(parents=True, exist_ok=True)
    command = [str(pbpt_binary), str(scene_path), str(out_exr), str(spp)]
    result = run_subprocess(command=command, cwd=repo_root(), timeout_sec=timeout_sec)
    out_exists = out_exr.exists()
    error_code = classify_pbpt_error(result, out_exists)

    return {
        "ok": error_code == "",
        "error_code": error_code,
        "timestamp_utc": now_utc_iso(),
        "scene_path": str(scene_path),
        "out_exr": str(out_exr),
        "out_exr_exists": out_exists,
        "spp": spp,
        "timeout_sec": timeout_sec,
        "command": command,
        "result": asdict(result),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Render scene with PBPT pbpt_cli executable.")
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--out-exr", required=True)
    parser.add_argument("--spp", type=int, default=16)
    parser.add_argument("--timeout-sec", type=int, default=300)
    parser.add_argument("--binary", default="build/Release/examples/pbpt_cli")
    parser.add_argument("--log-path", default="")
    args = parser.parse_args()

    root = repo_root()
    scene_path = Path(args.scene_path)
    if not scene_path.is_absolute():
        scene_path = root / scene_path

    out_exr = Path(args.out_exr)
    if not out_exr.is_absolute():
        out_exr = root / out_exr

    binary = Path(args.binary)
    if not binary.is_absolute():
        binary = root / binary

    res = run_pbpt(scene_path=scene_path, out_exr=out_exr, spp=args.spp, timeout_sec=args.timeout_sec, pbpt_binary=binary)

    if args.log_path:
        log_path = Path(args.log_path)
        if not log_path.is_absolute():
            log_path = root / log_path
        raw = res["result"]
        log_content = (
            f"# PBPT command\n{' '.join(res['command'])}\n\n"
            f"# Exit\n{raw['exit_code']}\n\n"
            f"# Stdout\n{raw['stdout']}\n\n"
            f"# Stderr\n{raw['stderr']}\n"
        )
        write_text(log_path, log_content)

    print(json.dumps(res, ensure_ascii=True, indent=2))
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
