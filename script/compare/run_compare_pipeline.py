#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

if __package__ is None or __package__ == "":
    import sys

    sys.path.append(str(Path(__file__).resolve().parent))

from common import now_utc_iso, repo_root, scene_id_from_relpath, write_json, write_text
from compare_images import compare_exr
from render_mitsuba3 import run_mitsuba3
from render_pbpt import run_pbpt


def load_scene_list(path: Path) -> list[str]:
    rows: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        rows.append(stripped)
    return rows


def build_summary_md(records: list[dict[str, Any]], run_id: str) -> str:
    total = len(records)
    comparable = sum(1 for r in records if r["status"] == "COMPARABLE")
    skip = sum(1 for r in records if r["status"] == "SKIP_OUT_OF_SCOPE")
    failed = total - comparable - skip

    lines = [
        f"# Compare Run {run_id}",
        "",
        f"- generated_utc: {now_utc_iso()}",
        f"- total_records: {total}",
        f"- comparable: {comparable}",
        f"- failed: {failed}",
        f"- skipped_out_of_scope: {skip}",
        "",
        "## Top comparable metrics (worst NRMSE first)",
        "",
        "| scene | nrmse | psnr | max_abs |",
        "|---|---:|---:|---:|",
    ]

    metrics = [r for r in records if r["status"] == "COMPARABLE"]
    metrics.sort(key=lambda x: float(x.get("nrmse", 0.0)), reverse=True)
    for row in metrics[:20]:
        lines.append(
            f"| {row['scene_path']} | {float(row.get('nrmse', 0.0)):.6f} | "
            f"{float(row.get('psnr', 0.0)):.4f} | {float(row.get('max_abs', 0.0)):.6f} |"
        )

    if not metrics:
        lines.append("| (none) | - | - | - |")

    return "\n".join(lines) + "\n"


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run PBPT vs Mitsuba3 comparable pipeline."
    )
    parser.add_argument("--scene-list", default="script/compare/in_scope_scenes.txt")
    parser.add_argument(
        "--out-of-scope-list", default="script/compare/out_of_scope_scenes.txt"
    )
    parser.add_argument("--output-root", default="output/compare_mi3")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--pbpt-binary", default="build/Release/examples/pbpt_cli")
    parser.add_argument("--spp-pbpt", type=int, default=16)
    parser.add_argument("--spp-mi3", type=int, default=16)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--variant", default="llvm_ad_spectral")
    parser.add_argument("--timeout-sec", type=int, default=300)
    parser.add_argument("--max-parallel-jobs", type=int, default=1)
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Debug: process only first N in-scope scenes.",
    )
    args = parser.parse_args()

    root = repo_root()
    scene_list = Path(args.scene_list)
    if not scene_list.is_absolute():
        scene_list = root / scene_list
    out_scope_list = Path(args.out_of_scope_list)
    if not out_scope_list.is_absolute():
        out_scope_list = root / out_scope_list

    run_id = args.run_id.strip() or now_utc_iso().replace(":", "").replace("-", "")
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = root / output_root
    run_root = output_root / run_id
    run_root.mkdir(parents=True, exist_ok=True)

    if args.max_parallel_jobs != 1:
        print("max_parallel_jobs != 1 is currently not implemented; falling back to 1")

    in_scope = load_scene_list(scene_list)
    out_scope = load_scene_list(out_scope_list) if out_scope_list.exists() else []
    if args.limit > 0:
        in_scope = in_scope[: args.limit]

    pbpt_binary = Path(args.pbpt_binary)
    if not pbpt_binary.is_absolute():
        pbpt_binary = root / pbpt_binary

    summary_rows: list[dict[str, Any]] = []

    for scene_rel in in_scope:
        scene_abs = root / scene_rel
        scene_id = scene_id_from_relpath(scene_rel)
        scene_out_dir = run_root / scene_id
        scene_out_dir.mkdir(parents=True, exist_ok=True)

        pbpt_exr = scene_out_dir / "pbpt.exr"
        mi3_exr = scene_out_dir / "mi3.exr"
        diff_exr = scene_out_dir / "diff.exr"
        pbpt_log = scene_out_dir / "pbpt.log"
        mi3_log = scene_out_dir / "mi3.log"
        meta_json = scene_out_dir / "meta.json"

        pbpt_result = run_pbpt(
            scene_path=scene_abs,
            out_exr=pbpt_exr,
            spp=args.spp_pbpt,
            timeout_sec=args.timeout_sec,
            pbpt_binary=pbpt_binary,
        )
        pbpt_raw = pbpt_result["result"]
        write_text(
            pbpt_log,
            (
                f"# PBPT command\n{' '.join(pbpt_result['command'])}\n\n"
                f"# Exit\n{pbpt_raw['exit_code']}\n\n"
                f"# Stdout\n{pbpt_raw['stdout']}\n\n"
                f"# Stderr\n{pbpt_raw['stderr']}\n"
            ),
        )

        mi3_result = run_mitsuba3(
            scene_path=scene_abs,
            out_exr=mi3_exr,
            spp=args.spp_mi3,
            seed=args.seed,
            variant=args.variant,
            timeout_sec=args.timeout_sec,
            resource_dirs=[scene_abs.parent, root],
        )
        mi3_raw = mi3_result["result"]
        write_text(
            mi3_log,
            (
                f"# Mitsuba3 command\n{' '.join(mi3_result['command'])}\n\n"
                f"# Exit\n{mi3_raw['exit_code']}\n\n"
                f"# Stdout\n{mi3_raw['stdout']}\n\n"
                f"# Stderr\n{mi3_raw['stderr']}\n"
            ),
        )

        compare_result: dict[str, Any]
        if pbpt_result["ok"] and mi3_result.get("ok", False):
            compare_result = compare_exr(
                ref_exr=mi3_exr,
                test_exr=pbpt_exr,
                diff_exr=diff_exr,
                variant=args.variant,
            )
        else:
            compare_result = {
                "ok": False,
                "error_code": "",
                "duration_ms": 0,
                "metrics": {},
            }

        status = "COMPARABLE"
        if not pbpt_result["ok"]:
            status = pbpt_result["error_code"]
        elif not mi3_result.get("ok", False):
            status = mi3_result.get("error_code", "MI3_RENDER_ERROR")
        elif not compare_result["ok"]:
            status = compare_result.get("error_code", "COMPARE_METRIC_ERROR")

        meta = {
            "timestamp_utc": now_utc_iso(),
            "run_id": run_id,
            "scene_rel": scene_rel,
            "scene_abs": str(scene_abs),
            "scene_id": scene_id,
            "status": status,
            "params": {
                "spp_pbpt": args.spp_pbpt,
                "spp_mi3": args.spp_mi3,
                "seed": args.seed,
                "variant": args.variant,
                "timeout_sec": args.timeout_sec,
            },
            "paths": {
                "pbpt_exr": str(pbpt_exr),
                "mi3_exr": str(mi3_exr),
                "diff_exr": str(diff_exr),
                "pbpt_log": str(pbpt_log),
                "mi3_log": str(mi3_log),
            },
            "pbpt": pbpt_result,
            "mi3": mi3_result,
            "compare": compare_result,
        }
        write_json(meta_json, meta)

        metrics = compare_result.get("metrics", {}) if compare_result.get("ok") else {}

        summary_rows.append(
            {
                "scene_path": scene_rel,
                "scope": "IN_SCOPE",
                "scene_id": scene_id,
                "status": status,
                "pbpt_exit_code": pbpt_result["result"]["exit_code"],
                "mi3_exit_code": mi3_result["result"]["exit_code"],
                "pbpt_duration_ms": pbpt_result["result"]["duration_ms"],
                "mi3_duration_ms": mi3_result["result"]["duration_ms"],
                "compare_duration_ms": compare_result.get("duration_ms", 0),
                "nrmse": metrics.get("nrmse", ""),
                "psnr": metrics.get("psnr", ""),
                "max_abs": metrics.get("max_abs", ""),
                "pbpt_out_exr": str(pbpt_exr),
                "mi3_out_exr": str(mi3_exr),
                "diff_exr": str(diff_exr),
                "pbpt_log": str(pbpt_log),
                "mi3_log": str(mi3_log),
                "meta_json": str(meta_json),
            }
        )

    for scene_rel in out_scope:
        scene_id = scene_id_from_relpath(scene_rel)
        summary_rows.append(
            {
                "scene_path": scene_rel,
                "scope": "OUT_OF_SCOPE",
                "scene_id": scene_id,
                "status": "SKIP_OUT_OF_SCOPE",
                "pbpt_exit_code": "",
                "mi3_exit_code": "",
                "pbpt_duration_ms": "",
                "mi3_duration_ms": "",
                "compare_duration_ms": "",
                "nrmse": "",
                "psnr": "",
                "max_abs": "",
                "pbpt_out_exr": "",
                "mi3_out_exr": "",
                "diff_exr": "",
                "pbpt_log": "",
                "mi3_log": "",
                "meta_json": "",
            }
        )

    fieldnames = [
        "scene_path",
        "scope",
        "scene_id",
        "status",
        "pbpt_exit_code",
        "mi3_exit_code",
        "pbpt_duration_ms",
        "mi3_duration_ms",
        "compare_duration_ms",
        "nrmse",
        "psnr",
        "max_abs",
        "pbpt_out_exr",
        "mi3_out_exr",
        "diff_exr",
        "pbpt_log",
        "mi3_log",
        "meta_json",
    ]

    summary_csv = run_root / "summary.csv"
    failures_csv = run_root / "failures.csv"
    metrics_csv = run_root / "metrics.csv"
    summary_md = run_root / "summary.md"

    write_csv(summary_csv, summary_rows, fieldnames)

    failure_rows = [
        r
        for r in summary_rows
        if r["status"] not in {"COMPARABLE", "SKIP_OUT_OF_SCOPE"}
    ]
    write_csv(failures_csv, failure_rows, fieldnames)

    metric_rows = [
        {
            "scene_path": r["scene_path"],
            "scene_id": r["scene_id"],
            "nrmse": r["nrmse"],
            "psnr": r["psnr"],
            "max_abs": r["max_abs"],
            "diff_exr": r["diff_exr"],
        }
        for r in summary_rows
        if r["status"] == "COMPARABLE"
    ]
    write_csv(
        metrics_csv,
        metric_rows,
        ["scene_path", "scene_id", "nrmse", "psnr", "max_abs", "diff_exr"],
    )

    write_text(summary_md, build_summary_md(summary_rows, run_id))

    print(
        json.dumps(
            {
                "run_id": run_id,
                "summary_csv": str(summary_csv),
                "failures_csv": str(failures_csv),
                "metrics_csv": str(metrics_csv),
                "summary_md": str(summary_md),
                "record_count": len(summary_rows),
                "in_scope_count": len(in_scope),
                "out_of_scope_count": len(out_scope),
            },
            ensure_ascii=True,
            indent=2,
        )
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
