#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import traceback
from pathlib import Path
from typing import Any

import numpy as np

if __package__ is None or __package__ == "":
    import sys

    sys.path.append(str(Path(__file__).resolve().parent))

from common import now_utc_iso, repo_root


def compare_exr(ref_exr: Path, test_exr: Path, diff_exr: Path, variant: str = "scalar_spectral") -> dict[str, Any]:
    import time

    start = time.perf_counter()
    try:
        import mitsuba as mi

        mi.set_variant(variant)

        ref_bitmap = mi.Bitmap(str(ref_exr)).convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.Float32, False)
        test_bitmap = mi.Bitmap(str(test_exr)).convert(mi.Bitmap.PixelFormat.RGB, mi.Struct.Type.Float32, False)

        ref = np.array(ref_bitmap, copy=False)
        test = np.array(test_bitmap, copy=False)

        if ref.shape != test.shape:
            return {
                "ok": False,
                "error_code": "COMPARE_IO_ERROR",
                "error": f"Shape mismatch: ref={ref.shape}, test={test.shape}",
                "duration_ms": int((time.perf_counter() - start) * 1000),
                "timestamp_utc": now_utc_iso(),
            }

        diff = np.abs(ref - test)
        mse = float(np.mean((ref - test) ** 2))
        ref_rms = float(np.sqrt(np.mean(ref**2)))
        nrmse = float(np.sqrt(mse) / max(1e-6, ref_rms))
        p99 = float(np.percentile(np.abs(ref), 99.0))
        psnr = float(20.0 * np.log10(max(1e-6, p99)) - 10.0 * np.log10(max(1e-12, mse)))
        max_abs = float(np.max(diff))

        diff_exr.parent.mkdir(parents=True, exist_ok=True)
        mi.util.write_bitmap(str(diff_exr), diff)

        return {
            "ok": True,
            "error_code": "",
            "timestamp_utc": now_utc_iso(),
            "duration_ms": int((time.perf_counter() - start) * 1000),
            "metrics": {
                "mse": mse,
                "nrmse": nrmse,
                "psnr": psnr,
                "max_abs": max_abs,
            },
            "ref_shape": list(ref.shape),
            "diff_exr": str(diff_exr),
        }
    except Exception:  # noqa: BLE001
        return {
            "ok": False,
            "error_code": "COMPARE_METRIC_ERROR",
            "timestamp_utc": now_utc_iso(),
            "duration_ms": int((time.perf_counter() - start) * 1000),
            "error": traceback.format_exc(),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare two EXR files and emit metrics + diff EXR.")
    parser.add_argument("--ref-exr", required=True)
    parser.add_argument("--test-exr", required=True)
    parser.add_argument("--diff-exr", required=True)
    parser.add_argument("--variant", default="scalar_spectral")
    args = parser.parse_args()

    root = repo_root()
    ref_exr = Path(args.ref_exr)
    if not ref_exr.is_absolute():
        ref_exr = root / ref_exr
    test_exr = Path(args.test_exr)
    if not test_exr.is_absolute():
        test_exr = root / test_exr
    diff_exr = Path(args.diff_exr)
    if not diff_exr.is_absolute():
        diff_exr = root / diff_exr

    res = compare_exr(ref_exr=ref_exr, test_exr=test_exr, diff_exr=diff_exr, variant=args.variant)
    print(json.dumps(res, ensure_ascii=True, indent=2))
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
