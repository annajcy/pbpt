#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import traceback
from pathlib import Path
from typing import Any

if __package__ is None or __package__ == "":
    import sys

    sys.path.append(str(Path(__file__).resolve().parent))

from common import classify_mi3_error, now_utc_iso, repo_root, write_text


def _worker(
    scene_path: str,
    out_exr: str,
    spp: int,
    seed: int,
    variant: str,
    resource_dirs: list[str],
    queue: Any,
) -> None:
    try:
        global mi, dr
        import mitsuba as mi
        import drjit as dr

        mi.set_variant(variant)
        resolver = mi.Thread.thread().file_resolver()
        for resource_dir in resource_dirs:
            resolver.append(resource_dir)

        import drjit as dr

        class SimplePathIntegrator(mi.SamplingIntegrator):
            def __init__(self, props=mi.Properties()):
                super().__init__(props)
                self.max_depth = props.get("max_depth", -1)
                self.rr_depth = props.get("rr_depth", 5)

            def sample(self, scene, sampler, ray_, medium, active):
                ray = mi.Ray3f(ray_)
                active = mi.Bool(active)
                throughput = mi.Spectrum(1.0)
                result = mi.Spectrum(0.0)
                depth = mi.UInt32(0)
                eta = mi.Float(1.0)

                valid_ray = scene.ray_intersect(ray_, active).is_valid()
                max_depth = mi.UInt32(self.max_depth if self.max_depth >= 0 else 100)
                rr_depth = mi.UInt32(self.rr_depth)

                @dr.syntax
                def run_loop(
                    scene,
                    sampler,
                    ray,
                    throughput,
                    result,
                    eta,
                    depth,
                    active,
                    max_depth,
                    rr_depth,
                ):
                    while active:
                        si = scene.ray_intersect(ray, active)
                        escaped = active & ~si.is_valid()
                        env = scene.environment()
                        if env is not None:
                            result += dr.select(
                                escaped, throughput * env.eval(si, escaped), 0.0
                            )
                        active &= si.is_valid()

                        emitter = si.emitter(scene, active)
                        hit_emitter = active & (emitter != None)
                        result += dr.select(
                            hit_emitter, throughput * emitter.eval(si, hit_emitter), 0.0
                        )

                        bsdf = si.bsdf(ray)
                        active &= bsdf != None

                        ctx = mi.BSDFContext()
                        bs, bsdf_val = bsdf.sample(
                            ctx,
                            si,
                            sampler.next_1d(active),
                            sampler.next_2d(active),
                            active,
                        )
                        throughput[active] *= bsdf_val
                        active &= dr.max(throughput) != 0.0

                        ray[active] = si.spawn_ray(si.to_world(bs.wo))
                        eta[active] *= bs.eta
                        depth += 1
                        rr_active = active & (depth >= rr_depth)
                        q = dr.minimum(dr.max(throughput) * dr.square(eta), 0.95)
                        rr_continue = sampler.next_1d(rr_active) < q
                        throughput[rr_active] *= dr.rcp(dr.detach(q))
                        active &= ~rr_active | rr_continue

                        if max_depth > 0:
                            active &= depth < max_depth

                    return result

                result = run_loop(
                    scene,
                    sampler,
                    ray,
                    throughput,
                    result,
                    eta,
                    depth,
                    active,
                    max_depth,
                    rr_depth,
                )
                return result, valid_ray, []

        mi.register_integrator("simple_path", lambda props: SimplePathIntegrator(props))

        scene = mi.load_file(scene_path)
        image = mi.render(scene, spp=spp, seed=seed)
        mi.util.write_bitmap(out_exr, image)
        queue.put({"ok": True, "traceback": "", "stdout": ""})
    except Exception:  # noqa: BLE001
        queue.put({"ok": False, "traceback": traceback.format_exc(), "stdout": ""})


def run_mitsuba3(
    scene_path: Path,
    out_exr: Path,
    spp: int,
    seed: int,
    variant: str,
    timeout_sec: int,
    resource_dirs: list[Path] | None = None,
) -> dict[str, Any]:
    import time

    out_exr.parent.mkdir(parents=True, exist_ok=True)

    ctx = mp.get_context("spawn")
    queue: Any = ctx.Queue()

    resource_dirs = resource_dirs or []
    resource_dirs_str = [str(p) for p in resource_dirs]

    start = time.perf_counter()
    proc = ctx.Process(
        target=_worker,
        args=(
            str(scene_path),
            str(out_exr),
            spp,
            seed,
            variant,
            resource_dirs_str,
            queue,
        ),
    )
    proc.start()
    proc.join(timeout=timeout_sec)

    timed_out = False
    exit_code = -1
    trace = ""
    stdout = ""

    if proc.is_alive():
        timed_out = True
        proc.terminate()
        proc.join()
        exit_code = 124
    else:
        exit_code = proc.exitcode if proc.exitcode is not None else -1
        if not queue.empty():
            result = queue.get()
            trace = result.get("traceback", "")
            stdout = result.get("stdout", "")
        elif exit_code != 0:
            trace = "Mitsuba subprocess exited unexpectedly without traceback payload."

    duration_ms = int((time.perf_counter() - start) * 1000)
    out_exists = out_exr.exists()
    error_code = classify_mi3_error(
        timed_out=timed_out,
        traceback_text=trace,
        exit_code=exit_code,
        out_exr_exists=out_exists,
    )

    return {
        "ok": error_code == "",
        "error_code": error_code,
        "timestamp_utc": now_utc_iso(),
        "scene_path": str(scene_path),
        "out_exr": str(out_exr),
        "out_exr_exists": out_exists,
        "spp": spp,
        "seed": seed,
        "variant": variant,
        "timeout_sec": timeout_sec,
        "command": [
            "python",
            "render_mitsuba3.py",
            "--scene-path",
            str(scene_path),
            "--out-exr",
            str(out_exr),
            "--spp",
            str(spp),
            "--seed",
            str(seed),
            "--variant",
            variant,
        ],
        "resource_dirs": resource_dirs_str,
        "result": {
            "command": ["mitsuba-python-worker"],
            "cwd": str(repo_root()),
            "exit_code": exit_code,
            "duration_ms": duration_ms,
            "stdout": stdout,
            "stderr": trace,
            "timed_out": timed_out,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render scene with Mitsuba3 Python API."
    )
    parser.add_argument("--scene-path", required=True)
    parser.add_argument("--out-exr", required=True)
    parser.add_argument("--spp", type=int, default=16)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--variant", default="llvm_ad_spectral")
    parser.add_argument("--timeout-sec", type=int, default=300)
    parser.add_argument("--resource-dir", action="append", default=[])
    parser.add_argument("--log-path", default="")
    args = parser.parse_args()

    root = repo_root()
    scene_path = Path(args.scene_path)
    if not scene_path.is_absolute():
        scene_path = root / scene_path

    out_exr = Path(args.out_exr)
    if not out_exr.is_absolute():
        out_exr = root / out_exr

    res = run_mitsuba3(
        scene_path=scene_path,
        out_exr=out_exr,
        spp=args.spp,
        seed=args.seed,
        variant=args.variant,
        timeout_sec=args.timeout_sec,
        resource_dirs=[Path(p) for p in args.resource_dir],
    )

    if args.log_path:
        log_path = Path(args.log_path)
        if not log_path.is_absolute():
            log_path = root / log_path
        raw = res["result"]
        log_content = (
            f"# Mitsuba3 pseudo-command\n{' '.join(res['command'])}\n\n"
            f"# Exit\n{raw['exit_code']}\n\n"
            f"# Stdout\n{raw['stdout']}\n\n"
            f"# Stderr\n{raw['stderr']}\n"
        )
        write_text(log_path, log_content)

    print(json.dumps(res, ensure_ascii=True, indent=2))
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
