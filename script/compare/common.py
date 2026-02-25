#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import subprocess
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass
class CommandResult:
    command: list[str]
    cwd: str
    exit_code: int
    duration_ms: int
    stdout: str
    stderr: str
    timed_out: bool


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def scene_id_from_relpath(scene_rel: str) -> str:
    normalized = scene_rel.replace("\\", "/")
    if normalized.endswith(".xml"):
        normalized = normalized[:-4]
    return normalized.replace("/", "__")


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def write_text(path: Path, content: str) -> None:
    ensure_parent(path)
    path.write_text(content, encoding="utf-8")


def write_json(path: Path, payload: dict[str, Any]) -> None:
    ensure_parent(path)
    path.write_text(json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")


def run_subprocess(command: list[str], cwd: Path, timeout_sec: int) -> CommandResult:
    start = time.perf_counter()
    timed_out = False
    exit_code = -1
    stdout = ""
    stderr = ""

    try:
        completed = subprocess.run(
            command,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
            env=os.environ.copy(),
        )
        exit_code = completed.returncode
        stdout = completed.stdout or ""
        stderr = completed.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        exit_code = 124
        stdout = (exc.stdout or "") if isinstance(exc.stdout, str) else ""
        stderr = (exc.stderr or "") if isinstance(exc.stderr, str) else ""

    duration_ms = int((time.perf_counter() - start) * 1000)
    return CommandResult(
        command=command,
        cwd=str(cwd),
        exit_code=exit_code,
        duration_ms=duration_ms,
        stdout=stdout,
        stderr=stderr,
        timed_out=timed_out,
    )


def classify_pbpt_error(result: CommandResult, out_exr_exists: bool) -> str:
    if result.timed_out:
        return "PBPT_TIMEOUT"
    if result.exit_code == 0 and out_exr_exists:
        return ""

    log = (result.stdout + "\n" + result.stderr).lower()
    if "unsupported" in log and "type" in log:
        return "PBPT_LOAD_UNSUPPORTED_TYPE"
    if "xml error" in log or ("parse" in log and "xml" in log):
        return "PBPT_PARSE_ERROR"
    return "PBPT_RUNTIME_ERROR"


def classify_mi3_error(timed_out: bool, traceback_text: str, exit_code: int, out_exr_exists: bool) -> str:
    if timed_out:
        return "MI3_TIMEOUT"
    if exit_code == 0 and out_exr_exists:
        return ""

    log = traceback_text.lower()
    if "plugin" in log and (
        "not found" in log or "could not instantiate" in log or "unknown plugin" in log
    ):
        return "MI3_PLUGIN_MISSING"
    if "xml" in log and ("parse" in log or "syntax" in log or "mismatched" in log):
        return "MI3_XML_PARSE_ERROR"
    return "MI3_RENDER_ERROR"


def command_result_to_dict(result: CommandResult) -> dict[str, Any]:
    return asdict(result)
