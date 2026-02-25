#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import xml.etree.ElementTree as ET

LEGACY_NAME_MAP = {
    "toWorld": "to_world",
    "fovAxis": "fov_axis",
    "focusDistance": "focus_distance",
    "nearClip": "near_clip",
    "farClip": "far_clip",
    "sampleCount": "sample_count",
    "maxDepth": "max_depth",
    "emitterSamples": "emitter_samples",
    "bsdfSamples": "bsdf_samples",
    "shapeIndex": "shape_index",
    "maxSmoothAngle": "max_smooth_angle",
    "intIOR": "int_ior",
    "specularReflectance": "specular_reflectance",
    "specularTransmittance": "specular_transmittance",
    "diffuseReflectance": "diffuse_reflectance",
    "alphaU": "alpha_u",
    "alphaV": "alpha_v",
    "roughnessU": "roughness_u",
    "roughnessV": "roughness_v",
    "specularTransmission": "specular_transmission",
    "baseColor": "base_color",
    "specularTint": "specular_tint",
    "sheenTint": "sheen_tint",
    "clearcoatGloss": "clearcoat_gloss",
    "pixelFormat": "pixel_format",
    "sigmaA": "sigma_a",
    "sigmaS": "sigma_s",
    "stepSize": "step_size",
    "wrapModeU": "wrap_mode_u",
    "wrapModeV": "wrap_mode_v",
}

LEGACY_TAG_MAP = {
    "lookAt": "lookat",
}

LEGACY_PLUGIN_MAP = {
    ("sampler", "ldsampler"): "independent",
    ("bsdf", "dielectric_specular"): "dielectric",
    ("bsdf", "dielectric_rough"): "roughdielectric",
    ("bsdf", "conductor_specular"): "conductor",
    ("bsdf", "conductor_rough"): "roughconductor",
    ("bsdf", "disneybsdf"): "principled",
    ("bsdf", "disneydiffuse"): "principled",
    ("bsdf", "disneymetal"): "principled",
    ("bsdf", "disneyglass"): "principled",
    ("bsdf", "disneyclearcoat"): "principled",
    ("bsdf", "disneysheen"): "principled",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def node_path(parents: list[ET.Element], elem: ET.Element) -> str:
    parts: list[str] = []
    for node in [*parents, elem]:
        suffix = ""
        if "id" in node.attrib and node.attrib["id"]:
            suffix = f"[@id='{node.attrib['id']}']"
        elif "name" in node.attrib and node.attrib["name"]:
            suffix = f"[@name='{node.attrib['name']}']"
        parts.append(f"{node.tag}{suffix}")
    return "/" + "/".join(parts)


def walk(parents: list[ET.Element], elem: ET.Element):
    yield parents, elem
    parents2 = [*parents, elem]
    for child in list(elem):
        yield from walk(parents2, child)


def lint_file(path: Path) -> list[dict[str, str]]:
    violations: list[dict[str, str]] = []
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        return [{
            "code": "XML_PARSE_ERROR",
            "message": str(exc),
            "node": "/",
            "expected": "valid XML",
        }]

    root = tree.getroot()
    if root.tag != "scene":
        violations.append({
            "code": "ROOT_TAG_INVALID",
            "message": f"root tag must be <scene>, got <{root.tag}>",
            "node": f"/{root.tag}",
            "expected": "scene",
        })
    version = root.attrib.get("version", "")
    if version != "3.0.0":
        violations.append({
            "code": "SCENE_VERSION_INVALID",
            "message": f"scene version must be 3.0.0, got '{version}'",
            "node": f"/{root.tag}",
            "expected": "3.0.0",
        })

    for parents, elem in walk([], root):
        path_str = node_path(parents, elem)

        if elem.tag in LEGACY_TAG_MAP:
            violations.append({
                "code": "LEGACY_TAG",
                "message": f"legacy tag '{elem.tag}' is not allowed",
                "node": path_str,
                "expected": LEGACY_TAG_MAP[elem.tag],
            })

        name = elem.attrib.get("name")
        if name in LEGACY_NAME_MAP:
            violations.append({
                "code": "LEGACY_NAME",
                "message": f"legacy field name '{name}' is not allowed",
                "node": path_str,
                "expected": LEGACY_NAME_MAP[name],
            })

        etype = elem.attrib.get("type")
        plugin_expected = LEGACY_PLUGIN_MAP.get((elem.tag, etype))
        if plugin_expected is not None:
            violations.append({
                "code": "LEGACY_PLUGIN",
                "message": f"legacy plugin '{etype}' is not allowed for <{elem.tag}>",
                "node": path_str,
                "expected": plugin_expected,
            })

    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description="Lint scene XML files for Mitsuba3-only schema")
    parser.add_argument("--scene-root", default="asset/scene")
    parser.add_argument("--report", default="output/compare_mi3/schema_report.json")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    scene_root = (repo_root / args.scene_root).resolve()
    files = sorted(scene_root.rglob("*.xml"))

    records = []
    violations_total = 0
    for path in files:
        violations = lint_file(path)
        violations_total += len(violations)
        records.append({
            "path": str(path),
            "ok": len(violations) == 0,
            "violations": violations,
        })

    report = {
        "generated_utc": utc_now(),
        "scene_root": str(scene_root),
        "total_files": len(files),
        "ok_files": sum(1 for r in records if r["ok"]),
        "violations_total": violations_total,
        "files": records,
    }

    report_path = (repo_root / args.report).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({
        "total_files": report["total_files"],
        "ok_files": report["ok_files"],
        "violations_total": report["violations_total"],
        "report": str(report_path),
    }, ensure_ascii=True, indent=2))

    return 0 if violations_total == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
