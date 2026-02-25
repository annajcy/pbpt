#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

NAME_MAP = {
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

TAG_MAP = {
    "lookAt": "lookat",
}

TYPE_MAP = {
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

MICROFACET_ROUGHNESS_NAMES = {
    "roughness",
    "roughness_x",
    "roughness_y",
    "roughness_u",
    "roughness_v",
    "alpha",
    "alpha_x",
    "alpha_y",
    "alpha_u",
    "alpha_v",
}


def has_microfacet_roughness_params(bsdf_elem: ET.Element) -> bool:
    for child in list(bsdf_elem):
        if child.attrib.get("name") in MICROFACET_ROUGHNESS_NAMES:
            return True
    return False


def migrate_tree(tree: ET.ElementTree) -> dict[str, int]:
    root = tree.getroot()
    stats = {
        "version_updates": 0,
        "tag_renames": 0,
        "name_renames": 0,
        "type_renames": 0,
    }

    if root.tag == "scene" and root.attrib.get("version") != "3.0.0":
        root.attrib["version"] = "3.0.0"
        stats["version_updates"] += 1

    for elem in root.iter():
        if elem.tag in TAG_MAP:
            elem.tag = TAG_MAP[elem.tag]
            stats["tag_renames"] += 1

        name = elem.attrib.get("name")
        if name in NAME_MAP:
            elem.attrib["name"] = NAME_MAP[name]
            stats["name_renames"] += 1

        etype = elem.attrib.get("type")
        mapped = TYPE_MAP.get((elem.tag, etype))
        if mapped is not None:
            elem.attrib["type"] = mapped
            stats["type_renames"] += 1

        if elem.tag == "bsdf":
            current_type = elem.attrib.get("type")
            if current_type == "dielectric" and has_microfacet_roughness_params(elem):
                elem.attrib["type"] = "roughdielectric"
                stats["type_renames"] += 1
            elif current_type == "conductor" and has_microfacet_roughness_params(elem):
                elem.attrib["type"] = "roughconductor"
                stats["type_renames"] += 1

    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description="Migrate PBPT scenes to Mitsuba3-style XML schema.")
    parser.add_argument("--scene-root", default="asset/scene", help="Root directory containing scene XML files")
    parser.add_argument("--dry-run", action="store_true", help="Only report changes without writing files")
    parser.add_argument("--report", default="output/compare_mi3/migration_report.json", help="Output JSON report")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    scene_root = (repo_root / args.scene_root).resolve()

    if not scene_root.exists():
        raise FileNotFoundError(f"scene root does not exist: {scene_root}")

    files = sorted(scene_root.rglob("*.xml"))
    report = {
        "scene_root": str(scene_root),
        "total_files": len(files),
        "changed_files": 0,
        "parse_failures": [],
        "changes": [],
    }

    for path in files:
        try:
            tree = ET.parse(path)
        except ET.ParseError as exc:
            report["parse_failures"].append({"path": str(path), "error": str(exc)})
            continue

        before = tree.getroot().attrib.get("version", "")
        stats = migrate_tree(tree)
        changed = any(v > 0 for v in stats.values())

        if changed:
            report["changed_files"] += 1
            report["changes"].append({
                "path": str(path),
                "before_version": before,
                **stats,
            })
            if not args.dry_run:
                ET.indent(tree, space="    ")
                tree.write(path, encoding="utf-8", xml_declaration=True)

    report_path = (repo_root / args.report).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True, indent=2))

    return 0 if not report["parse_failures"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
