# PBPT vs Mitsuba3 Compare Pipeline (Phase 1)

This directory contains the Phase 1 comparable pipeline for PBPT scene validation against Mitsuba3.
Scenes are consumed directly from repository XML files (Mitsuba3-style), with no runtime normalize step.

## Files

- `in_scope_scenes.txt`: 22 scenes used in Phase 1 scoring.
- `out_of_scope_scenes.txt`: 19 scenes marked as `SKIP_OUT_OF_SCOPE`.
- `render_pbpt.py`: run PBPT `render_scene` executable.
- `render_mitsuba3.py`: run Mitsuba3 render via Python API.
- `compare_images.py`: compute `MSE`, `NRMSE`, `PSNR`, `MaxAbs`, write `diff.exr`.
- `run_compare_pipeline.py`: orchestrate full pipeline and reports.
- `lint_mi3_scene_schema.py`: schema gate for Mitsuba3-only scene syntax.

## Setup

1. Install Python deps in the project venv.

```bash
uv sync
uv add "mitsuba>=3.7"
```

2. Ensure PBPT renderer exists.

```bash
ls build/Release/examples/render_scene
```

## Run

Default run (22 in-scope + 19 out-of-scope records):

```bash
uv run python script/compare/run_compare_pipeline.py
```

Debug run on first 2 in-scope scenes:

```bash
uv run python script/compare/run_compare_pipeline.py --limit 2 --spp-pbpt 1 --spp-mi3 1 --timeout-sec 120
```

Schema lint run:

```bash
uv run python script/compare/lint_mi3_scene_schema.py --report output/compare_mi3/schema_report.json
```

## Outputs

Pipeline outputs to:

`output/compare_mi3/<run_id>/`

Per scene artifacts:

- `pbpt.exr`
- `mi3.exr`
- `diff.exr` (only when comparable)
- `pbpt.log`
- `mi3.log`
- `meta.json`

Run-level artifacts:

- `summary.csv`
- `failures.csv`
- `metrics.csv`
- `summary.md`

## Error Codes

- `PBPT_LOAD_UNSUPPORTED_TYPE`
- `PBPT_PARSE_ERROR`
- `PBPT_RUNTIME_ERROR`
- `PBPT_TIMEOUT`
- `MI3_XML_PARSE_ERROR`
- `MI3_PLUGIN_MISSING`
- `MI3_RENDER_ERROR`
- `MI3_TIMEOUT`
- `COMPARE_IO_ERROR`
- `COMPARE_METRIC_ERROR`
- `COMPARABLE`
- `SKIP_OUT_OF_SCOPE`
