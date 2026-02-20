# Value Codec Architecture (serde)

## Purpose

`pbpt::serde::ValueCodec<T, ValueT>` is the single value-level parse/write contract for scene XML.

- Domain serdes handle node orchestration.
- Value codecs handle `ValueT` conversion from/to XML node/text.

## Core Types

Defined in `src/pbpt/serde/value/value_codec_traits.hpp`:

- `ValueCodecReadEnv<T>`
- `ValueCodecWriteEnv<T>`
- `ValueCodec<T, ValueT>`
- `ValueCodecConcept<T, ValueT>`

Required methods per codec:

- `parse_node(const pugi::xml_node&, const ValueCodecReadEnv<T>&)`
- `write_node(const ValueT&, pugi::xml_node&, const ValueCodecWriteEnv<T>&)`
- `parse_text(std::string_view, const ValueCodecReadEnv<T>&)`
- `write_text(const ValueT&, const ValueCodecWriteEnv<T>&)`

## Shared XML Ops

Defined in:

- `src/pbpt/serde/value/xml_field_ops.hpp`
- `src/pbpt/serde/value/value_codec_dispatch.hpp`

Common helpers:

- `find_child_value(...)`
- `find_child_reference_id(...)`
- `parse_child_value<T, ValueT>(...)`
- `parse_required_child_value<T, ValueT>(...)`
- `write_child_value<T, ValueT>(...)`
- `parse_named_ref(...)`

## Module Placement

Implemented codec files:

- `src/pbpt/geometry/codec/transform_value_codec.hpp`
- `src/pbpt/camera/codec/render_transform_value_codec.hpp`
- `src/pbpt/radiometry/codec/piecewise_spectrum_value_codec.hpp`
- `src/pbpt/radiometry/codec/rgb_value_codec.hpp`
- `src/pbpt/texture/codec/wrap_mode_value_codec.hpp`
- `src/pbpt/material/codec/microfacet_model_value_codec.hpp`

## Removed Legacy APIs

The following value-level helpers are removed and must not be used:

- `src/pbpt/loader/serde/transform_serde.hpp`
- `src/pbpt/loader/serde/spectrum_serde.hpp`
- `src/pbpt/loader/serde/xml_utils.hpp`
- `src/pbpt/texture/wrap_mode_codec.hpp`

## Migration Pattern

From old call style:

```cpp
auto value = parse_piecewise_spectrum_value<T>(text, ctx);
```

To new style:

```cpp
const pbpt::serde::ValueCodecReadEnv<T> read_env{ctx.scene.resources, ctx.base_path};
auto value = pbpt::serde::ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(text, read_env);
```

For XML child values:

```cpp
auto alpha = pbpt::serde::parse_child_value<T, T>(node, "float", "alpha", read_env);
```

## Design Constraints

1. New value codecs must be implemented in the owning module and specialized under `pbpt::serde`.
2. Domain serdes should only orchestrate XML nodes and references.
3. `load_scene<T>()` / `write_scene()` remain public entry points and must stay stable.
4. Domain serde write targets are domain-specific: `IdValueWriteTarget` (texture/material), `ShapeWriteTarget` (shape),
   and `SceneWriteTarget` (camera/integrator/sampler).
