# Plugin Development / Serde Architecture

PBPT scene I/O is split into two layers:

1. **Domain Serde Traits** (`pbpt::serde`): texture/material/shape/camera/integrator/sampler node dispatch.
2. **Value Codec Traits** (`pbpt::serde`): reusable value-level parse/write for XML/text payloads.

## Add a Domain Plugin Serde

1. Implement a domain serde trait, for example texture:

```cpp
#include "pbpt/serde/domain/trait_contracts.hpp"

template <typename T>
struct MyTextureSerde {
    static constexpr std::string_view domain = "texture";
    static constexpr std::string_view xml_type = "my_texture";
    using value_type = texture::MyTexture<T>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx);
    static void write(const value_type& value, const std::string& id, pugi::xml_node& node, WriteContext<T>& ctx);
};

static_assert(pbpt::serde::TextureSerdeConcept<float, MyTextureSerde<float>>);
```

2. Register it in `src/pbpt/serde/domain/typelist.hpp`.

```cpp
template <typename T>
using TextureSerdeList = std::tuple<
    BitmapTextureSerde<T>,
    CheckerboardTextureSerde<T>,
    MyTextureSerde<T>
>;
```

`typelist.hpp` enforces domain `xml_type` uniqueness via `has_unique_xml_types<...>()` + `static_assert`.

## Use Value Codec Traits

Value codecs are type-based and live under `pbpt::serde`.

```cpp
#include "pbpt/serde/value/value_codec_traits.hpp"

// Example call-site
const pbpt::serde::ValueCodecReadEnv<float> read_env{resources, base_dir};
auto rgb = pbpt::serde::ValueCodec<float, pbpt::radiometry::RGB<float>>::parse_text("0.8 0.7 0.6", read_env);
```

When writing serdes, prefer:

- `pbpt::serde::parse_child_value<T, ValueT>(...)`
- `pbpt::serde::write_child_value<T, ValueT>(...)`
- `pbpt::serde::parse_named_ref(...)`

from `pbpt/serde/value/value_codec_dispatch.hpp`.

## Architecture Rules

1. Domain node orchestration stays in `pbpt::serde::scene_loader` / `pbpt::serde::scene_writer` and domain serdes.
2. Value parsing/serialization must go through `pbpt::serde::ValueCodec<T, ValueT>`.
3. Concrete value codec implementations belong to their domain modules:

- `geometry/codec/*`
- `camera/codec/*`
- `radiometry/codec/*`
- `texture/codec/*`
- `material/codec/*`

4. Do not add new value-level helpers outside `pbpt::serde`.

This keeps plugin dispatch and value conversion independent and composable.
