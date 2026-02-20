#pragma once

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <pugixml.hpp>

#include "pbpt/loader/writer_context.hpp"
#include "pbpt/radiometry/color.hpp"
#include "pbpt/utils/image_io.hpp"

namespace pbpt::loader {

template <typename T>
inline std::string serialize_rgb_triplet(const radiometry::RGB<T>& rgb) {
    std::ostringstream oss;
    oss << std::setprecision(9) << rgb.r() << ' ' << rgb.g() << ' ' << rgb.b();
    return oss.str();
}

inline std::string serialize_wrap_mode(texture::WrapMode mode) {
    switch (mode) {
        case texture::WrapMode::Clamp:
            return "clamp";
        case texture::WrapMode::Repeat:
        default:
            return "repeat";
    }
}

template <typename T>
void write_texture_nodes(pugi::xml_node& root, WriterContext<T>& ctx) {
    for (const auto& [texture_name, texture_id] : ctx.resources.reflectance_texture_library.name_to_id()) {
        const auto& any_texture = ctx.resources.reflectance_texture_library.get(texture_id);

        auto texture_node = root.append_child("texture");
        texture_node.append_attribute("id") = texture_name.c_str();

        std::visit(
            [&](const auto& texture_variant) {
                using TextureT = std::decay_t<decltype(texture_variant)>;
                if constexpr (std::is_same_v<TextureT, texture::BitmapTexture<T>>) {
                    texture_node.append_attribute("type") = "bitmap";

                    // Write base-level image to textures/ directory as EXR (lossless)
                    const std::string output_filename = texture_name + ".exr";
                    const auto abs_out_path = ctx.texture_dir / output_filename;
                    std::filesystem::create_directories(abs_out_path.parent_path());
                    const auto& base_image = texture_variant.mipmap().level(0);
                    utils::write_image<T>(abs_out_path, base_image);

                    // XML node references the relative path
                    auto filename_node = texture_node.append_child("string");
                    filename_node.append_attribute("name") = "filename";
                    const auto rel_path = ctx.relative_texture_path(texture_name, ".exr");
                    filename_node.append_attribute("value") = rel_path.c_str();

                    auto wrap_u = texture_node.append_child("string");
                    wrap_u.append_attribute("name") = "wrapModeU";
                    const auto wrap_u_str = serialize_wrap_mode(texture_variant.wrap_u());
                    wrap_u.append_attribute("value") = wrap_u_str.c_str();

                    auto wrap_v = texture_node.append_child("string");
                    wrap_v.append_attribute("name") = "wrapModeV";
                    const auto wrap_v_str = serialize_wrap_mode(texture_variant.wrap_v());
                    wrap_v.append_attribute("value") = wrap_v_str.c_str();
                } else if constexpr (std::is_same_v<TextureT, texture::CheckerboardTexture<T>>) {
                    texture_node.append_attribute("type") = "checkerboard";

                    auto color0 = texture_node.append_child("rgb");
                    color0.append_attribute("name") = "color0";
                    const auto color0_text = serialize_rgb_triplet(texture_variant.color0());
                    color0.append_attribute("value") = color0_text.c_str();

                    auto color1 = texture_node.append_child("rgb");
                    color1.append_attribute("name") = "color1";
                    const auto color1_text = serialize_rgb_triplet(texture_variant.color1());
                    color1.append_attribute("value") = color1_text.c_str();

                    auto uscale = texture_node.append_child("float");
                    uscale.append_attribute("name") = "uscale";
                    uscale.append_attribute("value") = texture_variant.uscale();

                    auto vscale = texture_node.append_child("float");
                    vscale.append_attribute("name") = "vscale";
                    vscale.append_attribute("value") = texture_variant.vscale();

                    auto uoffset = texture_node.append_child("float");
                    uoffset.append_attribute("name") = "uoffset";
                    uoffset.append_attribute("value") = texture_variant.uoffset();

                    auto voffset = texture_node.append_child("float");
                    voffset.append_attribute("name") = "voffset";
                    voffset.append_attribute("value") = texture_variant.voffset();
                } else {
                    throw std::runtime_error("Unsupported texture type for serialization: " + texture_name);
                }
            },
            any_texture);

        ctx.texture_name_by_id[texture_id] = texture_name;
    }
}

}  // namespace pbpt::loader
