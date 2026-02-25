#pragma once

#include <string>
#include <string_view>
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <pugixml.hpp>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/serde/value/impl/rgb.hpp"
#include "pbpt/serde/value/impl/wrap_mode.hpp"
#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"
#include "pbpt/texture/plugin/texture/checkerboard_texture.hpp"
#include "pbpt/utils/image_io.hpp"

namespace pbpt::serde {

template <typename T>
struct BitmapTextureSerde {
    static constexpr std::string_view domain = "texture";
    static constexpr std::string_view xml_type = "bitmap";
    using value_type = texture::BitmapTexture<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path};

        auto filename = find_child_value(node, "string", "filename");
        if (!filename) {
            throw std::runtime_error("bitmap texture missing filename: " + std::string(node.attribute("id").value()));
        }

        const auto wrap_mode = find_child_value(node, "string", "wrap_mode");
        const auto wrap_u_legacy = find_child_value(node, "string", "wrap_mode_u");
        const auto wrap_v_legacy = find_child_value(node, "string", "wrap_mode_v");

        const auto wrap_u_str = wrap_mode.value_or(wrap_u_legacy.value_or("repeat"));
        const auto wrap_v_str = wrap_mode.value_or(wrap_v_legacy.value_or("repeat"));
        const auto wrap_u = ValueCodec<T, texture::WrapMode>::parse_text(wrap_u_str, read_env);
        const auto wrap_v = ValueCodec<T, texture::WrapMode>::parse_text(wrap_v_str, read_env);

        return texture::BitmapTexture<T>(ctx.resolve_path(*filename), wrap_u, wrap_v);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& tex = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.mesh_dir, ctx.texture_dir};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();

        const std::string output_filename = id + ".exr";
        const auto abs_out_path = ctx.texture_dir / output_filename;
        std::filesystem::create_directories(abs_out_path.parent_path());
        const auto& base_image = tex.mipmap().level(0);
        utils::write_image<T>(abs_out_path, base_image);

        auto filename_node = node.append_child("string");
        filename_node.append_attribute("name") = "filename";
        const auto rel_path = ctx.relative_texture_path(id, ".exr");
        filename_node.append_attribute("value") = rel_path.c_str();

        const auto wrap_u_text = ValueCodec<T, texture::WrapMode>::write_text(tex.wrap_u(), write_env);
        const auto wrap_v_text = ValueCodec<T, texture::WrapMode>::write_text(tex.wrap_v(), write_env);
        if (wrap_u_text != wrap_v_text) {
            throw std::runtime_error("bitmap texture requires identical wrap_u and wrap_v for MI3 wrap_mode output");
        }

        auto wrap_mode_node = node.append_child("string");
        wrap_mode_node.append_attribute("name") = "wrap_mode";
        wrap_mode_node.append_attribute("value") = wrap_u_text.c_str();
    }
};

template <typename T>
struct CheckerboardTextureSerde {
    static constexpr std::string_view domain = "texture";
    static constexpr std::string_view xml_type = "checkerboard";
    using value_type = texture::CheckerboardTexture<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path};

        const auto color0_text = find_child_value(node, "rgb", "color0").value_or("0");
        const auto color1_text = find_child_value(node, "rgb", "color1").value_or("1");
        const auto uscale = parse_child_value<T, T>(node, "float", "uscale", read_env).value_or(T(1));
        const auto vscale = parse_child_value<T, T>(node, "float", "vscale", read_env).value_or(T(1));
        const auto uoffset = parse_child_value<T, T>(node, "float", "uoffset", read_env).value_or(T(0));
        const auto voffset = parse_child_value<T, T>(node, "float", "voffset", read_env).value_or(T(0));

        T uscale_final = uscale;
        T vscale_final = vscale;
        T uoffset_final = uoffset;
        T voffset_final = voffset;

        pugi::xml_node to_uv;
        for (const auto& child : node.children("transform")) {
            if (std::string(child.attribute("name").value()) == "to_uv") {
                to_uv = child;
                break;
            }
        }

        if (to_uv) {
            for (const auto& op : to_uv.children()) {
                const std::string op_name = op.name();
                if (op_name == "scale") {
                    uscale_final *= op.attribute("x").as_float(1.f);
                    vscale_final *= op.attribute("y").as_float(1.f);
                } else if (op_name == "translate") {
                    uoffset_final += op.attribute("x").as_float(0.f);
                    voffset_final += op.attribute("y").as_float(0.f);
                }
            }
        }

        return texture::CheckerboardTexture<T>(
            ValueCodec<T, radiometry::RGB<T>>::parse_text(color0_text, read_env),
            ValueCodec<T, radiometry::RGB<T>>::parse_text(color1_text, read_env), uscale_final, vscale_final,
            uoffset_final, voffset_final);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& tex = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.mesh_dir, ctx.texture_dir};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();

        auto color0 = node.append_child("rgb");
        color0.append_attribute("name") = "color0";
        const auto color0_text = ValueCodec<T, radiometry::RGB<T>>::write_text(tex.color0(), write_env);
        color0.append_attribute("value") = color0_text.c_str();

        auto color1 = node.append_child("rgb");
        color1.append_attribute("name") = "color1";
        const auto color1_text = ValueCodec<T, radiometry::RGB<T>>::write_text(tex.color1(), write_env);
        color1.append_attribute("value") = color1_text.c_str();

        const bool has_scale = std::abs(tex.uscale() - T(1)) > T(1e-6) || std::abs(tex.vscale() - T(1)) > T(1e-6);
        const bool has_offset = std::abs(tex.uoffset()) > T(1e-6) || std::abs(tex.voffset()) > T(1e-6);
        if (has_scale || has_offset) {
            auto to_uv = node.append_child("transform");
            to_uv.append_attribute("name") = "to_uv";
            if (has_scale) {
                auto scale = to_uv.append_child("scale");
                scale.append_attribute("x") = tex.uscale();
                scale.append_attribute("y") = tex.vscale();
            }
            if (has_offset) {
                auto translate = to_uv.append_child("translate");
                translate.append_attribute("x") = tex.uoffset();
                translate.append_attribute("y") = tex.voffset();
            }
        }
    }
};

static_assert(TextureSerdeConcept<float, BitmapTextureSerde<float>>);
static_assert(TextureSerdeConcept<float, CheckerboardTextureSerde<float>>);

}  // namespace pbpt::serde
