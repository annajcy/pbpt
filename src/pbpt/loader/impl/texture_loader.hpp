#pragma once

#include "pbpt/loader/component_loader.hpp"
#include "pbpt/loader/parser_utils.hpp"
#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"
#include "pbpt/texture/plugin/texture/checkerboard_texture.hpp"

namespace pbpt::loader {

template <typename T>
void register_texture_loaders() {
    using Registry = TextureLoaderRegistry<T>;

    Registry::register_loader("bitmap",
                              [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> texture::AnyTexture<T> {
                                  auto filename = find_child_value(node, "string", "filename");
                                  if (!filename) {
                                      throw std::runtime_error("bitmap texture missing filename: " +
                                                               std::string(node.attribute("id").value()));
                                  }

                                  auto wrap_u_str = find_child_value(node, "string", "wrapModeU").value_or("repeat");
                                  auto wrap_v_str = find_child_value(node, "string", "wrapModeV").value_or("repeat");
                                  auto wrap_u = parse_wrap_mode(wrap_u_str);
                                  auto wrap_v = parse_wrap_mode(wrap_v_str);

                                  // use regular RGB texture and convert to spectrum at evaluation time
                                  return texture::BitmapTexture<T>(ctx.resolve_path(*filename), wrap_u, wrap_v);
                              });

    Registry::register_loader(
        "checkerboard", [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> texture::AnyTexture<T> {
            auto color0 = find_child_value(node, "rgb", "color0").value_or("0");
            auto color1 = find_child_value(node, "rgb", "color1").value_or("1");
            auto uscale = find_float_property<T>(node, "uscale").value_or(T(1));
            auto vscale = find_float_property<T>(node, "vscale").value_or(T(1));
            auto uoffset = find_float_property<T>(node, "uoffset").value_or(T(0));
            auto voffset = find_float_property<T>(node, "voffset").value_or(T(0));

            return texture::CheckerboardTexture<T>(parse_rgb_triplet<T>(color0), parse_rgb_triplet<T>(color1), uscale,
                                                   vscale, uoffset, voffset);
        });
}

}  // namespace pbpt::loader
