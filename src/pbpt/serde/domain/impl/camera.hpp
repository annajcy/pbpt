#pragma once

#include <string_view>
#include <pugixml.hpp>
#include <stdexcept>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/serde/value/impl/transform.hpp"
#include "pbpt/serde/value/impl/render_transform.hpp"
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"

namespace pbpt::serde {

template <typename T>
struct PerspectiveCameraSerde {
    static constexpr std::string_view domain = "camera";
    static constexpr std::string_view xml_type = "perspective";
    using value_type = camera::ThinLensPerspectiveCamera<T>;
    using load_result = void;
    using write_target = ValueWriteTarget<value_type>;

    static load_result load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        auto& scene = ctx.result.scene;
        const ValueCodecReadEnv<T> read_env{scene.resources, ctx.base_path};

        const float fov = parse_child_value<T, float>(node, "float", "fov", read_env).value_or(0.f);
        const auto axis_str =
            parse_child_value<T, std::string>(node, "string", "fov_axis", read_env).value_or("smaller");
        const float focus_d = parse_child_value<T, float>(node, "float", "focus_distance", read_env).value_or(0.f);
        const float near_clip = -parse_child_value<T, float>(node, "float", "near_clip", read_env).value_or(0.1f);
        const float far_clip = -parse_child_value<T, float>(node, "float", "far_clip", read_env).value_or(10000.0f);

        int width = 512, height = 512;
        if (auto film_node = node.child("film")) {
            width = parse_child_value<T, int>(film_node, "integer", "width", read_env).value_or(width);
            height = parse_child_value<T, int>(film_node, "integer", "height", read_env).value_or(height);
        }

        auto pixel_sensor =
            camera::PixelSensor<T, radiometry::constant::CIED65SpectrumType<T>,
                                radiometry::constant::CIED65SpectrumType<T>, radiometry::constant::XYZSpectrumType<T>>(
                radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::CIE_D65_ilum<T>,
                radiometry::constant::sRGB<T>,
                radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                    radiometry::constant::CIE_X<T>, radiometry::constant::CIE_Y<T>, radiometry::constant::CIE_Z<T>),
                T{1.0});

        auto hdr_film = camera::HDRFilm<T, decltype(pixel_sensor)>(math::Vector<int, 2>(width, height), pixel_sensor);

        scene.camera = camera::ThinLensPerspectiveCamera<T>(camera::AnyFilm<T>(std::move(hdr_film)), T(fov),
                                                            camera::fov_axis_from_string(axis_str), T(near_clip),
                                                            T(far_clip), T(focus_d));
        scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5));
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& cam = target.value;
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.mesh_dir, ctx.texture_dir};

        node.append_attribute("type") = xml_type.data();

        auto fov_node = node.append_child("float");
        fov_node.append_attribute("name") = "fov";
        fov_node.append_attribute("value") = cam.fov_degrees();

        const auto fov_axis_str = camera::fov_axis_to_string(cam.fov_axis());
        auto fov_axis_node = node.append_child("string");
        fov_axis_node.append_attribute("name") = "fov_axis";
        fov_axis_node.append_attribute("value") = fov_axis_str.c_str();

        auto near_node = node.append_child("float");
        near_node.append_attribute("name") = "near_clip";
        near_node.append_attribute("value") = -cam.near_clip();

        auto far_node = node.append_child("float");
        far_node.append_attribute("name") = "far_clip";
        far_node.append_attribute("value") = -cam.far_clip();

        auto fd_node = node.append_child("float");
        fd_node.append_attribute("name") = "focus_distance";
        fd_node.append_attribute("value") = cam.focal_distance();

        auto transform = node.append_child("transform");
        transform.append_attribute("name") = "to_world";
        const auto mat_text =
            ValueCodec<T, geometry::Transform<T>>::write_text(ctx.result.scene.render_transform.camera_to_world(), write_env);
        transform.append_child("matrix").append_attribute("value") = mat_text.c_str();

        const auto res = cam.film_resolution();
        auto film = node.append_child("film");
        film.append_attribute("type") = "hdrfilm";
        auto w = film.append_child("integer");
        w.append_attribute("name") = "width";
        w.append_attribute("value") = res.x();
        auto h = film.append_child("integer");
        h.append_attribute("name") = "height";
        h.append_attribute("value") = res.y();
    }
};

static_assert(CameraSerdeConcept<float, PerspectiveCameraSerde<float>>);

}  // namespace pbpt::serde
