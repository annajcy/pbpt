#pragma once

#include <stdexcept>
#include <string>

#include "pbpt/camera/render_transform.hpp"
#include "pbpt/serde/value/impl/transform.hpp"
#include "pbpt/serde/value/value_codec_traits.hpp"

namespace pbpt::serde {

template <typename T>
struct ValueCodec<T, camera::RenderTransform<T>> {
    static camera::RenderTransform<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        bool has_look_at = false;
        bool has_matrix = false;

        camera::RenderTransform<T> look_at_transform{};
        geometry::Transform<T> matrix_camera_to_world = geometry::Transform<T>::identity();

        for (const auto& child : node.children()) {
            const std::string name = child.name();
            if (name == "lookat") {
                if (has_matrix) {
                    throw std::runtime_error("Sensor transform cannot contain both lookat and matrix");
                }
                const auto origin = detail::parse_point3<T>(child.attribute("origin").value(), "lookat.origin");
                const auto target = detail::parse_point3<T>(child.attribute("target").value(), "lookat.target");
                const auto up = detail::parse_vector3<T>(child.attribute("up").value(), "lookat.up");
                look_at_transform = camera::RenderTransform<T>::look_at(origin, target, up, camera::RenderSpace::World);
                has_look_at = true;
            } else if (name == "matrix") {
                if (has_look_at) {
                    throw std::runtime_error("Sensor transform cannot contain both lookat and matrix");
                }
                const char* value = child.attribute("value").value();
                if (!value || value[0] == '\0') {
                    throw std::runtime_error("Sensor transform matrix is missing value attribute");
                }
                matrix_camera_to_world = ValueCodec<T, geometry::Transform<T>>::parse_text(value, env);
                has_matrix = true;
            }
        }

        if (has_look_at) {
            return look_at_transform;
        }
        if (has_matrix) {
            return camera::RenderTransform<T>::from_camera_to_world(matrix_camera_to_world, camera::RenderSpace::World);
        }
        return camera::RenderTransform<T>::from_camera_to_world(geometry::Transform<T>::identity(),
                                                                camera::RenderSpace::World);
    }

    static void write_node(const camera::RenderTransform<T>& value, pugi::xml_node& node,
                           const ValueCodecWriteEnv<T>& env) {
        auto matrix = node.append_child("matrix");
        matrix.append_attribute("value") = write_text(value, env).c_str();
    }

    static camera::RenderTransform<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>& env) {
        const auto camera_to_world = ValueCodec<T, geometry::Transform<T>>::parse_text(text, env);
        return camera::RenderTransform<T>::from_camera_to_world(camera_to_world, camera::RenderSpace::World);
    }

    static std::string write_text(const camera::RenderTransform<T>& value, const ValueCodecWriteEnv<T>& env) {
        return ValueCodec<T, geometry::Transform<T>>::write_text(value.camera_to_world(), env);
    }
};

}  // namespace pbpt::serde
