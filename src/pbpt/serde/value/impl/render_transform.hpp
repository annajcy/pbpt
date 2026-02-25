#pragma once

#include <stdexcept>
#include <string>

#include "pbpt/camera/render_transform.hpp"
#include "pbpt/serde/value/impl/transform.hpp"
#include "pbpt/serde/value/value_codec_traits.hpp"

namespace pbpt::serde {

template <typename T>
struct ValueCodec<T, camera::RenderTransform<T>> {
    static geometry::Transform<T> switch_handedness(const geometry::Transform<T>& transform) {
        const auto handedness_flip = geometry::Transform<T>::scale(math::Vector<T, 3>(T(-1), T(1), T(-1)));
        return transform * handedness_flip;
    }

    static camera::RenderTransform<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env,
                                                 bool to_left_handed) {
        bool has_look_at = false;
        bool has_matrix = false;

        geometry::Transform<T> camera_to_world = geometry::Transform<T>::identity();

        for (const auto& child : node.children()) {
            const std::string name = child.name();
            if (name == "lookat") {
                if (has_matrix) {
                    throw std::runtime_error("Sensor transform cannot contain both lookat and matrix");
                }
                const auto origin = detail::parse_point3<T>(child.attribute("origin").value(), "lookat.origin");
                const auto target = detail::parse_point3<T>(child.attribute("target").value(), "lookat.target");
                const auto up = detail::parse_vector3<T>(child.attribute("up").value(), "lookat.up");
                camera_to_world = geometry::Transform<T>::look_at(origin, target, up).inversed();
                has_look_at = true;
            } else if (name == "matrix") {
                if (has_look_at) {
                    throw std::runtime_error("Sensor transform cannot contain both lookat and matrix");
                }
                const char* value = child.attribute("value").value();
                if (!value || value[0] == '\0') {
                    throw std::runtime_error("Sensor transform matrix is missing value attribute");
                }
                camera_to_world = ValueCodec<T, geometry::Transform<T>>::parse_text(value, env);
                has_matrix = true;
            }
        }

        if (to_left_handed) {
            camera_to_world = switch_handedness(camera_to_world);
        }

        if (has_look_at || has_matrix) {
            return camera::RenderTransform<T>::from_camera_to_world(camera_to_world, camera::RenderSpace::World);
        }
        return camera::RenderTransform<T>::from_camera_to_world(geometry::Transform<T>::identity(),
                                                                 camera::RenderSpace::World);
    }

    static camera::RenderTransform<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        return parse_node(node, env, false);
    }

    static void write_node(const camera::RenderTransform<T>& value, pugi::xml_node& node,
                           const ValueCodecWriteEnv<T>& env, bool to_left_handed) {
        auto matrix = node.append_child("matrix");
        matrix.append_attribute("value") = write_text(value, env, to_left_handed).c_str();
    }

    static void write_node(const camera::RenderTransform<T>& value, pugi::xml_node& node,
                           const ValueCodecWriteEnv<T>& env) {
        write_node(value, node, env, false);
    }

    static camera::RenderTransform<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>& env,
                                                 bool to_left_handed) {
        auto camera_to_world = ValueCodec<T, geometry::Transform<T>>::parse_text(text, env);
        if (to_left_handed) {
            camera_to_world = switch_handedness(camera_to_world);
        }
        return camera::RenderTransform<T>::from_camera_to_world(camera_to_world, camera::RenderSpace::World);
    }

    static camera::RenderTransform<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>& env) {
        return parse_text(text, env, false);
    }

    static std::string write_text(const camera::RenderTransform<T>& value, const ValueCodecWriteEnv<T>& env,
                                  bool to_left_handed) {
        auto camera_to_world = value.camera_to_world();
        if (to_left_handed) {
            camera_to_world = switch_handedness(camera_to_world);
        }
        return ValueCodec<T, geometry::Transform<T>>::write_text(camera_to_world, env);
    }

    static std::string write_text(const camera::RenderTransform<T>& value, const ValueCodecWriteEnv<T>& env) {
        return write_text(value, env, false);
    }
};

}  // namespace pbpt::serde
