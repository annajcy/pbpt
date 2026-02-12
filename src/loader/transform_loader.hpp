#pragma once

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pugixml.hpp>

#include "camera/render_transform.hpp"
#include "geometry/transform.hpp"
#include "loader/basic_parser.hpp"

namespace pbpt::loader {

namespace detail {

template <typename T>
std::vector<T> parse_numeric_values(std::string text, const std::string& field_name) {
    for (char& c : text) {
        if (c == ',') {
            c = ' ';
        }
    }

    std::stringstream ss(text);
    std::vector<T> values{};
    T value = T(0);
    while (ss >> value) {
        if (!std::isfinite(static_cast<double>(value))) {
            throw std::runtime_error(field_name + " contains non-finite values.");
        }
        values.emplace_back(value);
    }

    if (ss.fail() && !ss.eof()) {
        throw std::runtime_error(field_name + " contains invalid numeric token.");
    }

    return values;
}

} // namespace detail

template <typename T>
math::Matrix<T, 4, 4> parse_matrix_4x4_value(const std::string& value_str) {
    const auto values = detail::parse_numeric_values<T>(value_str, "matrix value");
    if (values.size() != 16u) {
        throw std::runtime_error("matrix value must contain exactly 16 numbers.");
    }

    math::Matrix<T, 4, 4> matrix{};
    std::size_t idx = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix.at(row, col) = values[idx++];
        }
    }
    return matrix;
}

template <typename T>
geometry::Transform<T> load_transform(const pugi::xml_node& node) {
    geometry::Transform<T> transform = geometry::Transform<T>::identity();

    for (const auto& child : node.children()) {
        const std::string name = child.name();
        if (name == "lookAt") {
            const auto origin = parse_point<T, 3>(child.attribute("origin").value());
            const auto target = parse_point<T, 3>(child.attribute("target").value());
            const auto up = parse_vector<T, 3>(child.attribute("up").value());
            transform = transform * geometry::Transform<T>::look_at(origin, target, up).inversed();
        } else if (name == "matrix") {
            const std::string value = child.attribute("value").value();
            if (value.empty()) {
                throw std::runtime_error("transform matrix is missing value attribute.");
            }
            transform = transform * geometry::Transform<T>(parse_matrix_4x4_value<T>(value));
        } else if (name == "translate") {
            const T x = child.attribute("x").as_float(0);
            const T y = child.attribute("y").as_float(0);
            const T z = child.attribute("z").as_float(0);
            transform = transform * geometry::Transform<T>::translate(math::Vector<T, 3>(x, y, z));
        } else if (name == "scale") {
            const T x = child.attribute("x").as_float(1);
            const T y = child.attribute("y").as_float(1);
            const T z = child.attribute("z").as_float(1);
            transform = transform * geometry::Transform<T>::scale(math::Vector<T, 3>(x, y, z));
        }
    }

    return transform;
}

// Special parser for Camera RenderTransform which supports lookAt and matrix.
template <typename T>
camera::RenderTransform<T> load_render_transform(const pugi::xml_node& node) {
    bool has_look_at = false;
    bool has_matrix = false;

    camera::RenderTransform<T> look_at_transform{};
    geometry::Transform<T> matrix_camera_to_world = geometry::Transform<T>::identity();

    for (const auto& child : node.children()) {
        const std::string name = child.name();
        if (name == "lookAt") {
            if (has_matrix) {
                throw std::runtime_error("Sensor transform cannot contain both lookAt and matrix.");
            }
            const auto origin = parse_point<T, 3>(child.attribute("origin").value());
            const auto target = parse_point<T, 3>(child.attribute("target").value());
            const auto up = parse_vector<T, 3>(child.attribute("up").value());
            look_at_transform =
                camera::RenderTransform<T>::look_at(origin, target, up, camera::RenderSpace::World);
            has_look_at = true;
        } else if (name == "matrix") {
            if (has_look_at) {
                throw std::runtime_error("Sensor transform cannot contain both lookAt and matrix.");
            }
            const std::string value = child.attribute("value").value();
            if (value.empty()) {
                throw std::runtime_error("Sensor transform matrix is missing value attribute.");
            }
            matrix_camera_to_world = geometry::Transform<T>(parse_matrix_4x4_value<T>(value));
            has_matrix = true;
        }
    }

    if (has_look_at) {
        return look_at_transform;
    }
    if (has_matrix) {
        return camera::RenderTransform<T>::from_camera_to_world(
            matrix_camera_to_world,
            camera::RenderSpace::World
        );
    }

    return camera::RenderTransform<T>::from_camera_to_world(
        geometry::Transform<T>::identity(),
        camera::RenderSpace::World
    );
}

} // namespace pbpt::loader
