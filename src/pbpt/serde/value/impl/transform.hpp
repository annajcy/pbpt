#pragma once

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "pbpt/geometry/transform.hpp"
#include "pbpt/serde/value/value_codec_traits.hpp"

namespace pbpt::serde {

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
            throw std::runtime_error(field_name + " contains non-finite values");
        }
        values.emplace_back(value);
    }

    if (ss.fail() && !ss.eof()) {
        throw std::runtime_error(field_name + " contains invalid numeric token");
    }

    return values;
}

template <typename T>
math::Vector<T, 3> parse_vector3(std::string_view text, const std::string& field_name) {
    const auto values = parse_numeric_values<T>(std::string(text), field_name);
    if (values.size() != 3u) {
        throw std::runtime_error(field_name + " must contain exactly 3 numbers");
    }
    return math::Vector<T, 3>(values[0], values[1], values[2]);
}

template <typename T>
math::Point<T, 3> parse_point3(std::string_view text, const std::string& field_name) {
    const auto v = parse_vector3<T>(text, field_name);
    return math::Point<T, 3>(v.x(), v.y(), v.z());
}

template <typename T>
math::Matrix<T, 4, 4> parse_matrix_4x4_value(std::string_view value_str) {
    const auto values = parse_numeric_values<T>(std::string(value_str), "matrix value");
    if (values.size() != 16u) {
        throw std::runtime_error("matrix value must contain exactly 16 numbers");
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

}  // namespace detail

template <typename T>
struct ValueCodec<T, geometry::Transform<T>> {
    static geometry::Transform<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        geometry::Transform<T> transform = geometry::Transform<T>::identity();

        for (const auto& child : node.children()) {
            const std::string name = child.name();
            if (name == "lookat") {
                const auto origin = detail::parse_point3<T>(child.attribute("origin").value(), "lookat.origin");
                const auto target = detail::parse_point3<T>(child.attribute("target").value(), "lookat.target");
                const auto up = detail::parse_vector3<T>(child.attribute("up").value(), "lookat.up");
                transform = transform * geometry::Transform<T>::look_at(origin, target, up).inversed();
            } else if (name == "matrix") {
                const char* value = child.attribute("value").value();
                if (!value || value[0] == '\0') {
                    throw std::runtime_error("transform matrix is missing value attribute");
                }
                transform = transform * geometry::Transform<T>(detail::parse_matrix_4x4_value<T>(value));
            } else if (name == "translate") {
                const T x = child.attribute("x") ? parse_scalar(child.attribute("x").value(), env) : T(0);
                const T y = child.attribute("y") ? parse_scalar(child.attribute("y").value(), env) : T(0);
                const T z = child.attribute("z") ? parse_scalar(child.attribute("z").value(), env) : T(0);
                transform = transform * geometry::Transform<T>::translate(math::Vector<T, 3>(x, y, z));
            } else if (name == "scale") {
                const T x = child.attribute("x") ? parse_scalar(child.attribute("x").value(), env) : T(1);
                const T y = child.attribute("y") ? parse_scalar(child.attribute("y").value(), env) : T(1);
                const T z = child.attribute("z") ? parse_scalar(child.attribute("z").value(), env) : T(1);
                transform = transform * geometry::Transform<T>::scale(math::Vector<T, 3>(x, y, z));
            }
        }

        return transform;
    }

    static void write_node(const geometry::Transform<T>& value, pugi::xml_node& node,
                           const ValueCodecWriteEnv<T>& env) {
        auto matrix = node.append_child("matrix");
        matrix.append_attribute("value") = write_text(value, env).c_str();
    }

    static geometry::Transform<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>&) {
        return geometry::Transform<T>(detail::parse_matrix_4x4_value<T>(text));
    }

    static std::string write_text(const geometry::Transform<T>& value, const ValueCodecWriteEnv<T>&) {
        std::ostringstream oss;
        oss << std::setprecision(std::numeric_limits<T>::max_digits10);
        const auto& matrix = value.matrix();
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                if (row != 0 || col != 0) {
                    oss << ", ";
                }
                oss << matrix.at(row, col);
            }
        }
        return oss.str();
    }

private:
    static T parse_scalar(const std::string& text, const ValueCodecReadEnv<T>& env) {
        return ValueCodec<T, T>::parse_text(text, env);
    }
};

}  // namespace pbpt::serde
