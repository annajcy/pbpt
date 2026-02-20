#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/serde/value/value_codec_traits.hpp"

namespace pbpt::serde {

template <typename T>
struct ValueCodec<T, radiometry::RGB<T>> {
    static radiometry::RGB<T> parse_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& env) {
        const char* value = node.attribute("value").value();
        if (!value || value[0] == '\0') {
            throw std::runtime_error("rgb node is missing value attribute");
        }
        return parse_text(std::string_view(value), env);
    }

    static void write_node(const radiometry::RGB<T>& value, pugi::xml_node& node, const ValueCodecWriteEnv<T>& env) {
        const auto text = write_text(value, env);
        node.append_attribute("value") = text.c_str();
    }

    static radiometry::RGB<T> parse_text(std::string_view text, const ValueCodecReadEnv<T>&) {
        std::string normalized(text);
        std::replace(normalized.begin(), normalized.end(), ',', ' ');

        std::istringstream iss(normalized);
        T r = T(0);
        T g = T(0);
        T b = T(0);
        if (!(iss >> r)) {
            throw std::runtime_error("invalid rgb value: " + normalized);
        }
        if (!(iss >> g)) {
            g = r;
        }
        if (!(iss >> b)) {
            b = r;
        }
        if (!std::isfinite(static_cast<double>(r)) || !std::isfinite(static_cast<double>(g)) ||
            !std::isfinite(static_cast<double>(b))) {
            throw std::runtime_error("rgb value contains non-finite component");
        }
        return radiometry::RGB<T>(r, g, b);
    }

    static std::string write_text(const radiometry::RGB<T>& value, const ValueCodecWriteEnv<T>&) {
        std::ostringstream oss;
        oss << std::setprecision(std::numeric_limits<T>::max_digits10) << value.r() << ' ' << value.g() << ' '
            << value.b();
        return oss.str();
    }
};

}  // namespace pbpt::serde
