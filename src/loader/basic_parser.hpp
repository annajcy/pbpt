#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <charconv>
#include <pugixml.hpp>

#include "math/vector.hpp"
#include "math/point.hpp"

namespace pbpt::loader {

// String splitting helper
inline std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

// Basic type parsers
template <typename T>
T parse_value(const std::string& str);

template <>
inline float parse_value<float>(const std::string& str) {
    return std::stof(str);
}

template <>
inline double parse_value<double>(const std::string& str) {
    return std::stod(str);
}

template <>
inline int parse_value<int>(const std::string& str) {
    return std::stoi(str);
}

template <>
inline bool parse_value<bool>(const std::string& str) {
    return str == "true";
}

template <>
inline std::string parse_value<std::string>(const std::string& str) {
    return str;
}

// Math type parsers
template <typename T, int N>
math::Vector<T, N> parse_vector(const std::string& str) {
    auto tokens = split(str, ',');
    math::Vector<T, N> v;
    for (int i = 0; i < N && i < tokens.size(); ++i) {
        v[i] = parse_value<T>(tokens[i]);
    }
    return v;
}

template <typename T, int N>
math::Point<T, N> parse_point(const std::string& str) {
    auto tokens = split(str, ',');
    math::Point<T, N> p;
    for (int i = 0; i < N && i < tokens.size(); ++i) {
        p[i] = parse_value<T>(tokens[i]);
    }
    return p;
}

// Attribute helpers
template <typename T>
T get_attr(const pugi::xml_node& node, const std::string& name, T default_val = T{}) {
    auto attr = node.attribute(name.c_str());
    if (attr) {
        return parse_value<T>(attr.value());
    }
    // Search in child nodes for property-like definitions if needed?
    // Based on cbox.xml: <integer name="width" value="512"/>
    // This is different from attributes.
    return default_val;
}

// Function to find a typed child property like <integer name="foo" value="bar"/>
template <typename T>
T parse_property(const pugi::xml_node& node, const std::string& name, T default_val = T{}) {
    // Determine tag name based on type
    std::string tag_type;
    if constexpr (std::is_same_v<T, int>) tag_type = "integer";
    else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) tag_type = "float";
    else if constexpr (std::is_same_v<T, bool>) tag_type = "boolean";
    else if constexpr (std::is_same_v<T, std::string>) tag_type = "string";
    
    for (auto child : node.children(tag_type.c_str())) {
        if (std::string(child.attribute("name").value()) == name) {
            return parse_value<T>(child.attribute("value").value());
        }
    }
    return default_val;
}

}
