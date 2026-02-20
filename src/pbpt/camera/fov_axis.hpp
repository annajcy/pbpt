#pragma once
#include <string>

namespace pbpt::camera {

enum class FovAxis { X, Y, Smaller, Larger };

inline std::string fov_axis_to_string(FovAxis axis) {
    switch (axis) {
        case FovAxis::X:
            return "x";
        case FovAxis::Y:
            return "y";
        case FovAxis::Smaller:
            return "smaller";
        case FovAxis::Larger:
            return "larger";
    }
    return "smaller";
}

inline FovAxis fov_axis_from_string(const std::string& s) {
    if (s == "x")
        return FovAxis::X;
    if (s == "y")
        return FovAxis::Y;
    if (s == "smaller")
        return FovAxis::Smaller;
    if (s == "larger")
        return FovAxis::Larger;
    return FovAxis::Smaller;  // safe default
}

}  // namespace pbpt::camera
