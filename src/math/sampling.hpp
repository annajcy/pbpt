#pragma once

#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

template<typename T>
math::Point<T, 2> sample_uniform_disk(const math::Point<T, 2>& uv, T radius = 1.0) {
    T r = std::sqrt(uv.x()) * radius;
    T theta = 2.0 * math::pi_v<T> * uv.y();
    return math::Point<T, 2>(r * std::cos(theta), r * std::sin(theta));
}

template<std::floating_point T>
inline math::Point<T, 3> sample_uniform_hemisphere(const math::Point<T, 2>& uv) {
    T z = uv.x();
    T r = std::sqrt(std::max(T(0), T(1) - z * z));
    T phi = 2.0 * math::pi_v<T> * uv.y();
    return math::Point<T, 3>(r * std::cos(phi), r * std::sin(phi), z);
}

template<typename T>
math::Point<T, 2> sample_uniform_disk_concentric(const math::Point<T, 2>& uv, T radius = 1.0) {
    math::Point<T, 2> p_offset = 2.0 * uv.to_vector() - math::Vector<T, 2>(1, 1);

    if (p_offset.x() == 0 && p_offset.y() == 0) {
        return math::Point<T, 2>(0, 0);
    }

    T theta{}, r{};
    if (std::abs(p_offset.x()) > std::abs(p_offset.y())) {
        r = p_offset.x();
        theta = (math::pi_v<T> / 4.0) * (p_offset.y() / p_offset.x());
    } else {
        r = p_offset.y();
        theta = (math::pi_v<T> / 2.0) - (math::pi_v<T> / 4.0) * (p_offset.x() / p_offset.y());
    }

    return math::Point<T, 2>(radius * r * math::Vector<T, 2>(std::cos(theta), std::sin(theta)));
}

template<typename T>
math::Point<T, 3> sample_uniform_hemisphere_concentric(const math::Point<T, 2>& uv) {
    auto d = math::sample_uniform_disk_concentric(uv);
    auto r2 = d.x() * d.x() + d.y() * d.y();
    auto z = std::sqrt(std::max(T(0), T(1) - r2));
    return math::Point<T, 3>{d.x(), d.y(), z};
}

}
