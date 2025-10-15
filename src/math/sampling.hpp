#pragma once

#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

template<typename T>
math::Point<T, 2> sample_uniform_disk_polar(const math::Point<T, 2>& p_lens, T lens_radius = 1.0) {
    T r = std::sqrt(p_lens.x()) * lens_radius;
    T theta = 2.0 * math::pi_v<T> * p_lens.y();
    return math::Point<T, 2>(r * std::cos(theta), r * std::sin(theta));
}

template<typename T>
math::Point<T, 2> sample_uniform_disk_concentric(const math::Point<T, 2>& p_lens, T lens_radius = 1.0) {
    math::Point<T, 2> p_offset = 2.0 * p_lens.to_vector() - math::Vector<T, 2>(1, 1);

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

    return math::Point<T, 2>(lens_radius * r * math::Vector<T, 2>(std::cos(theta), std::sin(theta)));
}

};