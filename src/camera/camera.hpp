#pragma once

#include "geometry/ray.hpp"

namespace pbpt::camera {

template<typename T>
struct CameraSample{
    math::Point<T, 2> p_film{};
    math::Point<T, 2> p_lens{};

    static CameraSample<T> create_pinhole_sample(const math::Point<T, 2>& p_film) {
        return CameraSample<T>{p_film, math::Point<T, 2>(0, 0)};
    }

    static CameraSample<T> create_thinlens_sample(const math::Point<T, 2>& p_film, const math::Point<T, 2>& p_lens) {
        return CameraSample<T>{p_film, p_lens};
    }
};

template<typename T>
class Camera {
public:
    Camera() = default;
    virtual ~Camera() = default;
    virtual geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) const = 0;
    virtual geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) const = 0;
};

} // namespace pbpt::camera
