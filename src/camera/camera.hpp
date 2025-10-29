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

template<typename T, typename Derived>
class Camera {
public:
    Camera() = default;
    ~Camera() = default;

    geometry::Ray<T, 3> generate_ray(const CameraSample<T>& sample) const {
        return as_derived().generate_ray_impl(sample);
    }

    geometry::RayDifferential<T, 3> generate_differential_ray(const CameraSample<T>& sample) const {
        return as_derived().generate_differential_ray_impl(sample);
    }

private:
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

} // namespace pbpt::camera
