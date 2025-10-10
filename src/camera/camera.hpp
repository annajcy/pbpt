#pragma once

#include <optional>

#include "geometry/ray.hpp"
#include "math/point.hpp"

#include "camera_transform.hpp"

namespace pbpt::camera {

template<typename T>
struct CameraSample {
    math::Point<T, 2> p_film;
};

template<typename T>
struct CameraRay {
    geometry::Ray<T, 3> ray;   
};

template<typename T, typename Derived>
class Camera {
private:
    RenderTransform<T> m_render_transform{};

public:
    Camera(const RenderTransform<T>& render_transform) : 
    m_render_transform(render_transform) {}

    RenderTransform<T> render_transform() const {
        return m_render_transform;
    }
  
    std::optional<CameraRay<T>> generate_ray(const CameraSample<T>& sample) const {
        return as_derived().generate_ray_impl(sample);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

};

};