#pragma once

#include <optional>

#include "camera_transform.hpp"
#include "film.hpp"

namespace pbpt::camera {

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

    template<template<typename> class CameraRayType, template <typename> class CameraSampleType>
    std::optional<CameraRayType<T>> generate_ray(const CameraSampleType<T>& sample) const {
        return as_derived().template generate_ray_impl<CameraRayType, CameraSampleType>(sample);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

};

};