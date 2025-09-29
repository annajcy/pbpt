#pragma once

#include <optional>
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/point.hpp"

namespace pbpt::core {

enum class RenderSpace{
    Camera, 
    CameraWorld,
    World
};

template<typename T>
class RenderTransform {  
private:
    geometry::Transform<T> m_camera_to_render{};
    geometry::Transform<T> m_render_to_world{};
    geometry::Transform<T> m_camera_to_world{};
    RenderSpace m_render_space{RenderSpace::CameraWorld};

public:
    RenderTransform(
        const geometry::Transform<T>& camera_to_world, 
        RenderSpace space = RenderSpace::CameraWorld
    ) : m_camera_to_world(camera_to_world), m_render_space(space) {
        if (space == RenderSpace::Camera) {
            m_camera_to_render = geometry::Transform<T>::identity();
            m_render_to_world = camera_to_world;
        } else if (space == RenderSpace::CameraWorld) {
            auto [translation, rotation, scale] = fast_decompose_transform(camera_to_world);
            m_camera_to_render = rotation * scale;
            m_render_to_world = translation;
        } else if (space == RenderSpace::World) {
            m_camera_to_render = camera_to_world;
            m_render_to_world = geometry::Transform<T>::identity();
        }
    }

    RenderSpace render_space() const {
        return m_render_space;
    }

    geometry::Transform<T> camera_to_render() const {
        return m_camera_to_render;
    }

    geometry::Transform<T> render_to_camera() const {
        return m_camera_to_render.inversed();
    }

    geometry::Transform<T> render_to_world() const {
        return m_render_to_world;
    }

    geometry::Transform<T> world_to_render() const {
        return m_render_to_world.inversed();
    }

    geometry::Transform<T> camera_to_world() const {
        return m_camera_to_world;
    }

    geometry::Transform<T> world_to_camera() const {
        return m_camera_to_world.inversed();
    }

    RenderTransform<T>& change_render_space(RenderSpace space) {
        *this = to_render_space(space);
        return *this;
    }

    RenderTransform<T> to_render_space(RenderSpace space) const {
        return RenderTransform<T>(m_camera_to_world, space);
    }
};

template<typename T>
struct CameraSample {
    math::Point<T, 2> p_film;   // 影像平面上的点，范围通常是[0,1]x[0,1]
};

template<typename T>
struct CameraRay {
    geometry::Ray<T, 3> ray;   
};

template<typename T>
struct Film {
    math::Point<T, 2> resolution; // 影像分辨率
    math::Point<T, 2> size;       // 影像尺寸，单位通常是毫米
};

template<typename T, typename Derived>
class Camera {
private:
    RenderTransform<T> m_render_transform{};

public:
    Camera<T, Derived>(const RenderTransform<T>& render_transform) : 
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