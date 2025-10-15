#pragma once

#include "geometry/transform.hpp"

namespace pbpt::camera {

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
        return m_camera_to_render.inverse_matrix();
    }

    geometry::Transform<T> render_to_world() const {
        return m_render_to_world;
    }

    geometry::Transform<T> world_to_render() const {
        return m_render_to_world.inverse_matrix();
    }

    geometry::Transform<T> camera_to_world() const {
        return m_camera_to_world;
    }

    geometry::Transform<T> world_to_camera() const {
        return m_camera_to_world.inverse_matrix();
    }

    RenderTransform<T>& change_render_space(RenderSpace space) {
        *this = to_render_space(space);
        return *this;
    }

    RenderTransform<T> to_render_space(RenderSpace space) const {
        return RenderTransform<T>(m_camera_to_world, space);
    }
};
};