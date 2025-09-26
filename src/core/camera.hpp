#pragma once

#include "geometry/transform.hpp"

namespace pbpt::core {

enum class RenderSpace{
    Camera, 
    CameraWorld,
    World
};

template<typename T>
struct RenderTransform {
    geometry::Transform<T> camera_to_render{};
    geometry::Transform<T> world_to_render{};

    RenderTransform(const geometry::Transform<T>& camera_to_world, RenderSpace space) {
        if (space == RenderSpace::Camera) {
            camera_to_render = geometry::Transform<T>::identity();
            world_to_render = camera_to_world.inversed();
        } else if (space == RenderSpace::CameraWorld) {
            auto [translation, rotation, scale] = fast_decompose_transform(camera_to_world);
            camera_to_render =  rotation * scale;
            world_to_render = translation.inversed();
        } else if (space == RenderSpace::World) {
            camera_to_render = camera_to_world;
            world_to_render = geometry::Transform<T>::identity();
        }
    }
};

template<typename T, typename Derived>
class Camera {
private:
    RenderTransform<T> m_render_transform{};

public:
    RenderTransform<T> render_transform() const {
        return m_render_transform;
    }

};

};