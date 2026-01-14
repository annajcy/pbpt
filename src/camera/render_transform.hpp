/**
 * @file
 * @brief Helpers for choosing the coordinate space used for rendering.
 */
#pragma once

#include "geometry/transform.hpp"

namespace pbpt::camera {

/**
 * @brief Coordinate space where rendering computations are performed.
 *
 * - Camera: rendering is done in camera space; camera_to_render is identity.
 * - CameraWorld: rendering space separates local camera rotation/scale
 *   from world translation (useful for improving numerical stability).
 * - World: rendering is done directly in world space.
 */
enum class RenderSpace{
    Camera, 
    CameraWorld,
    World
};

/**
 * @brief Manages transforms between camera, render, and world spaces.
 *
 * Given a camera-to-world transform and a chosen RenderSpace, this class
 * decomposes or combines transforms so that all three spaces are easily
 * accessible. This is useful when you want to do shading in a space that
 * avoids large world-space translations or when you want to keep camera
 * space and world space separate.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template<typename T>
class RenderTransform {  
private:
    /// Transform from camera space to render space.
    geometry::Transform<T> m_camera_to_render{};
    /// Transform from render space to world space.
    geometry::Transform<T> m_render_to_world{};
    /// Original camera-to-world transform.
    geometry::Transform<T> m_camera_to_world{};
    /// Currently selected render space.
    RenderSpace m_render_space{RenderSpace::CameraWorld};

public:
    RenderTransform() = default;
    
    /// Create from a camera-to-world transform.
    static RenderTransform<T> from_camera_to_world(
        const geometry::Transform<T>& camera_to_world, 
        RenderSpace space = RenderSpace::CameraWorld
    ) {
        return RenderTransform<T>(camera_to_world, space);
    }

    /// Create from a world-to-camera transform.
    static RenderTransform<T> from_world_to_camera(
        const geometry::Transform<T>& world_to_camera, 
        RenderSpace space = RenderSpace::CameraWorld
    ) {
        return RenderTransform<T>(world_to_camera.inversed(), space);
    }

    /**
     * @brief Build a render transform from a look-at specification.
     *
     * The resulting transform uses the provided eye/target/up to create
     * a world-to-camera matrix and then derives the render-space
     * transforms from it.
     */
    static RenderTransform<T> look_at(
        const math::Point<T, 3>& eye,
        const math::Point<T, 3>& target,
        const math::Vector<T, 3>& up,
        RenderSpace space = RenderSpace::CameraWorld
    ) {
        auto world_to_camera = geometry::Transform<T>::look_at(eye, target, up);
        return from_world_to_camera(world_to_camera, space);
    }

public:
    /**
     * @brief Construct render transforms from a camera-to-world transform.
     *
     * Depending on @p space:
     * - Camera: render space equals camera space.
     * - CameraWorld: render space contains rotation and scale, while
     *   world space contains the translation component.
     * - World: render space equals world space.
     */
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

    /// Get the current render space mode.
    RenderSpace render_space() const {
        return m_render_space;
    }

    /// Transform from camera space to render space.
    geometry::Transform<T> camera_to_render() const {
        return m_camera_to_render;
    }

    /// Transform from render space back to camera space.
    geometry::Transform<T> render_to_camera() const {
        return m_camera_to_render.inversed();
    }

    /// Transform from render space to world space.
    geometry::Transform<T> render_to_world() const {
        return m_render_to_world;
    }

    /// Transform from world space to render space.
    geometry::Transform<T> world_to_render() const {
        return m_render_to_world.inversed();
    }

    /// Original transform from camera space to world space.
    geometry::Transform<T> camera_to_world() const {
        return m_camera_to_world;
    }

    /// Transform from world space back to camera space.
    geometry::Transform<T> world_to_camera() const {
        return m_camera_to_world.inversed();
    }

    /**
     * @brief Convert an object-to-world transform into object-to-render.
     *
     * Useful for building shapes in world space while shading in the chosen
     * render space.
     */
    geometry::Transform<T> object_to_render_from_object_to_world(
        const geometry::Transform<T>& object_to_world
    ) const {
        return world_to_render() * object_to_world;
    }

    /**
     * @brief Change the render space in-place.
     *
     * This recomputes internal transforms while preserving the original
     * camera-to-world transform.
     */
    RenderTransform<T>& change_render_space(RenderSpace space) {
        *this = to_render_space(space);
        return *this;
    }

    /**
     * @brief Create a new RenderTransform in a different render space.
     *
     * @param space Target render space.
     * @return New RenderTransform using the same camera-to-world transform.
     */
    RenderTransform<T> to_render_space(RenderSpace space) const {
        return RenderTransform<T>(m_camera_to_world, space);
    }
};

}; // namespace pbpt::camera
