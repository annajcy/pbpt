#pragma once

#include "geometry/transform.hpp"
#include "math/homogeneous.hpp"
#include "math/point.hpp"

namespace pbpt::camera {

template<typename T>
class CameraProjection {
private:
    geometry::Transform<T> m_clip_to_viewport{};
    geometry::Transform<T> m_camera_to_clip{};

    geometry::Transform<T> m_camera_to_viewport{};

public:
    CameraProjection(
        const geometry::Transform<T>& camera_to_clip, 
        const geometry::Transform<T>& clip_to_viewport
    ) : m_clip_to_viewport(clip_to_viewport), m_camera_to_clip(camera_to_clip) {
        m_camera_to_viewport = m_clip_to_viewport * m_camera_to_clip;
    }

    geometry::Transform<T> camera_to_clip() const {
        return m_camera_to_clip;
    }

    geometry::Transform<T> clip_to_viewport() const {
        return m_clip_to_viewport;
    }

    geometry::Transform<T> camera_to_viewport() const {
        return m_camera_to_viewport;
    }

    geometry::Transform<T> viewport_to_clip() const {
        return m_clip_to_viewport.inverse_matrix();
    }

    geometry::Transform<T> clip_to_camera() const {
        return m_camera_to_clip.inverse_matrix();
    }

    geometry::Transform<T> viewport_to_camera() const {
        return m_camera_to_viewport.inverse_matrix();
    }

    math::Point<T, 3> apply_viewport_to_camera(const math::Point<T, 2>& p) const {
        math::Homogeneous<T, 4> hp = math::Homogeneous<T, 4>::from_point(math::Point<T, 3>(p.x(), p.y(), 0.0));
        auto hc = viewport_to_camera() * hp;
        return hc.to_point();
    }

    static CameraProjection<T> orthographic(
        T left, T right, T bottom, T top, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::orthographic(left, right, bottom, top, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }

    static CameraProjection<T> perspective(
        T fov_y_rad, T aspect_xy, T near, T far, 
        T width, T height
    ) {
        geometry::Transform<T> camera_to_clip = geometry::Transform<T>::perspective(fov_y_rad, aspect_xy, near, far);
        geometry::Transform<T> clip_to_viewport = geometry::Transform<T>::viewport(width, height);
        return CameraProjection<T>(camera_to_clip, clip_to_viewport);
    }
};

};