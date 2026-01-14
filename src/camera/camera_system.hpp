#pragma once

#include "camera/render_transform.hpp"
namespace pbpt::camera {

template<typename T, typename CameraType, typename FilmType, typename PixelFilterType>
class CameraSystem {
private:
    CameraType m_camera;
    FilmType m_film;
    PixelFilterType m_pixel_filter;
    RenderTransform<T> m_render_transform;

public:
    CameraSystem() = default;
    CameraSystem(
        const CameraType& camera,
        const FilmType& film,
        const PixelFilterType& pixel_filter,
        const RenderTransform<T>& render_transform
    ) : m_camera(camera), m_film(film), m_pixel_filter(pixel_filter), m_render_transform(render_transform) {}

    const CameraType& camera() const { return m_camera; }
    const FilmType& film() const { return m_film; }
    const PixelFilterType& pixel_filter() const { return m_pixel_filter; }
    const RenderTransform<T>& render_transform() const { return m_render_transform; }
    
    CameraType& camera() { return m_camera; }
    FilmType& film() { return m_film; }
    PixelFilterType& pixel_filter() { return m_pixel_filter; }
    RenderTransform<T>& render_transform() { return m_render_transform; }


    void set_render_transform(const RenderTransform<T>& transform) {
        m_render_transform = transform;
    }
};

};