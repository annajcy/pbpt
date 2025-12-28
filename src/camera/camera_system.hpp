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
    CameraSystem(
        const CameraType& camera,
        const FilmType& film,
        const PixelFilterType& pixel_filter
    ) : m_camera(camera), m_film(film), m_pixel_filter(pixel_filter) {}

    const CameraType& camera() const {
        return m_camera;
    }

    const FilmType& film() const {
        return m_film;
    }

    const PixelFilterType& pixel_filter() const {
        return m_pixel_filter;
    }
};

};