#pragma once

namespace pbpt::camera {

template<typename CameraType, typename FilmType, typename PixelSensorType>
class CameraSystem {
private:
    CameraType m_camera{};
    FilmType m_film{};
    PixelSensorType m_pixel_sensor{};

public:
    CameraSystem(
        const CameraType& camera,
        const FilmType& film,
        const PixelSensorType& pixel_sensor
    ) : m_camera(camera),
        m_film(film),
        m_pixel_sensor(pixel_sensor)
    {}

    const CameraType& camera() const { return m_camera; }
    const FilmType& film() const { return m_film; }
    const PixelSensorType& pixel_sensor() const { return m_pixel_sensor; }
};

};