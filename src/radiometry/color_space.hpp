#pragma once

#include "math/matrix.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

#include "color.hpp"

namespace pbpt::radiometry {

template<typename T>
class RGBColorSpace {
private:
    math::Point<T, 2> m_r_xy;
    math::Point<T, 2> m_g_xy;
    math::Point<T, 2> m_b_xy;

    XYZ<T> m_white_point;

    math::Matrix<T, 3, 3> m_rgb_to_xyz;
    math::Matrix<T, 3, 3> m_xyz_to_rgb;

public:
    RGBColorSpace(
        const math::Point<T, 2>& r_xy, 
        const math::Point<T, 2>& g_xy, 
        const math::Point<T, 2>& b_xy, 
        const XYZ<T>& m_white_point
    ) : m_r_xy(r_xy), m_g_xy(g_xy), m_b_xy(b_xy), m_white_point(m_white_point) {
        XYZ<T> r_chroma = XYZ<T>::from_xyY(r_xy);
        XYZ<T> g_chroma = XYZ<T>::from_xyY(g_xy);
        XYZ<T> b_chroma = XYZ<T>::from_xyY(b_xy);

        math::Matrix<T, 3, 3> M{
            r_chroma, g_chroma, b_chroma
        };

        math::Vector<T, 3> s = M.inversed() * m_white_point;

        m_rgb_to_xyz = math::Matrix<T, 3, 3>{
            s.x() * r_chroma, s.y() * g_chroma, s.z() * b_chroma
        };

        m_xyz_to_rgb = m_rgb_to_xyz.inversed();
    }

    const math::Point<T, 2>& r_xy() const { return m_r_xy; }
    const math::Point<T, 2>& g_xy() const { return m_g_xy; }
    const math::Point<T, 2>& b_xy() const { return m_b_xy; }
    const XYZ<T>& white_point() const { return m_white_point; }

    const math::Matrix<T, 3, 3>& rgb_to_xyz_matrix() const { return m_rgb_to_xyz; }
    const math::Matrix<T, 3, 3>& xyz_to_rgb_matrix() const { return m_xyz_to_rgb; }

    XYZ<T> to_xyz(const RGB<T>& rgb) const {
        return m_rgb_to_xyz * rgb;
    }

    RGB<T> to_rgb(const XYZ<T>& xyz) const {
        return m_xyz_to_rgb * xyz;
    }

};

} 
