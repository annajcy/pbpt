/**
 * @file
 * @brief RGB color space definition and XYZ conversion.
 */
#pragma once

#include "pbpt/math/matrix.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"

#include "color.hpp"

namespace pbpt::radiometry {

/**
 * @brief Parametric RGB color space with primaries and white point.
 *
 * The color space is defined by the chromaticities of the red, green
 * and blue primaries and by a white point in XYZ. From these, the
 * matrices that convert between RGB and XYZ are derived.
 *
 * @tparam T Scalar type.
 */
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
    /// Default constructor - creates an uninitialized color space.
    RGBColorSpace() = default;
    
    /**
     * @brief Constructs an RGB color space from primaries and white point.
     *
     * The chromaticities of the red, green and blue primaries are given
     * as xy coordinates, and the white point is given in XYZ. The
     * constructor derives the matrices that convert between RGB and XYZ
     * so that the white point maps to RGB = (1, 1, 1).
     *
     * @param r_xy        Chromaticity of the red primary.
     * @param g_xy        Chromaticity of the green primary.
     * @param b_xy        Chromaticity of the blue primary.
     * @param m_white_point Reference white point in XYZ.
     */
    RGBColorSpace(
        const math::Point<T, 2>& r_xy, 
        const math::Point<T, 2>& g_xy, 
        const math::Point<T, 2>& b_xy, 
        const XYZ<T>& m_white_point
    ) : m_r_xy(r_xy), m_g_xy(g_xy), m_b_xy(b_xy), m_white_point(m_white_point) {
        XYZ<T> r_chroma = XYZ<T>::from_xyY(r_xy);
        XYZ<T> g_chroma = XYZ<T>::from_xyY(g_xy);
        XYZ<T> b_chroma = XYZ<T>::from_xyY(b_xy);

        math::Matrix<T, 3, 3> M = math::Matrix<T, 3, 3>::from_cols(
            r_chroma, 
            g_chroma, 
            b_chroma
        );

        math::Vector<T, 3> s = M.inversed() * m_white_point;

        m_rgb_to_xyz = math::Matrix<T, 3, 3>::from_cols( 
            s.x() * r_chroma, 
            s.y() * g_chroma, 
            s.z() * b_chroma
        );

        m_xyz_to_rgb = m_rgb_to_xyz.inversed();
    }

    /// Returns the red primary chromaticity (x, y).
    const math::Point<T, 2>& r_xy() const { return m_r_xy; }
    /// Returns the green primary chromaticity (x, y).
    const math::Point<T, 2>& g_xy() const { return m_g_xy; }
    /// Returns the blue primary chromaticity (x, y).
    const math::Point<T, 2>& b_xy() const { return m_b_xy; }
    /// Returns the white point in XYZ coordinates.
    const XYZ<T>& white_point() const { return m_white_point; }

    /// Returns the 3x3 matrix that converts RGB to XYZ.
    const math::Matrix<T, 3, 3>& rgb_to_xyz_matrix() const { return m_rgb_to_xyz; }
    /// Returns the 3x3 matrix that converts XYZ to RGB.
    const math::Matrix<T, 3, 3>& xyz_to_rgb_matrix() const { return m_xyz_to_rgb; }

    /// Converts a color from RGB to XYZ using this color space.
    XYZ<T> to_xyz(const RGB<T>& rgb) const {
        return m_rgb_to_xyz * rgb;
    }

    /// Converts a color from XYZ to RGB using this color space.
    RGB<T> to_rgb(const XYZ<T>& xyz) const {
        return m_xyz_to_rgb * xyz;
    }

};

} 
