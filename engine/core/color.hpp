#pragma once

#include "core/spectrum.hpp"

#include "math/matrix.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::core {

template<typename T>
class XYZ : public math::Vector<T, 3> {
public:
    XYZ() : math::Vector<T, 3>(0, 0, 0) {}
    XYZ(T x, T y, T z) : math::Vector<T, 3>(x, y, z) {}
    XYZ(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    static constexpr XYZ from_xyY(const math::Point<T, 2>& xy, T Y = T{1}) {
        if (xy.y() == 0) {
            return XYZ(0, 0, 0);
        }
        T X = (xy.x() * Y) / xy.y();
        T Z = ((1 - xy.x() - xy.y()) * Y) / xy.y();
        return XYZ(X, Y, Z);
    }

    template<typename SpectrumType>
    static constexpr XYZ from_spectrum_distribution(const SpectrumType& spectrum) {
        auto xyz = XYZ(
            inner_product(spectrum, CIE_X<T>), // X
            inner_product(spectrum, CIE_Y<T>), // Y
            inner_product(spectrum, CIE_Z<T>)  // Z
        );
        return xyz;
    }

    template<int N>
    static constexpr XYZ from_sampled_spectrum(
        const SampledSpectrum<T, N>& spectrum, 
        const SampledWavelength<T, N>& wavelengths, 
        const SampledPdf<T, N>& pdf) 
    {
        SampledSpectrum<T, N> X = CIE_X<T>.sample(wavelengths);
        SampledSpectrum<T, N> Y = CIE_Y<T>.sample(wavelengths);
        SampledSpectrum<T, N> Z = CIE_Z<T>.sample(wavelengths);
        return XYZ(
            (X * spectrum * pdf.inv()).average(), 
            (Y * spectrum * pdf.inv()).average(), 
            (Z * spectrum * pdf.inv()).average()
        );
    }

    T x() const { return this->operator[](0); }
    T y() const { return this->operator[](1); }
    T z() const { return this->operator[](2); }

    math::Point<T, 2> to_xy() const {
        return math::Point<T, 2>(x() / (x() + y() + z()), y() / (x() + y() + z()));
    }

    XYZ<T>& normalize_to_y(T target_y = T{1}) {
        T current_y = y();
        if (current_y != 0) {
            T scale = target_y / current_y;
            *this *= scale;
        }
        return *this;
    }

    XYZ<T> normalized_to_y(T target_y) const {
        return XYZ<T>(*this).normalize_to_y(target_y);
    }
};


template<typename T>
std::string to_string(const XYZ<T>& xyz) {
    return std::format("XYZ<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       xyz.x(), 
                       xyz.y(), 
                       xyz.z());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const XYZ<T>& xyz) {
    return os << to_string(xyz);
}


template<typename T>
class RGB : public math::Vector<T, 3> {
public:
    RGB() : math::Vector<T, 3>(0, 0, 0) {}
    RGB(T r, T g, T b) : math::Vector<T, 3>(r, g, b) {}
    RGB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    T r() const { return this->operator[](0); }
    T g() const { return this->operator[](1); }
    T b() const { return this->operator[](2); }
};

template<typename T>
std::string to_string(const RGB<T>& rgb) {
    return std::format("RGB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       rgb.r(), 
                       rgb.g(), 
                       rgb.b());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const RGB<T>& rgb) {
    return os << to_string(rgb);
}

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

};