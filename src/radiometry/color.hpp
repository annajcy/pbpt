#pragma once

#include <format>

#include "math/point.hpp"
#include "math/vector.hpp"

#include "sampled_spectrum.hpp"
#include "constant/xyz_spectrum.hpp"

namespace pbpt::radiometry {

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

    template<typename IlluminantSpectrumType>
    static XYZ<T> from_standard_illuminant(const IlluminantSpectrumType& illuminant) {
        T X = inner_product(illuminant, constant::CIE_X<T>);
        T Y = inner_product(illuminant, constant::CIE_Y<T>);
        T Z = inner_product(illuminant, constant::CIE_Z<T>);
        T Y_integral = Y;
        return XYZ<T>(X / Y_integral, Y / Y_integral, Z / Y_integral);
    }

    template<typename ReflectanceSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_reflectance_under_illuminant(const ReflectanceSpectrumType& reflectance,
                                                    const IlluminantSpectrumType& illuminant) {
        // 分子：R(λ)*I(λ) 与 CIE CMF 的内积
        T X = inner_product(reflectance * illuminant, constant::CIE_X<T>);
        T Y = inner_product(reflectance * illuminant, constant::CIE_Y<T>);
        T Z = inner_product(reflectance * illuminant, constant::CIE_Z<T>);
        // 分母：I(λ) 与 ȳ(λ) 的内积（归一化，使理想白 Y=1）
        T Y_integral = inner_product(illuminant, constant::CIE_Y<T>);
        return XYZ<T>(X / Y_integral, Y / Y_integral, Z / Y_integral);
    }

    // 发光谱 L 在“参考照明（D65）归一化”的 XYZ
    template<typename EmissionSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_emission_under_illuminant(
        const EmissionSpectrumType& emission, const IlluminantSpectrumType& illuminant // 例如 D65
    ) {
        T X = inner_product(emission, constant::CIE_X<T>);
        T Y = inner_product(emission, constant::CIE_Y<T>);
        T Z = inner_product(emission, constant::CIE_Z<T>);
        T Y_integral = inner_product(illuminant, constant::CIE_Y<T>);   // 固定：D65 的 Y
        return XYZ<T>(X / Y_integral, Y / Y_integral, Z / Y_integral);
    }

    template<int N>
    static constexpr XYZ from_sampled_radiance(
        const SampledSpectrum<T, N>& radiance, 
        const SampledWavelength<T, N>& wavelengths, 
        const SampledPdf<T, N>& pdf
    ) {
        SampledSpectrum<T, N> X_response = constant::CIE_X<T>.sample(wavelengths);
        SampledSpectrum<T, N> Y_response = constant::CIE_Y<T>.sample(wavelengths);
        SampledSpectrum<T, N> Z_response = constant::CIE_Z<T>.sample(wavelengths);
        return XYZ(
            (X_response * radiance * pdf.inv()).average(), 
            (Y_response * radiance * pdf.inv()).average(), 
            (Z_response * radiance * pdf.inv()).average()
        );
    }

    T x() const { return this->operator[](0); }
    T y() const { return this->operator[](1); }
    T z() const { return this->operator[](2); }

    math::Point<T, 2> to_xy() const {
        return math::Point<T, 2>(x() / (x() + y() + z()), y() / (x() + y() + z()));
    }

    XYZ<T>& normalize_to_y(T target_y = T{100}) {
        T current_y = y();
        if (current_y != 0) {
            T scale = target_y / current_y;
            *this *= scale;
        }
        return *this;
    }

    XYZ<T> normalized_to_y(T target_y = T{100}) const {
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
class LAB : public math::Vector<T, 3> {
public:
    LAB() : math::Vector<T, 3>(0, 0, 0) {}
    LAB(T L, T a, T b) : math::Vector<T, 3>(L, a, b) {}
    LAB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    static LAB<T> from_xyz(const XYZ<T>& xyz, const XYZ<T>& white_point) {
        auto f = [](T t) {
            const T delta = T(6) / T(29);
            if (t > delta * delta * delta) {
                return std::cbrt(t);
            } else {
                return t / (3 * delta * delta) + T(4) / T(29);
            }
        };

        T fx = f(xyz.x() / white_point.x());
        T fy = f(xyz.y() / white_point.y());
        T fz = f(xyz.z() / white_point.z());

        T L = T(116) * fy - T(16);
        T a = T(500) * (fx - fy);
        T b = T(200) * (fy - fz);

        return LAB<T>(L, a, b);
    }

    T L() const { return this->operator[](0); }
    T a() const { return this->operator[](1); }
    T b() const { return this->operator[](2); }
};

template<typename T>
std::string to_string(const LAB<T>& lab) {
    return std::format("LAB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       lab.L(), 
                       lab.a(), 
                       lab.b());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const LAB<T>& lab) {
    return os << to_string(lab);
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

}
