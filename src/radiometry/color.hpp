#pragma once

#include <format>

#include "math/point.hpp"
#include "math/vector.hpp"

#include "sampled_spectrum.hpp"
#include "radiometry_constant/xyz_ilum_spectrum.hpp"

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
    static XYZ<T> from_standard_illuminant(const IlluminantSpectrumType& I) {
        T Xn = inner_product(I, CIE_X<T>);
        T Yn = inner_product(I, CIE_Y<T>);
        T Zn = inner_product(I, CIE_Z<T>);
        T denom = Yn;
        return XYZ<T>(Xn / denom, Yn / denom, Zn / denom);
    }

    template<typename ReflectanceSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_reflectance_under_illuminant(const ReflectanceSpectrumType& R,
                                                    const IlluminantSpectrumType& I) {
        // 分子：R(λ)*I(λ) 与 CIE CMF 的内积
        T Xn = inner_product(R * I, CIE_X<T>);
        T Yn = inner_product(R * I, CIE_Y<T>);
        T Zn = inner_product(R * I, CIE_Z<T>);
        // 分母：I(λ) 与 ȳ(λ) 的内积（归一化，使理想白 Y=1）
        T denom = inner_product(I, CIE_Y<T>);
        return XYZ<T>(Xn / denom, Yn / denom, Zn / denom);
    }

    // 发光谱 L 在“参考照明（D65）归一化”的 XYZ
    template<typename IlluminantSpectrumType, typename RefIlluminantSpectrumType>
    static XYZ<T> from_emission_under_illuminant(
        const IlluminantSpectrumType& L, const RefIlluminantSpectrumType& Iref // 例如 D65
    ) {
        T Xn = inner_product(L, CIE_X<T>);
        T Yn = inner_product(L, CIE_Y<T>);
        T Zn = inner_product(L, CIE_Z<T>);
        T denom = inner_product(Iref, CIE_Y<T>);   // 固定：D65 的 Y
        return XYZ<T>(Xn / denom, Yn / denom, Zn / denom);
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
