#pragma once

#include <array>
#include <format>

#include "math/matrix.hpp"
#include "math/point.hpp"
#include "math/utils.hpp"
#include "math/vector.hpp"

#include "radiometry/radiometry_units.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "sampled_spectrum.hpp"
#include "constant/xyz_spectrum.hpp"

namespace pbpt::radiometry {

template<typename ResponseSpectrumType>
class ResponseSpectrum {
private:
    std::array<ResponseSpectrumType, 3> m_spectrum;

public:
    ResponseSpectrum(
        const std::array<ResponseSpectrumType, 3>& spectrum
    ) : m_spectrum{spectrum} {}

    ResponseSpectrum(
        const ResponseSpectrumType& r,
        const ResponseSpectrumType& g,
        const ResponseSpectrumType& b
    ) : m_spectrum{r,g,b} {}

    const ResponseSpectrumType& r() const { return m_spectrum[0]; }
    const ResponseSpectrumType& g() const { return m_spectrum[1]; }
    const ResponseSpectrumType& b() const { return m_spectrum[2]; }

    const ResponseSpectrumType& x() const { return m_spectrum[0]; }
    const ResponseSpectrumType& y() const { return m_spectrum[1]; }
    const ResponseSpectrumType& z() const { return m_spectrum[2]; }

    const ResponseSpectrumType& operator[](size_t index) const {
        math::assert_if(index >= 3, "Index out of bounds");
        return m_spectrum[index];
    }

};

template<typename T, template <typename> class Color, typename SpectrumType, typename ResponseSpectrumType> 
inline constexpr Color<T> project_spectrum(
    const SpectrumType& spectrum,
    const ResponseSpectrum<ResponseSpectrumType>& response
) {
    T r{}, g{}, b{};
    for (int lambda = lambda_min<int>; lambda <= lambda_max<int>; ++lambda) {
        r += spectrum.at(lambda) * response.r().at(lambda);
        g += spectrum.at(lambda) * response.g().at(lambda);
        b += spectrum.at(lambda) * response.b().at(lambda);
    }
    return Color<T> (r, g, b);
}

template<typename T, template <typename> class Color, typename IlluminantSpectrumType, typename ResponseSpectrumType> 
inline constexpr Color<T> project_illuminant(
    const IlluminantSpectrumType& illuminant,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r{}, g{}, b{};
    T g_integral{};
    for (int lambda = lambda_min<int>; lambda <= lambda_max<int>; ++lambda) {
        g_integral += illuminant.at(lambda) * response.g().at(lambda);
        r += illuminant.at(lambda) * response.r().at(lambda);
        g += illuminant.at(lambda) * response.g().at(lambda);
        b += illuminant.at(lambda) * response.b().at(lambda);
    }

    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T> (r, g, b);
    }

    return Color<T> (
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

template<typename T, template <typename> class Color, typename ReflectanceSpectrumType, typename IlluminantSpectrumType, typename ResponseSpectrumType> 
inline constexpr Color<T> project_reflectance(
    const ReflectanceSpectrumType& reflectance,
    const IlluminantSpectrumType& illuminant,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r{}, g{}, b{};
    T g_integral{};
    for (int lambda = lambda_min<int>; lambda <= lambda_max<int>; ++lambda) {
        g_integral += illuminant.at(lambda) * response.g().at(lambda);
        r += reflectance.at(lambda) * illuminant.at(lambda) * response.r().at(lambda);
        g += reflectance.at(lambda) * illuminant.at(lambda) * response.g().at(lambda);
        b += reflectance.at(lambda) * illuminant.at(lambda) * response.b().at(lambda);
    }

    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T>(r, g, b);
    }

    return Color<T>(
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

template<typename T, template <typename> class Color, typename EmissionSpectrumType, typename IlluminantSpectrumType, typename ResponseSpectrumType> 
inline constexpr Color<T> project_emission(
    const EmissionSpectrumType& emission,
    const IlluminantSpectrumType& illuminant,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r{}, g{}, b{};
    T g_integral{};
    for (int lambda = lambda_min<int>; lambda <= lambda_max<int>; ++lambda) {
        g_integral += illuminant.at(lambda) * response.g().at(lambda);
        r += emission.at(lambda) * response.r().at(lambda);
        g += emission.at(lambda) * response.g().at(lambda);
        b += emission.at(lambda) * response.b().at(lambda);
    }

    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T>(r, g, b);
    }

    return Color<T>(
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

template<typename T, template <typename> class Color, int N, typename ResponseSpectrumType>
inline constexpr Color<T> project_sampled_spectrum(
    const SampledSpectrum<T, N>& spectrum, 
    const SampledWavelength<T, N>& wavelengths, 
    const SampledPdf<T, N>& pdf,
    const ResponseSpectrum<ResponseSpectrumType>& response
) {
    return Color<T>(
        (response.r().sample(wavelengths) * spectrum * pdf.inv()).average(), 
        (response.g().sample(wavelengths) * spectrum * pdf.inv()).average(), 
        (response.b().sample(wavelengths) * spectrum * pdf.inv()).average()
    );
}

template<typename T, template <typename> class Color, int N, typename ResponseSpectrumType>
inline constexpr Color<T> project_sampled_illuminant(
    const SampledSpectrum<T, N>& illuminant, 
    const SampledWavelength<T, N>& wavelengths, 
    const SampledPdf<T, N>& pdf,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r = (response.r().sample(wavelengths) * illuminant * pdf.inv()).average();
    T g = (response.g().sample(wavelengths) * illuminant * pdf.inv()).average();
    T b = (response.b().sample(wavelengths) * illuminant * pdf.inv()).average();
    T g_integral = (response.g().sample(wavelengths) * illuminant * pdf.inv()).average();

    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T>(r, g, b);
    }

    return Color<T>(
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

template<typename T, template <typename> class Color, int N, typename ReflectanceSpectrumType, typename IlluminantSpectrumType, typename ResponseSpectrumType>
inline constexpr Color<T> project_sampled_reflectance(
    const SampledSpectrum<T, N>& reflectance, 
    const SampledSpectrum<T, N>& illuminant, 
    const SampledWavelength<T, N>& wavelengths, 
    const SampledPdf<T, N>& pdf,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r = (response.r().sample(wavelengths) * reflectance * illuminant * pdf.inv()).average();
    T g = (response.g().sample(wavelengths) * reflectance * illuminant * pdf.inv()).average();
    T b = (response.b().sample(wavelengths) * reflectance * illuminant * pdf.inv()).average();
    T g_integral = (response.g().sample(wavelengths) * illuminant * pdf.inv()).average();
    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T>(r, g, b);
    }
    return Color<T>(
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

template<typename T, template <typename> class Color, int N, typename EmissionSpectrumType, typename IlluminantSpectrumType, typename ResponseSpectrumType>
inline constexpr Color<T> project_sampled_emission(
    const SampledSpectrum<T, N>& emission, 
    const SampledSpectrum<T, N>& illuminant, 
    const SampledWavelength<T, N>& wavelengths, 
    const SampledPdf<T, N>& pdf,
    const ResponseSpectrum<ResponseSpectrumType>& response,
    bool is_normalize_by_g = true
) {
    T r = (response.r().sample(wavelengths) * emission * pdf.inv()).average();
    T g = (response.g().sample(wavelengths) * emission * pdf.inv()).average();
    T b = (response.b().sample(wavelengths) * emission * pdf.inv()).average();
    T g_integral = (response.g().sample(wavelengths) * illuminant * pdf.inv()).average();
    if (!is_normalize_by_g || math::is_zero(g_integral)) {
        return Color<T>(r, g, b);
    }

    return Color<T>(
        r / g_integral,
        g / g_integral,
        b / g_integral
    );
}

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
    static XYZ<T> from_spectrum(const SpectrumType& spectrum) {
        using ResponseSpectrumType = TabularSpectrumDistribution<T, 
            constant::XYZRange::LMinValue, constant::XYZRange::LMaxValue>;
        return project_spectrum<T, XYZ, SpectrumType, ResponseSpectrumType>(
            spectrum, 
            ResponseSpectrum<ResponseSpectrumType>{
                constant::CIE_X<T>,
                constant::CIE_Y<T>,
                constant::CIE_Z<T>
            }
        );
    }

    template<typename IlluminantSpectrumType>
    static XYZ<T> from_illuminant(const IlluminantSpectrumType& illuminant) {
        using ResponseSpectrumType = TabularSpectrumDistribution<T, 
            constant::XYZRange::LMinValue, constant::XYZRange::LMaxValue>;
        return project_illuminant<T, XYZ, IlluminantSpectrumType, ResponseSpectrumType>(
            illuminant, 
            ResponseSpectrum<ResponseSpectrumType>{
                constant::CIE_X<T>,
                constant::CIE_Y<T>,
                constant::CIE_Z<T>
            }
        );
    }

    template<typename ReflectanceSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_reflectance(
        const ReflectanceSpectrumType& reflectance,
        const IlluminantSpectrumType& illuminant
    ) {
        using ResponseSpectrumType = TabularSpectrumDistribution<T, 
            constant::XYZRange::LMinValue, constant::XYZRange::LMaxValue>;
        return project_reflectance<T, XYZ, ReflectanceSpectrumType, IlluminantSpectrumType, ResponseSpectrumType>(
            reflectance, 
            illuminant, 
            ResponseSpectrum<ResponseSpectrumType>{
                constant::CIE_X<T>,
                constant::CIE_Y<T>,
                constant::CIE_Z<T>
            }
        );
    }

    // emission with referenced illuminant XYZ
    template<typename EmissionSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_emission(
        const EmissionSpectrumType& emission, 
        const IlluminantSpectrumType& illuminant // e.g D65
    ) {
        using ResponseSpectrumType = TabularSpectrumDistribution<T, 
            constant::XYZRange::LMinValue, constant::XYZRange::LMaxValue>;
        return project_emission<T, XYZ, EmissionSpectrumType, IlluminantSpectrumType, ResponseSpectrumType>(
            emission, 
            illuminant, 
            ResponseSpectrum<ResponseSpectrumType>{
                constant::CIE_X<T>,
                constant::CIE_Y<T>,
                constant::CIE_Z<T>
            }
        );
    }

    template<int N>
    static constexpr XYZ from_sampled_spectrum(
        const SampledSpectrum<T, N>& spectrum, 
        const SampledWavelength<T, N>& wavelengths, 
        const SampledPdf<T, N>& pdf
    ) {
        using ResponseSpectrumType = TabularSpectrumDistribution<T, 
            constant::XYZRange::LMinValue, constant::XYZRange::LMaxValue>;
        return project_sampled_spectrum<T, XYZ, N, ResponseSpectrumType>(
            spectrum, 
            wavelengths, 
            pdf,
            ResponseSpectrum<ResponseSpectrumType>{
                constant::CIE_X<T>,
                constant::CIE_Y<T>,
                constant::CIE_Z<T>
            }
        );
    }

    T x() const { return this->operator[](0); }
    T y() const { return this->operator[](1); }
    T z() const { return this->operator[](2); }

    math::Point<T, 2> to_xy() const {
        return math::Point<T, 2>(
            x() / (x() + y() + z()), 
            y() / (x() + y() + z())
        );
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
class LMS : public math::Vector<T, 3> {
public:

    static constexpr math::Matrix<T, 3, 3> lms_to_xyz_matrix() {
        return math::Matrix<T, 3, 3>(
            0.986993, -0.147054, 0.159963,
            0.432305, 0.51836, 0.0492912,
            -0.00852866, 0.0400428, 0.968487
        );
    }

    static constexpr math::Matrix<T, 3, 3> xyz_to_lms_matrix() {
        return math::Matrix<T, 3, 3>(
            0.8951, 0.2664, -0.1614,
            -0.7502, 1.7135, 0.0367,
            0.0389, -0.0685, 1.0296
        );
    }

public:
    LMS() : math::Vector<T, 3>(0, 0, 0) {}
    LMS(T l, T m, T s) : math::Vector<T, 3>(l, m, s) {}
    LMS(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    T l() const { return this->operator[](0); }
    T m() const { return this->operator[](1); }
    T s() const { return this->operator[](2); }

    XYZ<T> to_xyz() const {
        return XYZ<T>(LMS<T>::lms_to_xyz_matrix() * (*this));
    }

    static LMS<T> from_xyz(const XYZ<T>& xyz) {
        return LMS<T>(LMS<T>::xyz_to_lms_matrix() * xyz);
    }
};

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

    const T& r() const { return this->operator[](0); }
    const T& g() const { return this->operator[](1); }
    const T& b() const { return this->operator[](2); }

    T& r() { return this->operator[](0); }
    T& g() { return this->operator[](1); }
    T& b() { return this->operator[](2); }

    RGB<T> clamp(T min = T{0}, T max = T{1}) {
        return RGB<T>(
            std::clamp(r(), min, max),
            std::clamp(g(), min, max),
            std::clamp(b(), min, max)
        );
    }

    template<typename SpectrumType, typename ResponseSpectrumType>
    static RGB<T> from_spectrum(
        const SpectrumType& spectrum,
        const ResponseSpectrum<ResponseSpectrumType>& response
    ) {
        return project_spectrum<T, RGB, SpectrumType, ResponseSpectrumType>(
            spectrum, 
            response
        );
    }

    template<int N, typename ResponseSpectrumType>
    RGB<T> from_sampled_spectrum(
        const SampledSpectrum<T, N>& spectrum, 
        const SampledWavelength<T, N>& wavelengths,
        const SampledPdf<T, N> &pdf,
        const ResponseSpectrum<ResponseSpectrumType>& response
    ) const {
        return project_sampled_spectrum<T, RGB, N, ResponseSpectrumType>(
            spectrum,
            wavelengths,
            pdf,
            response
        );
    }
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
inline constexpr math::Matrix<T, 3, 3> white_balance(
    math::Point<T, 2> src_chroma_xy,
    math::Point<T, 2> dst_chroma_xy
) {
    auto src_XYZ = radiometry::XYZ<T>::from_xyY(src_chroma_xy, T{1}), dst_XYZ = radiometry::XYZ<T>::from_xyY(dst_chroma_xy, T{1});
    auto src_LMS = radiometry::LMS<T>::from_xyz(src_XYZ), dst_LMS = radiometry::LMS<T>::from_xyz(dst_XYZ);
    T scale_X = dst_LMS.x() / src_LMS.x();
    T scale_Y = dst_LMS.y() / src_LMS.y();
    T scale_Z = dst_LMS.z() / src_LMS.z();
    math::Matrix<T, 3, 3> wb_matrix = math::Matrix<T, 3, 3>::zeros();
    wb_matrix[0][0] = scale_X;
    wb_matrix[1][1] = scale_Y;
    wb_matrix[2][2] = scale_Z;
    return  radiometry::LMS<T>::lms_to_xyz_matrix() * 
            wb_matrix * 
            radiometry::LMS<T>::xyz_to_lms_matrix();
}

template <typename T>
T encode_to_nonlinear_srgb_channel(T linear) {
    linear = std::clamp(linear, T{0}, T{1});
    if (linear <= T{0.0031308}) {
        return linear * T{12.92};
    }
    return T{1.055} * std::pow(linear, T{1.0 / 2.4}) - T{0.055};
}

template <typename T>
RGB<T> encode_to_nonlinear_srgb(const RGB<T>& linear_rgb) {
    return RGB<T>(
        encode_to_nonlinear_srgb_channel(linear_rgb.r()),
        encode_to_nonlinear_srgb_channel(linear_rgb.g()),
        encode_to_nonlinear_srgb_channel(linear_rgb.b())
    );
}

template<typename T>
T decode_from_nonlinear_srgb_channel(T nonlinear) {
    nonlinear =  std::clamp(nonlinear, T{0}, T{1});
    if (nonlinear <= T{0.04045}) {
        return nonlinear / T{12.92};
    }
    return std::pow((nonlinear + T{0.055}) / T{1.055}, T{2.4});
}

template<typename T>
RGB<T> decode_from_nonlinear_srgb(const RGB<T>& nonlinear_rgb) {
    return RGB<T>(
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.r()),
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.g()),
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.b())
    );
}

} // namespace pbpt::radiometry