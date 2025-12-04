/**
 * @file
 * @brief Color representations (XYZ, LMS, LAB, RGB) and spectrum projection.
 */
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

/**
 * @brief Triplet of spectral response curves (e.g. R, G, B or X, Y, Z).
 *
 * Each component is itself a spectrum that can be queried at wavelength
 * lambda. The template parameter @p ResponseSpectrumType is usually a
 * @c SpectrumDistribution.
 *
 * @tparam ResponseSpectrumType Underlying spectrum type for each channel.
 */
template<typename ResponseSpectrumType>
class ResponseSpectrum {
private:
    std::array<ResponseSpectrumType, 3> m_spectrum;

public:
    /**
     * @brief Constructs a response spectrum from three channel spectra.
     *
     * The channels are typically ordered as R, G, B or X, Y, Z.
     */
    ResponseSpectrum(
        const std::array<ResponseSpectrumType, 3>& spectrum
    ) : m_spectrum{spectrum} {}

    /**
     * @brief Constructs a response spectrum from individual channels.
     *
     * @param r Response spectrum for the first channel.
     * @param g Response spectrum for the second channel.
     * @param b Response spectrum for the third channel.
     */
    ResponseSpectrum(
        const ResponseSpectrumType& r,
        const ResponseSpectrumType& g,
        const ResponseSpectrumType& b
    ) : m_spectrum{r,g,b} {}

    /// Returns the first (R-like) response channel.
    const ResponseSpectrumType& r() const { return m_spectrum[0]; }
    /// Returns the second (G-like) response channel.
    const ResponseSpectrumType& g() const { return m_spectrum[1]; }
    /// Returns the third (B-like) response channel.
    const ResponseSpectrumType& b() const { return m_spectrum[2]; }

    /// Returns the X-like response channel (alias of r()).
    const ResponseSpectrumType& x() const { return m_spectrum[0]; }
    /// Returns the Y-like response channel (alias of g()).
    const ResponseSpectrumType& y() const { return m_spectrum[1]; }
    /// Returns the Z-like response channel (alias of b()).
    const ResponseSpectrumType& z() const { return m_spectrum[2]; }

    /**
     * @brief Indexed access to the channel spectra.
     *
     * @param index Channel index in [0, 2].
     * @return Const reference to the selected channel spectrum.
     */
    const ResponseSpectrumType& operator[](size_t index) const {
        math::assert_if(index >= 3, "Index out of bounds");
        return m_spectrum[index];
    }

};

/**
 * @brief Project a spectrum onto a 3-channel response.
 *
 * Computes three channel values by summing spectrum(lambda) *
 * response.channel(lambda) over the tabular wavelength range.
 */
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

/**
 * @brief Project an illuminant onto a 3-channel response.
 *
 * If @p is_normalize_by_g is true and the green channel integral is
 * non-zero, the result is normalized so that the green channel is 1.
 */
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

/**
 * @brief Project a reflectance illuminated by a given illuminant.
 *
 * Integrates reflectance(lambda) * illuminant(lambda) * response(lambda)
 * over wavelength, with optional normalization by the green channel.
 */
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

/**
 * @brief Project an emission spectrum using a response and reference illuminant.
 *
 * Similar to @c project_illuminant, but using an emission spectrum
 * instead of a reflectance.
 */
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

/**
 * @brief Monte Carlo version of @c project_spectrum using sampled wavelengths.
 */
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

/**
 * @brief Monte Carlo version of @c project_illuminant using sampled wavelengths.
 */
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

/**
 * @brief Monte Carlo version of @c project_reflectance using sampled wavelengths.
 */
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

/**
 * @brief Monte Carlo version of @c project_emission using sampled wavelengths.
 */
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

/**
 * @brief CIE XYZ tristimulus color.
 *
 * Represents absolute or relative color in the CIE 1931 XYZ space.
 * Provides helpers to build XYZ from spectra, illuminants and
 * reflectances using the CIE color-matching functions.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class XYZ : public math::Vector<T, 3> {
public:
    /// Construct an XYZ color initialized to (0, 0, 0).
    XYZ() : math::Vector<T, 3>(0, 0, 0) {}
    /// Construct an XYZ color from components.
    XYZ(T x, T y, T z) : math::Vector<T, 3>(x, y, z) {}
    /// Construct an XYZ color from a generic 3D vector.
    XYZ(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    /**
     * @brief Construct XYZ from chromaticity (x, y) and luminance Y.
     *
     * Uses the standard relations:
     *   X = x * Y / y, Z = (1 - x - y) * Y / y,
     * with a safeguard for y = 0 returning black.
     */
    static constexpr XYZ from_xyY(const math::Point<T, 2>& xy, T Y = T{1}) {
        if (xy.y() == 0) {
            return XYZ(0, 0, 0);
        }
        T X = (xy.x() * Y) / xy.y();
        T Z = ((1 - xy.x() - xy.y()) * Y) / xy.y();
        return XYZ(X, Y, Z);
    }

    /**
     * @brief Construct XYZ by integrating a spectrum against CIE XYZ CMFs.
     *
     * Uses the tabulated CIE X, Y, Z color-matching functions and
     * `project_spectrum` over the standard wavelength range.
     */
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

    /**
     * @brief Construct XYZ of an illuminant under the CIE XYZ observer.
     */
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

    /**
     * @brief Construct XYZ of a reflectance under a given illuminant.
     */
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
    /**
     * @brief Construct XYZ of an emission spectrum under a reference illuminant.
     *
     * This is typically used when an emission is meant to be referenced
     * to a particular white point (e.g. D65).
     */
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

    /**
     * @brief Construct XYZ from sampled spectrum data using Monte Carlo integration.
     */
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

    /// Get the X component.
    T x() const { return this->operator[](0); }
    /// Get the Y component (luminance-like).
    T y() const { return this->operator[](1); }
    /// Get the Z component.
    T z() const { return this->operator[](2); }

    /// Convert XYZ to chromaticity coordinates (x, y) with Y used as relative luminance.
    math::Point<T, 2> to_xy() const {
        return math::Point<T, 2>(
            x() / (x() + y() + z()), 
            y() / (x() + y() + z())
        );
    }

    XYZ<T>& normalize_to_y(T target_y = T{100}) {
        /// Normalize in-place so that the Y component equals @p target_y.
        T current_y = y();
        if (current_y != 0) {
            T scale = target_y / current_y;
            *this *= scale;
        }
        return *this;
    }

    /// Return a copy normalized so that Y equals target_y.
    XYZ<T> normalized_to_y(T target_y = T{100}) const {
        return XYZ<T>(*this).normalize_to_y(target_y);
    }
};

/**
 * @brief Convert XYZ color to a human-readable string.
 */
template<typename T>
std::string to_string(const XYZ<T>& xyz) {
    return std::format("XYZ<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       xyz.x(), 
                       xyz.y(), 
                       xyz.z());
}

/**
 * @brief Stream insertion operator for XYZ colors.
 */
template<typename T>
std::ostream& operator<<(std::ostream& os, const XYZ<T>& xyz) {
    return os << to_string(xyz);
}

/**
 * @brief LMS cone-response color space.
 *
 * Encodes color in a space approximating long-, medium- and
 * short-wavelength cone responses, using the Bradford matrices for
 * conversion to and from XYZ.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class LMS : public math::Vector<T, 3> {
public:

    /// Matrix that converts from LMS to XYZ in the Bradford space.
    static constexpr math::Matrix<T, 3, 3> lms_to_xyz_matrix() {
        return math::Matrix<T, 3, 3>(
            0.986993, -0.147054, 0.159963,
            0.432305, 0.51836, 0.0492912,
            -0.00852866, 0.0400428, 0.968487
        );
    }

    /**
     * @brief Matrix that converts from XYZ to LMS in the Bradford space.
     *
     * This is the inverse of lms_to_xyz_matrix() and is used to map
     * XYZ tristimulus values to approximate cone responses.
     */
    static constexpr math::Matrix<T, 3, 3> xyz_to_lms_matrix() {
        return math::Matrix<T, 3, 3>(
            0.8951, 0.2664, -0.1614,
            -0.7502, 1.7135, 0.0367,
            0.0389, -0.0685, 1.0296
        );
    }

public:
    /// Construct an LMS color initialized to (0, 0, 0).
    LMS() : math::Vector<T, 3>(0, 0, 0) {}
    /// Construct an LMS color from components.
    LMS(T l, T m, T s) : math::Vector<T, 3>(l, m, s) {}
    /// Construct an LMS color from a generic 3D vector.
    LMS(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    /// Returns the L (long-wavelength) channel.
    T l() const { return this->operator[](0); }
    /// Returns the M (medium-wavelength) channel.
    T m() const { return this->operator[](1); }
    /// Returns the S (short-wavelength) channel.
    T s() const { return this->operator[](2); }

    /**
     * @brief Converts this LMS color to XYZ using the Bradford matrix.
     */
    XYZ<T> to_xyz() const {
        return XYZ<T>(LMS<T>::lms_to_xyz_matrix() * (*this));
    }

    /**
     * @brief Converts an XYZ color to LMS using the Bradford matrix.
     *
     * @param xyz Input color in XYZ space.
     */
    static LMS<T> from_xyz(const XYZ<T>& xyz) {
        return LMS<T>(LMS<T>::xyz_to_lms_matrix() * xyz);
    }
};

/**
 * @brief CIE L*a*b* perceptual color space.
 *
 * Encodes color in terms of lightness L* and chromatic components
 * a* and b*, relative to a reference white point.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class LAB : public math::Vector<T, 3> {
public:
    /// Construct a LAB color initialized to (0, 0, 0).
    LAB() : math::Vector<T, 3>(0, 0, 0) {}
    /// Construct a LAB color from components.
    LAB(T L, T a, T b) : math::Vector<T, 3>(L, a, b) {}
    /// Construct a LAB color from a generic 3D vector.
    LAB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    /**
     * @brief Converts from XYZ to LAB using the standard CIE formula.
     *
     * The XYZ color is expressed relative to a reference white point.
     * Internally this applies the non-linear f(t) function defined by
     * the CIE model and computes:
     *   L* = 116 * f(Y/Yn) - 16
     *   a* = 500 * (f(X/Xn) - f(Y/Yn))
     *   b* = 200 * (f(Y/Yn) - f(Z/Zn)).
     *
     * @param xyz         Input color in XYZ space.
     * @param white_point Reference white in XYZ coordinates.
     */
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

    /// Returns the L* lightness component.
    T L() const { return this->operator[](0); }
    /// Returns the a* chroma component along the green–magenta axis.
    T a() const { return this->operator[](1); }
    /// Returns the b* chroma component along the blue–yellow axis.
    T b() const { return this->operator[](2); }
};

/**
 * @brief Convert LAB color to a human-readable string.
 */
template<typename T>
std::string to_string(const LAB<T>& lab) {
    return std::format("LAB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       lab.L(), 
                       lab.a(), 
                       lab.b());
}

/**
 * @brief Stream insertion operator for LAB colors.
 */
template<typename T>
std::ostream& operator<<(std::ostream& os, const LAB<T>& lab) {
    return os << to_string(lab);
}

/**
 * @brief Generic linear RGB color triplet.
 *
 * Represents linear (not gamma-encoded) RGB values in some color space.
 * Conversions to/from spectra are defined via ResponseSpectrum.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class RGB : public math::Vector<T, 3> {
public:
    /// Construct an RGB color initialized to (0, 0, 0).
    RGB() : math::Vector<T, 3>(0, 0, 0) {}
    /// Construct an RGB color from components.
    RGB(T r, T g, T b) : math::Vector<T, 3>(r, g, b) {}
    /// Construct an RGB color from a generic 3D vector.
    RGB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    /// Returns the red channel (const).
    const T& r() const { return this->operator[](0); }
    /// Returns the green channel (const).
    const T& g() const { return this->operator[](1); }
    /// Returns the blue channel (const).
    const T& b() const { return this->operator[](2); }

    /// Returns the red channel (mutable).
    T& r() { return this->operator[](0); }
    /// Returns the green channel (mutable).
    T& g() { return this->operator[](1); }
    /// Returns the blue channel (mutable).
    T& b() { return this->operator[](2); }

    /**
     * @brief Returns a copy of this RGB color with each channel clamped.
     *
     * Channels are clamped independently to the interval [min, max].
     * The default range [0,1] is suitable for normalized linear RGB.
     */
    RGB<T> clamp(T min = T{0}, T max = T{1}) {
        return RGB<T>(
            std::clamp(r(), min, max),
            std::clamp(g(), min, max),
            std::clamp(b(), min, max)
        );
    }

    /**
     * @brief Converts a spectrum to RGB using a three-channel response.
     *
     * The spectrum is projected onto the response channels using
     * `project_spectrum`, which effectively integrates spectrum(lambda)
     * times each response channel over wavelength.
     *
     * @tparam SpectrumType        Type of the input spectrum.
     * @tparam ResponseSpectrumType Spectrum type used for each response channel.
     * @param spectrum             Input spectral distribution.
     * @param response             Triplet of response spectra (R, G, B).
     */
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

    /**
     * @brief Monte Carlo conversion from a sampled spectrum to RGB.
     *
     * Uses randomly sampled wavelengths, their values and sampling pdf
     * to estimate the RGB channels via `project_sampled_spectrum`.
     *
     * @tparam N                    Number of samples.
     * @tparam ResponseSpectrumType Spectrum type used for each response channel.
     * @param spectrum              Sampled spectrum values.
     * @param wavelengths           Sampled wavelengths.
     * @param pdf                   Sampling probability density for each sample.
     * @param response              Triplet of response spectra (R, G, B).
     */
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

/**
 * @brief Convert RGB color to a human-readable string.
 */
template<typename T>
std::string to_string(const RGB<T>& rgb) {
    return std::format("RGB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       rgb.r(), 
                       rgb.g(), 
                       rgb.b());
}

/**
 * @brief Stream insertion operator for RGB colors.
 */
template<typename T>
std::ostream& operator<<(std::ostream& os, const RGB<T>& rgb) {
    return os << to_string(rgb);
}

/**
 * @brief Compute a simple white-balance matrix between two chromaticities.
 *
 * Builds a diagonal scaling in LMS space that maps the source white
 * chromaticity to the destination white, and converts that back to XYZ.
 */
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

/**
 * @brief Encode a single linear sRGB channel to non-linear (gamma) sRGB.
 */
template <typename T>
T encode_to_nonlinear_srgb_channel(T linear) {
    linear = std::clamp(linear, T{0}, T{1});
    if (linear <= T{0.0031308}) {
        return linear * T{12.92};
    }
    return T{1.055} * std::pow(linear, T{1.0 / 2.4}) - T{0.055};
}

/**
 * @brief Encode linear RGB to non-linear sRGB using the standard transfer curve.
 */
template <typename T>
RGB<T> encode_to_nonlinear_srgb(const RGB<T>& linear_rgb) {
    return RGB<T>(
        encode_to_nonlinear_srgb_channel(linear_rgb.r()),
        encode_to_nonlinear_srgb_channel(linear_rgb.g()),
        encode_to_nonlinear_srgb_channel(linear_rgb.b())
    );
}

/**
 * @brief Decode a single non-linear sRGB channel back to linear.
 */
template<typename T>
T decode_from_nonlinear_srgb_channel(T nonlinear) {
    nonlinear =  std::clamp(nonlinear, T{0}, T{1});
    if (nonlinear <= T{0.04045}) {
        return nonlinear / T{12.92};
    }
    return std::pow((nonlinear + T{0.055}) / T{1.055}, T{2.4});
}

/**
 * @brief Decode non-linear sRGB RGB into linear RGB.
 */
template<typename T>
RGB<T> decode_from_nonlinear_srgb(const RGB<T>& nonlinear_rgb) {
    return RGB<T>(
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.r()),
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.g()),
        decode_from_nonlinear_srgb_channel(nonlinear_rgb.b())
    );
}

} // namespace pbpt::radiometry
