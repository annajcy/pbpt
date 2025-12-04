/**
 * @file
 * @brief Strongly-typed radiometric quantities (radiance, flux, area, etc.).
 */
#pragma once

#include <concepts>
#include <string>
#include <sstream>
#include <iomanip>
#include <type_traits>

#include "math/operator.hpp"
#include "math/utils.hpp"
#include "math/function.hpp"
#include "math/vector.hpp"

using namespace pbpt::math;

namespace pbpt::radiometry {

/// Tag identifying radiance units (W / (m^2·sr)).
struct RadianceTag {};
/// Tag identifying irradiance units (W / m^2).
struct IrradianceTag {};
/// Tag identifying flux (power) units (W).
struct FluxTag {};
/// Tag identifying intensity units (W / sr).
struct IntensityTag {};
/// Tag identifying solid-angle units (sr).
struct SolidAngleTag {};
/// Tag identifying area units (m^2).
struct AreaTag {};
/// Tag identifying wavelength units (nm).
struct WaveLengthTag {};
/// Tag identifying temperature units (K).
struct TemperatureTag {};

/**
 * @brief Marks a radiometric unit as spectral (per wavelength).
 *
 * For example, `SpectralTag<RadianceTag>` describes spectral radiance.
 *
 * @tparam Tag Base (non-spectral) unit tag.
 */
template<typename Tag>
struct SpectralTag {
    /// Underlying non-spectral tag type.
    using OriginalTag = Tag;
};

/**
 * @brief Trait helper used to inspect unit tags.
 *
 * For plain tags `Tag`, `TagTrait<Tag>::is_spectral()` is false and
 * `tag_type` is `Tag` itself.
 *
 * @tparam Tag Unit tag.
 */
template<typename Tag>
struct TagTrait {
    /// Returns true if the tag represents a spectral quantity.
    static constexpr bool is_spectral() { return false; }
    /// The tag type itself.
    using tag_type = Tag;
};

/**
 * @brief TagTrait specialization for spectral quantities.
 *
 * For `SpectralTag<Tag>`, `is_spectral()` is true, `tag_type` is the
 * spectral tag itself and `original_tag_type` recovers the base tag.
 */
template<typename Tag>
struct TagTrait<SpectralTag<Tag>> {
    /// Returns true for all spectral tags.
    static constexpr bool is_spectral() { return true; }
    /// The spectral tag type (e.g. SpectralTag<RadianceTag>).
    using tag_type = SpectralTag<Tag>;
    /// The underlying non-spectral tag type.
    using original_tag_type = Tag;
};

/**
 * @brief Strongly-typed scalar radiometric quantity.
 *
 * Wraps a floating-point value with a tag that encodes its physical
 * dimension (e.g. RadianceTag, FluxTag). All arithmetic is type-safe:
 * only quantities with the same tag can be added or subtracted, and
 * cross-unit combinations use free functions (e.g. Radiance * SolidAngle).
 *
 * @tparam T   Floating-point scalar type.
 * @tparam Tag Unit tag (e.g. RadianceTag).
 */
template<std::floating_point T, typename Tag>
class RadiometryUnit {
protected:
    /// Stored scalar value in SI units for this quantity.
    T m_value;

public:
    /// Tag that encodes the physical dimension of this quantity.
    using unit_tag = Tag;

    /** @brief Default-constructs a zero-valued quantity. */
    explicit constexpr RadiometryUnit() : m_value(T{0}) {}
    /**
     * @brief Constructs a quantity from a raw scalar value.
     *
     * The value is assumed to be expressed in the corresponding SI unit
     * (for example watts, watts per square meter, etc., depending on @p Tag).
     */
    explicit constexpr RadiometryUnit(T v) : m_value(v) {}
    /// Returns the stored scalar value (read-only).
    constexpr const T& value() const { return m_value; }
    /// Returns the stored scalar value (modifiable).
    constexpr T& value() { return m_value; }
    
    /**
     * @brief Adds two quantities with the same unit tag.
     *
     * The result type promotes the scalar to a common floating-point type.
     */
    template<std::floating_point U>
    constexpr auto operator+(const RadiometryUnit<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) + static_cast<R>(other.value()));
    }
    
    /**
     * @brief Subtracts another quantity with the same unit tag.
     *
     * The result type promotes the scalar to a common floating-point type.
     */
    template<std::floating_point U>
    constexpr auto operator-(const RadiometryUnit<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) - static_cast<R>(other.value()));
    }
    
    /**
     * @brief Multiplies this quantity by a scalar.
     *
     * @tparam U Arithmetic scalar type.
     * @param scalar Scalar multiplier.
     */
    template<typename  U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator*(U scalar) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) * static_cast<R>(scalar));
    }
    
    /**
     * @brief Divides this quantity by a scalar.
     *
     * Asserts that @p scalar is non-zero.
     *
     * @tparam U Arithmetic scalar type.
     * @param scalar Scalar divisor.
     */
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator/(U scalar) const {
        using R = std::common_type_t<T, U>;
        assert_if(scalar == U{0}, "Division by zero");
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) / static_cast<R>(scalar));
    }
    
    /// Adds another quantity in-place.
    template<std::floating_point U>
    constexpr RadiometryUnit& operator+=(const RadiometryUnit<U, Tag>& other) {
        m_value += static_cast<T>(other.value());
        return *this;
    }
    
    /// Subtracts another quantity in-place.
    template<std::floating_point U>
    constexpr RadiometryUnit& operator-=(const RadiometryUnit<U, Tag>& other) {
        m_value -= static_cast<T>(other.value());
        return *this;
    }
    
    /// Multiplies this quantity by a scalar in-place.
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr RadiometryUnit& operator*=(U scalar) {
        m_value *= static_cast<T>(scalar);
        return *this;
    }
    
    /// Divides this quantity by a scalar in-place (asserting non-zero divisor).
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr RadiometryUnit& operator/=(U scalar) {
        assert_if(scalar == U{0}, "Division by zero");
        m_value /= static_cast<T>(scalar);
        return *this;
    }
    
    /**
     * @brief Equality comparison with type promotion.
     *
     * Uses the floating-point helpers in `math/utils.hpp` for robust comparison.
     */
    template<std::floating_point U>
    constexpr bool operator==(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    /// Inequality comparison.
    template<std::floating_point U>
    constexpr bool operator!=(const RadiometryUnit<U, Tag>& other) const {
        return !(*this == other);
    }
    
    /// Strict less-than comparison.
    template<std::floating_point U>
    constexpr bool operator<(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    /// Less-or-equal comparison.
    template<std::floating_point U>
    constexpr bool operator<=(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    /// Strict greater-than comparison.
    template<std::floating_point U>
    constexpr bool operator>(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    /// Greater-or-equal comparison.
    template<std::floating_point U>
    constexpr bool operator>=(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
};

/**
 * @brief Multiplies a radiometric quantity by a scalar from the left.
 *
 * This is symmetric with `quantity * scalar` and uses the same promotion rules.
 */
template<typename U, std::floating_point T, typename Tag>
requires std::is_arithmetic_v<U>
constexpr auto operator*(U scalar, const RadiometryUnit<T, Tag>& quantity) {
    return quantity * scalar;
}

/// Radiance (W / (m^2·sr)): power per unit area per unit solid angle.
template<std::floating_point T>
using Radiance = RadiometryUnit<T, RadianceTag>;

/// Spectral radiance (W / (m^2·sr·nm)): radiance per unit wavelength.
template<std::floating_point T>
using SpectralRadiance = RadiometryUnit<T, SpectralTag<RadianceTag>>;

/// Radiant intensity (W / sr): power per unit solid angle.
template<std::floating_point T>
using Intensity = RadiometryUnit<T, IntensityTag>;

/// Spectral radiant intensity (W / (sr·nm)).
template<std::floating_point T>
using SpectralIntensity = RadiometryUnit<T, SpectralTag<IntensityTag>>;

/// Irradiance (W / m^2): power incident per unit area.
template<std::floating_point T>
using Irradiance = RadiometryUnit<T, IrradianceTag>;

/// Spectral irradiance (W / (m^2·nm)).
template<std::floating_point T>
using SpectralIrradiance = RadiometryUnit<T, SpectralTag<IrradianceTag>>;

/// Radiant flux (W): total emitted, received or transferred power.
template<std::floating_point T>
using Flux = RadiometryUnit<T, FluxTag>;

/// Spectral radiant flux (W / nm).
template<std::floating_point T>
using SpectralFlux = RadiometryUnit<T, SpectralTag<FluxTag>>;

/// Geometric area (m^2).
template<std::floating_point T>
using Area = RadiometryUnit<T, AreaTag>;

/**
 * @brief Projects an area using a cosine factor.
 *
 * This is typically used when computing the projected area of a surface
 * relative to some direction: projected_area = area * cos(theta).
 *
 * @param area Surface area.
 * @param cos_theta Cosine of the angle between surface normal and direction.
 */
template<std::floating_point T>
inline Area<T> project_area_cos(const Area<T>& area, T cos_theta) {
    return Area<T>(area.value() * cos_theta);
}

/// Solid angle (sr).
template<std::floating_point T>
using SolidAngle = RadiometryUnit<T, SolidAngleTag>;

/**
 * @brief Solid angle element with an associated direction.
 *
 * This is useful for representing directional light sources where both
 * the solid angle and its central direction are needed.
 */
template<std::floating_point T>
struct DirectionalSolidAngle {
    /// Solid angle magnitude.
    SolidAngle<T> solid_angle;
    /// Direction associated with this solid angle (e.g. for a directional source).
    math::Vector<T, 3> direction;
};

/**
 * @brief Returns the solid angle of a hemisphere.
 *
 * The hemisphere has a solid angle of 2 * pi steradians.
 */
template<std::floating_point T>
inline constexpr SolidAngle<T> hemisphere_sr() {
    return SolidAngle<T>(T{2} * pi_v<T>);
}

/**
 * @brief Returns the solid angle of a full sphere.
 *
 * The sphere has a solid angle of 4 * pi steradians.
 */
template<std::floating_point T>
inline constexpr SolidAngle<T> sphere_sr() {
    return SolidAngle<T>(T{4} * pi_v<T>);
}

/// Wavelength (nm) represented as a radiometric unit.
template<std::floating_point T>
using WaveLength = RadiometryUnit<T, WaveLengthTag>;

/// Thermodynamic temperature (K).
template<std::floating_point T>
using Temperature = RadiometryUnit<T, TemperatureTag>;

/**
 * @brief Multiplies a spectral quantity by wavelength.
 *
 * This converts a per-wavelength quantity into the corresponding
 * non-spectral quantity (for example, W/(m^2·sr·nm) times nm gives W/(m^2·sr)).
 *
 * Only spectral tags are allowed; a runtime assertion checks this.
 */
template<typename T, typename Tag, typename U>
inline auto operator*(const RadiometryUnit<T, Tag>& quantity, WaveLength<U> lambda) {
    using R = std::common_type_t<T, U>;
    using tag_type = Tag;
    bool is_spectral = TagTrait<tag_type>::is_spectral();
    using original_tag_type = TagTrait<tag_type>::original_tag_type;
    assert_if(!is_spectral, "Only spectral units can be multiplied by wavelength");
    return RadiometryUnit<R, original_tag_type>(static_cast<R>(quantity.value()) * static_cast<R>(lambda.value()));
}

/**
 * @brief Multiplies wavelength by a spectral quantity (commutative form).
 *
 * See the documentation of the other overload for details.
 */
template<typename T, typename Tag, typename U>
inline auto operator*(WaveLength<U> lambda, const RadiometryUnit<T, Tag>& quantity) {
    using R = std::common_type_t<T, U>;
    using tag_type = Tag;
    bool is_spectral = TagTrait<tag_type>::is_spectral();
    using original_tag_type = TagTrait<tag_type>::original_tag_type;
    assert_if(!is_spectral, "Only spectral units can be multiplied by wavelength");
    return RadiometryUnit<R, original_tag_type>(static_cast<R>(quantity.value()) * static_cast<R>(lambda.value()));
}

// Physics operators with type promotion

/// Flux = Intensity * SolidAngle.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Intensity<T1>& I, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(I.value()) * static_cast<R>(w.value()));
}

/// Flux = SolidAngle * Intensity (commutative form).
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const SolidAngle<T1>& w, const Intensity<U>& I) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(I.value()) * static_cast<R>(w.value()));
}

/// Flux = Irradiance * Area.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Irradiance<T1>& E, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(E.value()) * static_cast<R>(a.value()));
}

/// Flux = Area * Irradiance (commutative form).
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Area<T1>& a, const Irradiance<U>& E) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(E.value()) * static_cast<R>(a.value()));
}

/// Irradiance = Flux / Area.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Flux<T1>& phi, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(a.value()), "Cannot divide by zero area");
    return Irradiance<R>(static_cast<R>(phi.value()) / static_cast<R>(a.value()));
}

/// Irradiance = Radiance * SolidAngle.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Radiance<T1>& L, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    return Irradiance<R>(static_cast<R>(L.value()) * static_cast<R>(w.value()));
}

/// Irradiance = SolidAngle * Radiance (commutative form).
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const SolidAngle<T1>& w, const Radiance<U>& L) {
    using R = std::common_type_t<T1, U>;
    return Irradiance<R>(static_cast<R>(L.value()) * static_cast<R>(w.value()));
}

/// Intensity = Flux / SolidAngle.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Flux<T1>& phi, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(w.value()), "Cannot divide by zero solid angle");
    return Intensity<R>(static_cast<R>(phi.value()) / static_cast<R>(w.value()));
}

/// Intensity = Radiance * Area.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Radiance<T1>& L, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    return Intensity<R>(static_cast<R>(L.value()) * static_cast<R>(a.value()));
}

/// Intensity = Area * Radiance (commutative form).
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Area<T1>& a, const Radiance<U>& L) {
    using R = std::common_type_t<T1, U>;
    return Intensity<R>(static_cast<R>(L.value()) * static_cast<R>(a.value()));
}

/// Radiance = Irradiance / SolidAngle.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Irradiance<T1>& E, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(w.value()), "Cannot divide by zero solid angle");
    return Radiance<R>(static_cast<R>(E.value()) / static_cast<R>(w.value()));
}

/// Radiance = Intensity / Area.
template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Intensity<T1>& I, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(a.value()), "Cannot divide by zero area");
    return Radiance<R>(static_cast<R>(I.value()) / static_cast<R>(a.value()));
}

/**
 * @brief Describes how a quantity with a given tag should be printed.
 *
 * Specializations provide a human-readable unit symbol for each tag.
 *
 * @tparam Tag Unit tag (e.g. RadianceTag).
 */
template<typename Tag> 
struct RadiometryUnitInfo {
    /// Human-readable unit symbol (e.g. "W/(m^2·sr)").
    static constexpr const char* symbol = "unknown";
};

/// Radiance unit info (W / (m^2·sr)).
template<> 
struct RadiometryUnitInfo<RadianceTag> {
    /// Symbol for radiance: watts per square meter per steradian.
    static constexpr const char* symbol = "w/(m²·sr)";
};

/// Radiant flux unit info (W).
template<> 
struct RadiometryUnitInfo<FluxTag> {
    /// Symbol for radiant flux (power): watts.
    static constexpr const char* symbol = "w";
};

/// Radiant intensity unit info (W / sr).
template<> 
struct RadiometryUnitInfo<IntensityTag> {
    /// Symbol for radiant intensity: watts per steradian.
    static constexpr const char* symbol = "w/sr";
};

/// Irradiance unit info (W / m^2).
template<> 
struct RadiometryUnitInfo<IrradianceTag> {
    /// Symbol for irradiance: watts per square meter.
    static constexpr const char* symbol = "w/m²";
};

/// Solid angle unit info (sr).
template<> 
struct RadiometryUnitInfo<SolidAngleTag> {
    /// Symbol for solid angle: steradian.
    static inline const std::string symbol = "sr";
};

/// Area unit info (m^2).
template<> 
struct RadiometryUnitInfo<AreaTag> {
    /// Symbol for area: square meters.
    static inline const std::string symbol = "m²";
};

/// Wavelength unit info (nm).
template<> 
struct RadiometryUnitInfo<WaveLengthTag> {
    /// Symbol for wavelength: nanometers.
    static inline const std::string  symbol = "nm";
};

/// Temperature unit info (K).
template<> 
struct RadiometryUnitInfo<TemperatureTag> {
    /// Symbol for temperature: kelvin.
    static inline const std::string  symbol = "k";
};

/**
 * @brief Unit info for spectral quantities.
 *
 * The symbol is constructed by appending "\/nm" to the underlying
 * non-spectral unit symbol (for example "W/m²" becomes "W/m²/nm").
 */
template<typename Tag> 
struct RadiometryUnitInfo<SpectralTag<Tag>> {
    /// Symbol for spectral quantity: base symbol per nanometer.
    static inline const std::string symbol = std::string(RadiometryUnitInfo<Tag>::symbol) + "/nm";
};

/**
 * @brief Formats a radiometric quantity as a string with its unit.
 *
 * The numeric value is printed with 6 decimal places followed by the
 * unit symbol from RadiometryUnitInfo.
 */
template<typename T, typename Tag>
std::string to_string(const RadiometryUnit<T, Tag>& q) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << q.value() << " " << RadiometryUnitInfo<Tag>::symbol;
    return oss.str();
}

/**
 * @brief Streams a radiometric quantity to an output stream.
 *
 * This uses the same formatting as `to_string`.
 */
template<typename T, typename Tag>
std::ostream & operator<<(std::ostream& os, const RadiometryUnit<T, Tag>& q) {
    return os << to_string(q);
}

}
