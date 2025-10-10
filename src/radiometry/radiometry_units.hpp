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

struct RadianceTag {};
struct IrradianceTag {};
struct FluxTag {};
struct IntensityTag {};
struct SolidAngleTag {};
struct AreaTag {};
struct WaveLengthTag {};
struct TemperatureTag {};

template<typename Tag>
struct SpectralTag {
    using OriginalTag = Tag;
};

// 基础 TagTrait 模板声明
template<typename Tag>
struct TagTrait {
    static constexpr bool is_spectral() { return false; }
    using tag_type = Tag;
};

// SpectralTag 的特化 - 用于识别光谱标签
template<typename Tag>
struct TagTrait<SpectralTag<Tag>> {
    static constexpr bool is_spectral() { return true; }
    using tag_type = SpectralTag<Tag>;
    using original_tag_type = Tag;
};

template<std::floating_point T, typename Tag>
class RadiometryUnit {
protected:
    T m_value;

public:
    using unit_tag = Tag;

    // Construction
    explicit constexpr RadiometryUnit() : m_value(T{0}) {}
    explicit constexpr RadiometryUnit(T v) : m_value(v) {}
    constexpr const T& value() const { return m_value; }
    constexpr T& value() { return m_value; }
    
    // typename operators with type promotion
    template<std::floating_point U>
    constexpr auto operator+(const RadiometryUnit<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) + static_cast<R>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr auto operator-(const RadiometryUnit<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) - static_cast<R>(other.value()));
    }
    
    template<typename  U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator*(U scalar) const {
        using R = std::common_type_t<T, U>;
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) * static_cast<R>(scalar));
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator/(U scalar) const {
        using R = std::common_type_t<T, U>;
        assert_if(scalar == U{0}, "Division by zero");
        return RadiometryUnit<R, Tag>(static_cast<R>(m_value) / static_cast<R>(scalar));
    }
    
    // Assignment operators with type promotion
    template<std::floating_point U>
    constexpr RadiometryUnit& operator+=(const RadiometryUnit<U, Tag>& other) {
        m_value += static_cast<T>(other.value());
        return *this;
    }
    
    template<std::floating_point U>
    constexpr RadiometryUnit& operator-=(const RadiometryUnit<U, Tag>& other) {
        m_value -= static_cast<T>(other.value());
        return *this;
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr RadiometryUnit& operator*=(U scalar) {
        m_value *= static_cast<T>(scalar);
        return *this;
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr RadiometryUnit& operator/=(U scalar) {
        assert_if(scalar == U{0}, "Division by zero");
        m_value /= static_cast<T>(scalar);
        return *this;
    }
    
    // Comparison operators with type promotion
    template<std::floating_point U>
    constexpr bool operator==(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator!=(const RadiometryUnit<U, Tag>& other) const {
        return !(*this == other);
    }
    
    template<std::floating_point U>
    constexpr bool operator<(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator<=(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator>(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator>=(const RadiometryUnit<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
};

// Global operators with type promotion
template<typename U, std::floating_point T, typename Tag>
requires std::is_arithmetic_v<U>
constexpr auto operator*(U scalar, const RadiometryUnit<T, Tag>& quantity) {
    return quantity * scalar;
}

template<std::floating_point T>
using Radiance = RadiometryUnit<T, RadianceTag>;

template<std::floating_point T>
using SpectralRadiance = RadiometryUnit<T, SpectralTag<RadianceTag>>;

template<std::floating_point T>
using Intensity = RadiometryUnit<T, IntensityTag>;

template<std::floating_point T>
using SpectralIntensity = RadiometryUnit<T, SpectralTag<IntensityTag>>;

template<std::floating_point T>
using Irradiance = RadiometryUnit<T, IrradianceTag>;

template<std::floating_point T>
using SpectralIrradiance = RadiometryUnit<T, SpectralTag<IrradianceTag>>;

template<std::floating_point T>
using Flux = RadiometryUnit<T, FluxTag>;

template<std::floating_point T>
using SpectralFlux = RadiometryUnit<T, SpectralTag<FluxTag>>;

template<std::floating_point T>
using Area = RadiometryUnit<T, AreaTag>;

template<std::floating_point T>
inline Area<T> project_area_cos(const Area<T>& area, T cos_theta) {
    return Area<T>(area.value() * cos_theta);
}

template<std::floating_point T>
using SolidAngle = RadiometryUnit<T, SolidAngleTag>;

template<std::floating_point T>
struct DirectionalSolidAngle {
    SolidAngle<T> solid_angle;
    math::Vector<T, 3> direction;
};

template<std::floating_point T>
inline constexpr SolidAngle<T> hemisphere_sr() {
    return SolidAngle<T>(T{2} * pi_v<T>);
}

template<std::floating_point T>
inline constexpr SolidAngle<T> sphere_sr() {
    return SolidAngle<T>(T{4} * pi_v<T>);
}

template<std::floating_point T>
using WaveLength = RadiometryUnit<T, WaveLengthTag>;

template<std::floating_point T>
using Temperature = RadiometryUnit<T, TemperatureTag>;

template<typename T, typename Tag, typename U>
inline auto operator*(const RadiometryUnit<T, Tag>& quantity, WaveLength<U> lambda) {
    using R = std::common_type_t<T, U>;
    using tag_type = Tag;
    bool is_spectral = TagTrait<tag_type>::is_spectral();
    using original_tag_type = TagTrait<tag_type>::original_tag_type;
    assert_if(!is_spectral, "Only spectral units can be multiplied by wavelength");
    return RadiometryUnit<R, original_tag_type>(static_cast<R>(quantity.value()) * static_cast<R>(lambda.value()));
}

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
template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Intensity<T1>& I, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(I.value()) * static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const SolidAngle<T1>& w, const Intensity<U>& I) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(I.value()) * static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Irradiance<T1>& E, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(E.value()) * static_cast<R>(a.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Area<T1>& a, const Irradiance<U>& E) {
    using R = std::common_type_t<T1, U>;
    return Flux<R>(static_cast<R>(E.value()) * static_cast<R>(a.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Flux<T1>& phi, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(a.value()), "Cannot divide by zero area");
    return Irradiance<R>(static_cast<R>(phi.value()) / static_cast<R>(a.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Radiance<T1>& L, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    return Irradiance<R>(static_cast<R>(L.value()) * static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const SolidAngle<T1>& w, const Radiance<U>& L) {
    using R = std::common_type_t<T1, U>;
    return Irradiance<R>(static_cast<R>(L.value()) * static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Flux<T1>& phi, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(w.value()), "Cannot divide by zero solid angle");
    return Intensity<R>(static_cast<R>(phi.value()) / static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Radiance<T1>& L, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    return Intensity<R>(static_cast<R>(L.value()) * static_cast<R>(a.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator*(const Area<T1>& a, const Radiance<U>& L) {
    using R = std::common_type_t<T1, U>;
    return Intensity<R>(static_cast<R>(L.value()) * static_cast<R>(a.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Irradiance<T1>& E, const SolidAngle<U>& w) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(w.value()), "Cannot divide by zero solid angle");
    return Radiance<R>(static_cast<R>(E.value()) / static_cast<R>(w.value()));
}

template<std::floating_point T1, std::floating_point U>
constexpr auto operator/(const Intensity<T1>& I, const Area<U>& a) {
    using R = std::common_type_t<T1, U>;
    assert_if(is_zero(a.value()), "Cannot divide by zero area");
    return Radiance<R>(static_cast<R>(I.value()) / static_cast<R>(a.value()));
}

template<typename Tag> 
struct RadiometryUnitInfo {
    static constexpr const char* symbol = "unknown";
};

template<> 
struct RadiometryUnitInfo<RadianceTag> {
    static constexpr const char* symbol = "w/(m²·sr)";
};

template<> 
struct RadiometryUnitInfo<FluxTag> {
    static constexpr const char* symbol = "w";
};

template<> 
struct RadiometryUnitInfo<IntensityTag> {
    static constexpr const char* symbol = "w/sr";
};

template<> 
struct RadiometryUnitInfo<IrradianceTag> {
    static constexpr const char* symbol = "w/m²";
};

template<> 
struct RadiometryUnitInfo<SolidAngleTag> {
    static inline const std::string symbol = "sr";
};

template<> 
struct RadiometryUnitInfo<AreaTag> {
    static inline const std::string symbol = "m²";
};

template<> 
struct RadiometryUnitInfo<WaveLengthTag> {
    static inline const std::string  symbol = "nm";
};

template<> 
struct RadiometryUnitInfo<TemperatureTag> {
    static inline const std::string  symbol = "k";
};

template<typename Tag> 
struct RadiometryUnitInfo<SpectralTag<Tag>> {
    static inline const std::string symbol = std::string(RadiometryUnitInfo<Tag>::symbol) + "/nm";
};

template<typename T, typename Tag>
std::string to_string(const RadiometryUnit<T, Tag>& q) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << q.value() << " " << RadiometryUnitInfo<Tag>::symbol;
    return oss.str();
}

template<typename T, typename Tag>
std::ostream & operator<<(std::ostream& os, const RadiometryUnit<T, Tag>& q) {
    return os << to_string(q);
}

}
