#pragma once

#include <concepts>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <type_traits>
#include "math/operator.hpp"
#include "math/utils.hpp"
#include "math/function.hpp"

using namespace pbpt::math;
namespace pbpt::core {

struct RadianceTag {};
struct FluxTag {};
struct IntensityTag {};
struct SolidAngleTag {};
struct AreaTag {};
struct IrradianceTag {};

template<std::floating_point T, typename Tag>
class Radiometry {
protected:
    T m_value;

public:
    // Construction
    explicit constexpr Radiometry(T v = T{0}) : m_value(v) {}
    constexpr const T& value() const { return m_value; }
    constexpr T& value() { return m_value; }
    
    // typename operators with type promotion
    template<std::floating_point U>
    constexpr auto operator+(const Radiometry<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return Radiometry<R, Tag>(static_cast<R>(m_value) + static_cast<R>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr auto operator-(const Radiometry<U, Tag>& other) const {
        using R = std::common_type_t<T, U>;
        return Radiometry<R, Tag>(static_cast<R>(m_value) - static_cast<R>(other.value()));
    }
    
    template<typename  U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator*(U scalar) const {
        using R = std::common_type_t<T, U>;
        return Radiometry<R, Tag>(static_cast<R>(m_value) * static_cast<R>(scalar));
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr auto operator/(U scalar) const {
        using R = std::common_type_t<T, U>;
        assert_if(scalar == U{0}, "Division by zero");
        return Radiometry<R, Tag>(static_cast<R>(m_value) / static_cast<R>(scalar));
    }
    
    // Assignment operators with type promotion
    template<std::floating_point U>
    constexpr Radiometry& operator+=(const Radiometry<U, Tag>& other) {
        m_value += static_cast<T>(other.value());
        return *this;
    }
    
    template<std::floating_point U>
    constexpr Radiometry& operator-=(const Radiometry<U, Tag>& other) {
        m_value -= static_cast<T>(other.value());
        return *this;
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr Radiometry& operator*=(U scalar) {
        m_value *= static_cast<T>(scalar);
        return *this;
    }
    
    template<typename U>
    requires std::is_arithmetic_v<U>
    constexpr Radiometry& operator/=(U scalar) {
        assert_if(scalar == U{0}, "Division by zero");
        m_value /= static_cast<T>(scalar);
        return *this;
    }
    
    // Comparison operators with type promotion
    template<std::floating_point U>
    constexpr bool operator==(const Radiometry<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator!=(const Radiometry<U, Tag>& other) const {
        return !(*this == other);
    }
    
    template<std::floating_point U>
    constexpr bool operator<(const Radiometry<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator<=(const Radiometry<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_less_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator>(const Radiometry<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
    
    template<std::floating_point U>
    constexpr bool operator>=(const Radiometry<U, Tag>& other) const {
        using CommonType = std::common_type_t<T, U>;
        return is_greater_equal(static_cast<CommonType>(m_value), static_cast<CommonType>(other.value()));
    }
};

// Global operators with type promotion
template<typename U, std::floating_point T, typename Tag>
requires std::is_arithmetic_v<U>
constexpr auto operator*(U scalar, const Radiometry<T, Tag>& quantity) {
    return quantity * scalar;
}

template<std::floating_point T>
using Radiance = Radiometry<T, RadianceTag>;

template<std::floating_point T>
using Intensity = Radiometry<T, IntensityTag>;

template<std::floating_point T>
using Irradiance = Radiometry<T, IrradianceTag>;

template<std::floating_point T>
using Flux = Radiometry<T, FluxTag>;

template<std::floating_point T>
using Area = Radiometry<T, AreaTag>;

template<std::floating_point T>
inline Area<T> project_area_cos(const Area<T>& area, T cos_theta) {
    return Area<T>(area.value() * cos_theta);
}

template<std::floating_point T>
using SolidAngle = Radiometry<T, SolidAngleTag>;

template<std::floating_point T>
inline constexpr SolidAngle<T> hemisphere_sr() {
    return SolidAngle<T>(T{2} * pi_v<T>);
}

template<std::floating_point T>
inline constexpr SolidAngle<T> sphere_sr() {
    return SolidAngle<T>(T{4} * pi_v<T>);
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

// =============================================================================
// Dimensional Analysis Unit Traits
// =============================================================================

template<typename Tag> 
struct RadiometryUnitInfo {
    static constexpr const char* symbol = "unknown";
    static constexpr int dim_work = 1;
    static constexpr int dim_length = -2;      
    static constexpr int dim_solid_angle = -1; 
};

template<> 
struct RadiometryUnitInfo<RadianceTag> {
    static constexpr const char* symbol = "W/(m²·sr)";
    static constexpr int dim_work = 1;
    static constexpr int dim_length = -2;      
    static constexpr int dim_solid_angle = -1; 
};

template<> 
struct RadiometryUnitInfo<FluxTag> {
    static constexpr const char* symbol = "W";
    static constexpr int dim_work = 1;
    static constexpr int dim_length = 0;      
    static constexpr int dim_solid_angle = 0; 
};

template<> 
struct RadiometryUnitInfo<IntensityTag> {
    static constexpr const char* symbol = "W/sr";
    static constexpr int dim_work = 1;
    static constexpr int dim_length = 0;      
    static constexpr int dim_solid_angle = -1; 
};

template<> 
struct RadiometryUnitInfo<IrradianceTag> {
    static constexpr const char* symbol = "W/m²";
    static constexpr int dim_work = 1;       
    static constexpr int dim_length = -2;      
    static constexpr int dim_solid_angle = 0;
};

template<> 
struct RadiometryUnitInfo<SolidAngleTag> {
    static constexpr const char* symbol = "sr";
    static constexpr int dim_work = 0;       
    static constexpr int dim_length = 0;      
    static constexpr int dim_solid_angle = -1;
};

template<> 
struct RadiometryUnitInfo<AreaTag> {
    static constexpr const char* symbol = "m²";
    static constexpr int dim_work = 0;
    static constexpr int dim_length = 2; 
    static constexpr int dim_solid_angle = 0;
};

template<typename T, typename Tag>
std::string to_string(const Radiometry<T, Tag>& q) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << q.value() << " " << RadiometryUnitInfo<Tag>::symbol;
    return oss.str();
}

template<typename T, typename Tag>
std::ostream & operator<<(std::ostream& os, const Radiometry<T, Tag>& q) {
    return os << to_string(q);
}

}
