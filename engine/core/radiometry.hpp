#pragma once

#include <concepts>
#include <cassert>

namespace pbpt::core {

template<std::floating_point T>
class Radiance;

template<std::floating_point T>
class Flux;

template<std::floating_point T>
class SolidAngle;

template<std::floating_point T>
class ProjectedArea;

// ----------------------------
// Radiance: W / (m² · sr)
// ----------------------------
template<std::floating_point T>
class Radiance {
    T m_value;

    

public:
    explicit constexpr Radiance(T v) : m_value(v) {}
    constexpr T value() const { return m_value; }
};

// ----------------------------
// SolidAngle: sr
// ----------------------------
template<std::floating_point T>
class SolidAngle {
    T m_value;

public:
    explicit constexpr SolidAngle(T v) : m_value(v) {}
    constexpr T value() const { return m_value; }
};

// ----------------------------
// ProjectedArea: m²
// ----------------------------
template<std::floating_point T>
class ProjectedArea {
    T m_value;

public:
    explicit constexpr ProjectedArea(T v) : m_value(v) {}
    constexpr T value() const { return m_value; }
};

// ----------------------------
// Flux: W = Radiance × SolidAngle × ProjectedArea
// ----------------------------
template<std::floating_point T>
class Flux {
    T m_value;

public:
    explicit constexpr Flux(T v) : m_value(v) {}
    constexpr T value() const { return m_value; }
};

// ----------------------------
// Multiplication: Radiance × SolidAngle × ProjectedArea → Flux
// ----------------------------
template<std::floating_point T>
constexpr Flux<T> operator*(const Radiance<T>& L, const SolidAngle<T>& omega) {
    return Flux<T>(L.value() * omega.value());
}

template<std::floating_point T>
constexpr Flux<T> operator*(const Radiance<T>& L, const ProjectedArea<T>& area) {
    return Flux<T>(L.value() * area.value()); // simplified
}

} // namespace
