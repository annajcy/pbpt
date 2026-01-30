#pragma once

#include <algorithm>
#include <optional>
#include "math/complex.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

template<typename T>
inline bool is_same_hemisphere(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wi) {
    return wo.z() * wi.z() > 0;
}

template<typename T>
inline math::Vector<T, 3> reflect(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& n) {
    return -wo + 2 * n * n.dot(wo);
}

template<typename T>
struct RefractResult {
    math::Vector<T, 3> wt;
    T etap;
};

template<typename T>
inline std::optional<RefractResult<T>> refract(const math::Vector<T, 3>& wo, math::Vector<T, 3> n, T eta) {
    T cos_theta_i = n.dot(wo);

    // flip normal if the incident direction is below the surface
    if (cos_theta_i < 0) {
        eta = 1 / eta;
        cos_theta_i = -cos_theta_i;
        n = -n;
    }

    // compute cosine of transmitted angle using Snell's law
    T sin2_theta_i = std::max(T(0), T(1) - cos_theta_i * cos_theta_i);
    T sin2_theta_t = sin2_theta_i / (eta * eta);

    if (sin2_theta_t >= 1) {
        // Total internal reflection
        return std::nullopt;
    }

    // compute transmitted direction
    T cos_theta_t = std::sqrt(1 - sin2_theta_t);
    math::Vector<T, 3> wt = -wo / eta + n * (cos_theta_i / eta - cos_theta_t);
    return RefractResult<T>{wt, eta};
}

template <typename T>
inline T fresnel_dielectric(T cos_theta_i, T eta) {
    cos_theta_i = std::clamp(cos_theta_i, T(-1), T(1));
    if (cos_theta_i < 0) {
        // Swap the indices of refraction
        eta = 1 / eta;
        cos_theta_i = -cos_theta_i;
    }

    // Compute cosine of transmitted angle for Fresnel equations using Snell’s law
    T sin2_theta_i = std::max(T(0), T(1) - cos_theta_i * cos_theta_i);
    T sin2_theta_t = sin2_theta_i / (eta * eta);
    if (sin2_theta_t >= 1) {
        // Total internal reflection
        return T(1);
    }
    T cos_theta_t = std::sqrt(1 - sin2_theta_t);

    T r_parl = ((eta * cos_theta_i) - cos_theta_t) /
                ((eta * cos_theta_i) + cos_theta_t);
    T r_perp = (cos_theta_i - (eta * cos_theta_t)) /
                (cos_theta_i + (eta * cos_theta_t));
    return (r_parl * r_parl + r_perp * r_perp) / T(2);
}

template <typename T>
inline T fresnel_conductor_per_wavelength(T cos_theta_i, const math::Complex<T>& eta) {
    cos_theta_i = std::clamp(cos_theta_i, T(0), T(1));
    // Compute complex cosine of transmitted angle using Snell's law
    T sin2_theta_i = std::max(T(0), T(1) - cos_theta_i * cos_theta_i);
    math::Complex<T> sin2_theta_t = sin2_theta_i / (eta * eta);
    math::Complex<T> cos_theta_t = (math::Complex<T>(1, 0) - sin2_theta_t).sqrt();

    math::Complex<T> r_parl = (eta * cos_theta_i - cos_theta_t) /
                              (eta * cos_theta_i + cos_theta_t);
    math::Complex<T> r_perp = (cos_theta_i - eta * cos_theta_t) /
                              (cos_theta_i + eta * cos_theta_t);
    return (r_perp.squared_length() + r_parl.squared_length()) / T(2);
}

template<typename T, int N>
radiometry::SampledSpectrum<T, N> fresnel_conductor(
    T cos_theta_i,
    const radiometry::SampledSpectrum<T, N>& eta,
    const radiometry::SampledSpectrum<T, N>& k
) {
    radiometry::SampledSpectrum<T, N> R;
    for (int i = 0; i < N; ++i) {
        math::Complex<T> eta_c(eta[i], k[i]);
        R[i] = fresnel_conductor_per_wavelength(cos_theta_i, eta_c);
    }
    return R;
}

template<typename T>
inline T fresenl_dielectric_schlick(T cos_theta_i, T eta) {
    cos_theta_i = std::clamp(cos_theta_i, T(-1), T(1));
    if (cos_theta_i < 0) {
        // Swap the indices of refraction
        eta = 1 / eta;
        cos_theta_i = -cos_theta_i;
    }
    T r0 = (eta - 1) / (eta + 1);
    r0 = r0 * r0;
    return r0 + (1 - r0) * std::pow((1 - cos_theta_i), 5);
}

template<typename T, int N>
radiometry::SampledSpectrum<T, N> fresnel_conductor_schlick(
    T cos_theta_i, T eta, 
    const radiometry::SampledSpectrum<T, N>& base_reflectance
) {
    radiometry::SampledSpectrum<T, N> R;
    for (int i = 0; i < N; ++i) {
        R[i] = base_reflectance[i] + (1 - base_reflectance[i]) * std::pow(1 - cos_theta_i, 5);
    }
    return R;
}

}  // namespace pbpt::material