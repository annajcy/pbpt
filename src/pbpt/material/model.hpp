#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include "pbpt/geometry/spherical.hpp"
#include "pbpt/math/function.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/sampler/2d.hpp"

namespace pbpt::material {

enum class MicrofacetDistribution { Beckmann, GGX };

template <typename T>
class MicrofacetModel {
private:
    T m_alpha_x;
    T m_alpha_y;
    MicrofacetDistribution m_distribution;

public:
    static T roughness_to_alpha(T roughness) { return std::sqrt(roughness); }

    static T alpha_to_roughness(T alpha) { return alpha * alpha; }

    static MicrofacetModel<T> from_roughness(T roughness_x, T roughness_y,
                                             MicrofacetDistribution distribution = MicrofacetDistribution::Beckmann) {
        T alpha_x = roughness_to_alpha(roughness_x);
        T alpha_y = roughness_to_alpha(roughness_y);
        return MicrofacetModel<T>(alpha_x, alpha_y, distribution);
    }

    static MicrofacetModel<T> from_alphas(T alpha_x, T alpha_y,
                                          MicrofacetDistribution distribution = MicrofacetDistribution::Beckmann) {
        return MicrofacetModel<T>(alpha_x, alpha_y, distribution);
    }

public:
    MicrofacetModel(T alpha_x, T alpha_y, MicrofacetDistribution distribution = MicrofacetDistribution::Beckmann)
        : m_alpha_x(alpha_x), m_alpha_y(alpha_y), m_distribution(distribution) {}

    const T& alpha_x() const { return m_alpha_x; }
    const T& alpha_y() const { return m_alpha_y; }
    MicrofacetDistribution distribution() const { return m_distribution; }
    T& alpha_x() { return m_alpha_x; }
    T& alpha_y() { return m_alpha_y; }

    T D(const math::Vector<T, 3>& wm) const {
        auto tan2_theta = geometry::tan2_theta(wm);
        if (std::isinf(tan2_theta)) {
            return T(0);
        }

        const T cos4_theta = geometry::cos2_theta(wm) * geometry::cos2_theta(wm);
        const auto px = (geometry::cos_phi(wm) / m_alpha_x);
        const auto py = (geometry::sin_phi(wm) / m_alpha_y);
        const T e = tan2_theta * (px * px + py * py);

        if (m_distribution == MicrofacetDistribution::Beckmann) {
            return std::exp(-e) / (math::pi_v<T> * m_alpha_x * m_alpha_y * cos4_theta);
        }
        return T(1) / (math::pi_v<T> * m_alpha_x * m_alpha_y * cos4_theta * (T(1) + e) * (T(1) + e));
    }

    T lambda(const math::Vector<T, 3>& w) const {
        if (m_distribution == MicrofacetDistribution::Beckmann) {
            const T abs_tan_theta = std::abs(geometry::tan_theta(w));
            if (std::isinf(abs_tan_theta)) {
                return T(0);
            }

            const T alpha_x2 = (geometry::cos_phi(w) * m_alpha_x) * (geometry::cos_phi(w) * m_alpha_x);
            const T alpha_y2 = (geometry::sin_phi(w) * m_alpha_y) * (geometry::sin_phi(w) * m_alpha_y);
            const T alpha = std::sqrt(alpha_x2 + alpha_y2);
            if (alpha == T(0) || abs_tan_theta == T(0)) {
                return T(0);
            }

            const T a = T(1) / (alpha * abs_tan_theta);
            if (a >= T(1.6)) {
                return T(0);
            }
            return (T(1) - T(1.259) * a + T(0.396) * a * a) / (T(3.535) * a + T(2.181) * a * a);
        }

        const T tan2_theta = geometry::tan2_theta(w);
        if (std::isinf(tan2_theta)) {
            return T(0);
        }

        const T a2 = (geometry::cos_phi(w) * m_alpha_x) * (geometry::cos_phi(w) * m_alpha_x) +
                     (geometry::sin_phi(w) * m_alpha_y) * (geometry::sin_phi(w) * m_alpha_y);
        return (-1 + std::sqrt(1 + tan2_theta * a2)) / T(2);
    }

    T G1(const math::Vector<T, 3>& w) const { return T(1) / (T(1) + lambda(w)); }

    T G(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wi) const {
        return T(1) / ((T(1) + lambda(wo)) * (T(1) + lambda(wi)));
    }

    T D_visible(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wm) const {
        const T cos_theta_o = geometry::cos_theta(wo);
        const T abs_cos_theta_o = std::abs(cos_theta_o);
        if (abs_cos_theta_o == T(0)) {
            return T(0);
        }

        const T dot_wo_wm = wo.dot(wm);
        const T same_side_dot = (cos_theta_o >= T(0)) ? dot_wo_wm : -dot_wo_wm;
        return D(wm) * G1(wo) * std::max(T(0), same_side_dot) / abs_cos_theta_o;
    }

    T pdf_wm(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wm) const { return D_visible(wo, wm); }

    math::Vector<T, 3> sample_wm(const math::Vector<T, 3>& wo, const math::Point<T, 2>& u2d) const {
        if (m_distribution == MicrofacetDistribution::Beckmann) {
            return sample_wm_beckmann(wo, u2d);
        }
        return sample_wm_ggx(wo, u2d);
    }

    bool is_delta() const { return std::max(m_alpha_x, m_alpha_y) < T(1e-3); }

private:
    math::Vector<T, 3> sample_wm_ggx(const math::Vector<T, 3>& wo, const math::Point<T, 2>& u2d) const {
        // transform the view direction to the hemisphere configuration
        auto wh = math::Vector<T, 3>(m_alpha_x * wo.x(), m_alpha_y * wo.y(), wo.z()).normalized();
        if (wh.z() < 0) {
            wh = -wh;
        }

        // Section 1: Construct PBRT-style Frame
        // T1 is forced to be perpendicular to macroscopic normal (0, 0, 1) and wh
        math::Vector<T, 3> T1 = (wh.z() < T(0.99999)) ? math::cross(math::Vector<T, 3>(0, 0, 1), wh).normalized()
                                                      : math::Vector<T, 3>(1, 0, 0);
        math::Vector<T, 3> T2 = math::cross(wh, T1);

        // Section 2: Sample uniform disk
        auto p = sampler::sample_uniform_disk_concentric(u2d);

        // Section 3: Apply Heitz's clipping mapping
        T h = std::sqrt(std::max(T(0), T(1) - p.x() * p.x()));
        T t = (T(1) + wh.z()) / T(2);
        T p_y = (T(1) - t) * h + t * p.y();

        // Section 4: Unproject to hemisphere
        T pz = std::sqrt(std::max(T(0), T(1) - p.x() * p.x() - p_y * p_y));
        auto nh = p.x() * T1 + p_y * T2 + pz * wh;

        // transform back to the ellipsoid configuration
        auto wm = math::Vector<T, 3>(m_alpha_x * nh.x(), m_alpha_y * nh.y(), std::max(T(0), nh.z())).normalized();
        return wm;
    }

    static std::pair<T, T> beckmann_sample11(T cos_theta_i, T u1, T u2) {
        constexpr T one_minus_eps = T(1) - T(1e-6);
        cos_theta_i = std::clamp(cos_theta_i, T(1e-6), one_minus_eps);
        u1 = std::clamp(u1, T(1e-6), one_minus_eps);
        u2 = std::clamp(u2, T(1e-6), one_minus_eps);

        if (cos_theta_i > T(0.9999)) {
            const T r = std::sqrt(-std::log(T(1) - u1));
            const T phi = T(2) * math::pi_v<T> * u2;
            return {r * std::cos(phi), r * std::sin(phi)};
        }

        const T sin_theta_i = std::sqrt(std::max(T(0), T(1) - cos_theta_i * cos_theta_i));
        const T tan_theta_i = sin_theta_i / cos_theta_i;
        const T cot_theta_i = T(1) / tan_theta_i;
        const T theta_i = std::acos(cos_theta_i);
        const T fit = T(1) + theta_i * (T(-0.876) + theta_i * (T(0.4265) - T(0.0594) * theta_i));
        const T c = std::erf(cot_theta_i);
        const T inv_sqrt_pi = T(1) / std::sqrt(math::pi_v<T>);

        T a = T(-1);
        T b = c - (T(1) + c) * std::pow(T(1) - u1, fit);
        T c_bracket = c;
        const T normalization =
            T(1) / (T(1) + c + inv_sqrt_pi * tan_theta_i * std::exp(-cot_theta_i * cot_theta_i));

        for (int i = 0; i < 10; ++i) {
            if (b <= a || b >= c_bracket) {
                b = T(0.5) * (a + c_bracket);
            }

            const T inv_erf = math::erf_inv(b);
            const T value =
                normalization * (T(1) + b + inv_sqrt_pi * tan_theta_i * std::exp(-inv_erf * inv_erf)) - u1;
            const T deriv = normalization * (T(1) - inv_erf * tan_theta_i);

            if (value > T(0)) {
                c_bracket = b;
            } else {
                a = b;
            }

            if (deriv == T(0)) {
                b = T(0.5) * (a + c_bracket);
                continue;
            }
            b -= value / deriv;
        }

        const T slope_x = math::erf_inv(b);
        const T slope_y = math::erf_inv(T(2) * u2 - T(1));
        return {slope_x, slope_y};
    }

    math::Vector<T, 3> sample_wm_beckmann(const math::Vector<T, 3>& wo, const math::Point<T, 2>& u2d) const {
        const auto wo_abs = (wo.z() < T(0)) ? -wo : wo;
        const auto wi = math::Vector<T, 3>(m_alpha_x * wo_abs.x(), m_alpha_y * wo_abs.y(), wo_abs.z()).normalized();

        auto [slope_x, slope_y] = beckmann_sample11(geometry::cos_theta(wi), u2d.x(), u2d.y());

        const T sin_phi = geometry::sin_phi(wi);
        const T cos_phi = geometry::cos_phi(wi);
        const T slope_x_rot = cos_phi * slope_x - sin_phi * slope_y;
        const T slope_y_rot = sin_phi * slope_x + cos_phi * slope_y;

        slope_x = m_alpha_x * slope_x_rot;
        slope_y = m_alpha_y * slope_y_rot;

        auto wm = math::Vector<T, 3>(-slope_x, -slope_y, T(1)).normalized();
        if (wm.z() < T(0)) {
            wm = -wm;
        }
        return wm;
    }
};

}  // namespace pbpt::material
