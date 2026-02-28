#pragma once

#include <algorithm>
#include <cmath>
#include "pbpt/geometry/spherical.hpp"
#include "pbpt/math/function.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/sampler/2d.hpp"

namespace pbpt::material {

template <typename T>
class MicrofacetModel {
private:
    T m_alpha_x;
    T m_alpha_y;

public:
    static T roughness_to_alpha(T roughness) { return std::sqrt(roughness); }

    static T alpha_to_roughness(T alpha) { return alpha * alpha; }

    static MicrofacetModel<T> from_roughness(T roughness_x, T roughness_y) {
        T alpha_x = roughness_to_alpha(roughness_x);
        T alpha_y = roughness_to_alpha(roughness_y);
        return MicrofacetModel<T>(alpha_x, alpha_y);
    }

    static MicrofacetModel<T> from_alphas(T alpha_x, T alpha_y) { return MicrofacetModel<T>(alpha_x, alpha_y); }

public:
    MicrofacetModel(T alpha_x, T alpha_y) : m_alpha_x(alpha_x), m_alpha_y(alpha_y) {}

    const T& alpha_x() const { return m_alpha_x; }
    const T& alpha_y() const { return m_alpha_y; }
    T& alpha_x() { return m_alpha_x; }
    T& alpha_y() { return m_alpha_y; }

    T D(const math::Vector<T, 3>& wm) const {
        auto tan2_theta = geometry::tan2_theta(wm);
        if (std::isinf(tan2_theta)) {
            return T(0);
        }

        T cos4_theta = geometry::cos2_theta(wm) * geometry::cos2_theta(wm);
        auto px = (geometry::cos_phi(wm) / m_alpha_x);
        auto py = (geometry::sin_phi(wm) / m_alpha_y);
        T e = tan2_theta * (px * px + py * py);
        return 1 / (math::pi_v<T> * m_alpha_x * m_alpha_y * cos4_theta * (1 + e) * (1 + e));
    }

    T lambda(const math::Vector<T, 3>& w) const {
        T tan2_theta = geometry::tan2_theta(w);
        if (std::isinf(tan2_theta)) {
            return T(0);
        }
        T a2 = (geometry::cos_phi(w) * m_alpha_x) * (geometry::cos_phi(w) * m_alpha_x) +
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

    bool is_delta() const { return std::max(m_alpha_x, m_alpha_y) < T(1e-3); }
};

}  // namespace pbpt::material
