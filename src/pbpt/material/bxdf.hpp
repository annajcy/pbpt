#pragma once

#include <cstddef>
#include <cstdlib>
#include <optional>
#include <vector>
#include "pbpt/geometry/ray.hpp"
#include "pbpt/geometry/interaction.hpp"
#include "pbpt/geometry/spherical.hpp"
#include "pbpt/math/function.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/sampler/3d.hpp"

namespace pbpt::material {

enum class TransportMode : int { Radiance, Importance };

enum class BxDFFlags : int {
    Unset = 0,
    // Directional types
    Reflection = 1 << 0,
    Transmission = 1 << 1,
    // Base types
    Diffuse = 1 << 2,
    Glossy = 1 << 3,
    Specular = 1 << 4,
    // Composite types
    DiffuseReflection = Diffuse | Reflection,
    DiffuseTransmission = Diffuse | Transmission,
    GlossyReflection = Glossy | Reflection,
    GlossyTransmission = Glossy | Transmission,
    SpecularReflection = Specular | Reflection,
    SpecularTransmission = Specular | Transmission,

    ALL = Reflection | Transmission | Diffuse | Glossy | Specular,
    ANY = ALL + 1  // sentinel meaning "no filtering"
};

// 重载位运算符
inline BxDFFlags operator|(BxDFFlags a, BxDFFlags b) {
    return static_cast<BxDFFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline BxDFFlags operator&(BxDFFlags a, BxDFFlags b) {
    return static_cast<BxDFFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline BxDFFlags& operator|=(BxDFFlags& a, BxDFFlags b) {
    a = a | b;
    return a;
}

// is type the subset of filters?
inline bool is_match_flags(BxDFFlags type, BxDFFlags filters) {
    return filters == BxDFFlags::ANY || (type & filters) == type;
}

// does type has the flag? even if only one flag bit exists in flag, return true
inline bool has_flag(BxDFFlags type, BxDFFlags flag) {
    return (type & flag) != BxDFFlags::Unset;
}

enum class BxDFReflTransFlags : int {
    Unset = 0,
    Reflection = 1 << 0,
    Transmission = 1 << 1,
    All = Reflection | Transmission
};

inline bool operator==(BxDFReflTransFlags a, int b) {
    return static_cast<int>(a) == b;
}

inline bool operator!(BxDFReflTransFlags a) {
    return a == BxDFReflTransFlags::Unset;
}

inline BxDFReflTransFlags operator|(BxDFReflTransFlags a, BxDFReflTransFlags b) {
    return static_cast<BxDFReflTransFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline BxDFReflTransFlags operator&(BxDFReflTransFlags a, BxDFReflTransFlags b) {
    return static_cast<BxDFReflTransFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline BxDFReflTransFlags& operator|=(BxDFReflTransFlags& a, BxDFReflTransFlags b) {
    a = a | b;
    return a;
}

inline bool has_flag(BxDFReflTransFlags type, BxDFReflTransFlags flag) {
    return (type & flag) != BxDFReflTransFlags::Unset;
}

inline bool is_match_refl_trans(BxDFFlags type, BxDFReflTransFlags flags) {
    if (flags == BxDFReflTransFlags::All)
        return true;
    if (flags == BxDFReflTransFlags::Unset)
        return false;

    bool matches_reflection = has_flag(flags, BxDFReflTransFlags::Reflection) && has_flag(type, BxDFFlags::Reflection);
    bool matches_transmission =
        has_flag(flags, BxDFReflTransFlags::Transmission) && has_flag(type, BxDFFlags::Transmission);
    return matches_reflection || matches_transmission;
}

template <typename T, int N>
struct BxDFSampleRecord {
    radiometry::SampledSpectrum<T, N> f;         // BxDF值
    math::Vector<T, 3> wi;                       // 入射方向
    T pdf = 0;                                   // 采样概率密度
    T eta = 1;                                   // 折射率比值
    BxDFFlags sampled_flags = BxDFFlags::Unset;  // 记录采样到的具体类型
};

template <typename Derived, typename T, int N>
class BxDF {
public:
    Derived& as_derived() { return static_cast<Derived&>(*this); }
    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }
    BxDFFlags type() const { return as_derived().type_impl(); }

    radiometry::SampledSpectrum<T, N> rou_hh(const radiometry::SampledWavelength<T, N>& swl, const std::vector<T>& uc,
                                             const std::vector<math::Point<T, 2>>& u2d_i,
                                             const std::vector<math::Point<T, 2>>& u2d_o,
                                             TransportMode mode = TransportMode::Radiance) {
        auto res = radiometry::SampledSpectrum<T, N>::zeros();
        for (size_t i = 0; i < uc.size(); i++) {
            math::Vector<T, 3> wo = sampler::sample_uniform_hemisphere(u2d_o[i]);
            if (wo.z() == 0)
                continue;
            T pdf_wo = sampler::sample_uniform_hemisphere_pdf<T>();
            auto bsdf_sample_rec_opt = this->sample_f(swl, wo, uc[i], u2d_i[i], mode);
            if (bsdf_sample_rec_opt) {
                res += (bsdf_sample_rec_opt->f * std::abs(geometry::cos_theta(bsdf_sample_rec_opt->wi)) *
                        std::abs(geometry::cos_theta(wo))) /
                       (bsdf_sample_rec_opt->pdf * pdf_wo);
            }
        }
        return res / static_cast<T>(uc.size() * math::pi_v<T>);
    }

    radiometry::SampledSpectrum<T, N> rou_hd(const radiometry::SampledWavelength<T, N>& swl,
                                             const math::Vector<T, 3>& wo, const std::vector<T>& uc,
                                             const std::vector<math::Point<T, 2>>& u2d,
                                             TransportMode mode = TransportMode::Radiance) {
        auto res = radiometry::SampledSpectrum<T, N>::zeros();
        for (size_t i = 0; i < uc.size(); ++i) {
            auto bsdf_sample_rec_opt = this->sample_f(swl, wo, uc[i], u2d[i], mode);
            if (bsdf_sample_rec_opt) {
                res += bsdf_sample_rec_opt->f * std::abs(geometry::cos_theta(bsdf_sample_rec_opt->wi)) /
                       bsdf_sample_rec_opt->pdf;
            }
        }
        return res / static_cast<T>(uc.size());
    }

    radiometry::SampledSpectrum<T, N> f(const radiometry::SampledWavelength<T, N>& swl, const math::Vector<T, 3>& wo,
                                        const math::Vector<T, 3>& wi,
                                        TransportMode mode = TransportMode::Radiance) const {
        return as_derived().f_impl(swl, wo, wi, mode);
    }

    std::optional<BxDFSampleRecord<T, N>> sample_f(
        const radiometry::SampledWavelength<T, N>& swl, const math::Vector<T, 3>& wo, const T uc,
        const math::Point<T, 2>& u2d, TransportMode mode = TransportMode::Radiance,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All) const {
        return as_derived().sample_f_impl(swl, wo, uc, u2d, mode, sample_flags);
    }

    T pdf(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wi, TransportMode mode = TransportMode::Radiance,
          const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All) const {
        return as_derived().pdf_impl(wo, wi, mode, sample_flags);
    }
};

template <typename T>
inline std::optional<geometry::RayDifferentialOffset<T>> make_ray_differential_offset(
    BxDFFlags bxdf_type, const geometry::RayDifferential<T, 3>& ray_diff, const geometry::SurfaceInteraction<T>& si,
    const geometry::ShadingInfo<T>& shading, const geometry::SurfaceDifferentials<T>& surface_diffs,
    const math::Vector<T, 3>& wi, T eta) {
    geometry::RayDifferentialOffset<T> offset{};

    const bool is_specular = has_flag(bxdf_type, BxDFFlags::Specular);
    const bool is_reflection = has_flag(bxdf_type, BxDFFlags::Reflection);
    const bool is_transmission = has_flag(bxdf_type, BxDFFlags::Transmission);
    if (!is_specular || (!is_reflection && !is_transmission)) {
        return std::nullopt;
    }

    auto finite_vec = [](const math::Vector<T, 3>& v) {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    };
    auto finite_point = [](const math::Point<T, 3>& p) {
        return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
    };

    auto n = shading.n.to_vector();
    if (n.length_squared() <= math::epsilon_v<T>) {
        return std::nullopt;
    }
    n = n.normalized();

    auto dndx = shading.dndu.to_vector() * surface_diffs.dudx + shading.dndv.to_vector() * surface_diffs.dvdx;
    auto dndy = shading.dndu.to_vector() * surface_diffs.dudy + shading.dndv.to_vector() * surface_diffs.dvdy;

    const auto& wo = si.wo();
    auto dwodx = -ray_diff.x().direction() - wo;
    auto dwody = -ray_diff.y().direction() - wo;

    offset.dpdx = surface_diffs.dpdx;
    offset.dpdy = surface_diffs.dpdy;

    if (is_reflection) {
        T dwo_dot_n_dx = dwodx.dot(n) + wo.dot(dndx);
        T dwo_dot_n_dy = dwody.dot(n) + wo.dot(dndy);
        T wo_dot_n = wo.dot(n);

        auto rx_direction = wi - dwodx + (wo_dot_n * dndx + dwo_dot_n_dx * n) * T(2);
        auto ry_direction = wi - dwody + (wo_dot_n * dndy + dwo_dot_n_dy * n) * T(2);
        offset.dwdx = rx_direction - wi;
        offset.dwdy = ry_direction - wi;
    } else if (is_transmission) {
        auto n_t = n;
        auto dndx_t = dndx;
        auto dndy_t = dndy;
        if (wo.dot(n_t) < T(0)) {
            n_t = -n_t;
            dndx_t = -dndx_t;
            dndy_t = -dndy_t;
        }

        T abs_dot_wi_n = std::abs(wi.dot(n_t));
        if (abs_dot_wi_n <= math::epsilon_v<T>) {
            return std::nullopt;
        }

        T dDNdx = dwodx.dot(n_t) + wo.dot(dndx_t);
        T dDNdy = dwody.dot(n_t) + wo.dot(dndy_t);
        T wo_dot_n = wo.dot(n_t);
        T mu = eta * wo_dot_n - abs_dot_wi_n;

        T dmudx = (eta - (eta * eta * wo_dot_n) / abs_dot_wi_n) * dDNdx;
        T dmudy = (eta - (eta * eta * wo_dot_n) / abs_dot_wi_n) * dDNdy;

        auto rx_direction = wi - eta * dwodx + (mu * dndx_t + dmudx * n_t);
        auto ry_direction = wi - eta * dwody + (mu * dndy_t + dmudy * n_t);
        offset.dwdx = rx_direction - wi;
        offset.dwdy = ry_direction - wi;
    }

    // Squash potentially troublesome differentials.
    constexpr T kMaxLen2 = static_cast<T>(1e16);
    auto rx_origin = si.point() + offset.dpdx;
    auto ry_origin = si.point() + offset.dpdy;
    auto rx_direction = wi + offset.dwdx;
    auto ry_direction = wi + offset.dwdy;

    bool invalid = !finite_vec(offset.dpdx) || !finite_vec(offset.dpdy) || !finite_vec(offset.dwdx) ||
                   !finite_vec(offset.dwdy) || !finite_point(rx_origin) || !finite_point(ry_origin) ||
                   !finite_vec(rx_direction) || !finite_vec(ry_direction) ||
                   rx_origin.to_vector().length_squared() > kMaxLen2 ||
                   ry_origin.to_vector().length_squared() > kMaxLen2 || rx_direction.length_squared() > kMaxLen2 ||
                   ry_direction.length_squared() > kMaxLen2;
    if (invalid) {
        return std::nullopt;
    }

    return std::make_optional(offset);
}

}  // namespace pbpt::material
