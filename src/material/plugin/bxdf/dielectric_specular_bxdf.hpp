#pragma once

#include <cmath>
#include <optional>

#include "geometry/spherical.hpp"
#include "material/bxdf.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "material/optics.hpp"

namespace pbpt::material {

template<typename T, int N>
class DielectricSpecularBxDF : public BxDF<DielectricSpecularBxDF<T, N>, T, N> {
    friend class BxDF<DielectricSpecularBxDF<T, N>, T, N>;
private:
    T m_eta;

public:
    DielectricSpecularBxDF(T eta) : m_eta(eta) {}

private:
    BxDFFlags type_impl() const {
        return BxDFFlags::SpecularReflection | BxDFFlags::SpecularTransmission;
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode
    ) const {
        return radiometry::SampledSpectrum<T, N>::filled(0);
    };

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T uc,
        const math::Point<T, 2>& u2d,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) {
            return std::nullopt;
        }

        T Re = fresnel_dielectric(geometry::cos_theta(wo), m_eta);
        T Tr = T(1) - Re;

        T pr = Re, pt = Tr;
        if (!(sample_flags & BxDFReflTransFlags::Reflection)) pr = T(0);
        if (!(sample_flags & BxDFReflTransFlags::Transmission)) pt = T(0);
        if (pr == T(0) && pt == T(0)) {
            return std::nullopt;
        }
        T sum = pr + pt;
        pr /= sum, pt /= sum;


        if (uc < pr) {
            // reflection
            math::Vector<T, 3> wi = math::Vector<T, 3>(-wo.x(), -wo.y(), wo.z());
            auto cos_theta_i = geometry::cos_theta(wi);
            auto abs_cos_theta_i = std::abs(cos_theta_i);
            return BxDFSampleRecord<T, N>{
                .wi = wi,
                .pdf = pr,
                .f = radiometry::SampledSpectrum<T, N>::filled(Re / abs_cos_theta_i),
                .sampled_flags = BxDFFlags::SpecularReflection,
                .eta = T(1)
            };
        } else {
            // transmission
            auto refract_result_opt = refract(wo, math::Vector<T, 3>(0, 0, 1), m_eta);
            if (!refract_result_opt.has_value()) {
                // total internal reflection, should not happen here
                return std::nullopt;
            }
            auto refract_result = refract_result_opt.value();
            auto etap = refract_result.etap;
            auto wi = refract_result.wt;
            
            if (is_same_hemisphere(wo, wi) || wi.z() == 0) {
                return std::nullopt;
            }

            auto ft = Tr / std::abs(wi.z());
            if (mode == TransportMode::Radiance) {
                ft /= (etap * etap);
            }

            return BxDFSampleRecord<T, N>{
                .wi = wi,
                .pdf = pt,
                .f = radiometry::SampledSpectrum<T, N>::filled(ft),
                .sampled_flags = BxDFFlags::SpecularTransmission,
                .eta = etap
            };
        }       
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode trasport_mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        return T(0);
    }
};

} // namespace pbpt::material
