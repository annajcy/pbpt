#pragma once

#include <cmath>
#include <optional>

#include "material/bxdf.hpp"
#include "math/function.hpp"
#include "math/normal.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "sampler/3d.hpp"

#include "material/optics.hpp"

namespace pbpt::material {

// --- Lambertian 实现 ---
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

        math::Vector<T, 3> wi = math::Vector<T, 3>(-wo.x(), -wo.y(), wo.z());
        auto cos_theta_i = wi.z();
        auto abs_cos_theta_i = std::abs(cos_theta_i);
        T reflect_p = fresnel_dielectric(cos_theta_i, m_eta);
        T transmission_p = T(1) - reflect_p;

        T pr, pt;
        if (is_match_refl_trans(BxDFFlags::SpecularReflection, sample_flags) &&
            is_match_refl_trans(BxDFFlags::SpecularTransmission, sample_flags)) {
            T sum = reflect_p + transmission_p;
            pr = reflect_p / sum;
            pt = transmission_p / sum;
        } else if (is_match_refl_trans(BxDFFlags::SpecularTransmission, sample_flags) && 
                   !is_match_refl_trans(BxDFFlags::SpecularReflection, sample_flags)) {
            pr = T(0);
            pt = T(1);
        } else if (is_match_refl_trans(BxDFFlags::SpecularReflection, sample_flags) && 
                   !is_match_refl_trans(BxDFFlags::SpecularTransmission, sample_flags)) {
            pr = T(1);
            pt = T(0);
        } else {
            return std::nullopt;
        }

        BxDFSampleRecord<T, N> record;
        if (uc < pr) {
            // reflection
            record.wi = wi;
            record.pdf = pr;
            record.f = reflect_p *
                       radiometry::SampledSpectrum<T, N>::filled(1.0 / abs_cos_theta_i);
            record.sampled_flags = BxDFFlags::SpecularReflection;
            record.eta = T(1);
        } else {
            // transmission
            auto refract_result_opt = refract(wo, math::Normal<T, 3>(0, 0, 1), m_eta);
            if (!refract_result_opt.has_value()) {
                // total internal reflection, should not happen here
                return std::nullopt;
            }
            auto refract_result = refract_result_opt.value();
            record.wi = refract_result.wt;
            record.pdf = pt;
            record.f = transmission_p *
                           radiometry::SampledSpectrum<T, N>::filled(1.0 / std::abs(record.wi.z()));
            if (mode == TransportMode::Radiance) {
                record.f /= (refract_result.etap * refract_result.etap);
            } 
            record.sampled_flags = BxDFFlags::SpecularTransmission;
            record.eta = refract_result.etap;
        }
        return record;
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
