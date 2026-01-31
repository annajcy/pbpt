#pragma once

#include <cmath>
#include <optional>

#include "material/bxdf.hpp"
#include "material/model.hpp"
#include "math/normal.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "material/optics.hpp"

namespace pbpt::material {

template<typename T, int N>
class DielectricRoughBxDF : public BxDF<DielectricRoughBxDF<T, N>, T, N> {
    friend class BxDF<DielectricRoughBxDF<T, N>, T, N>;
private:
    T m_eta;
    MicrofacetModel<T> m_microfacet_model;

public:
    DielectricRoughBxDF(T eta, const MicrofacetModel<T>& microfacet_model) : m_eta(eta), m_microfacet_model(microfacet_model) {}

private:
    BxDFFlags type_impl() const {
        if (m_eta == T(1)) {
            return BxDFFlags::GlossyTransmission;
        }
        return BxDFFlags::GlossyReflection | BxDFFlags::GlossyTransmission;
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode
    ) const {
        T cos_theta_o = geometry::cos_theta(wo), cos_theta_i = geometry::cos_theta(wi);
        bool is_reflect = is_same_hemisphere(wo, wi);
        T etap = 1;
        if (!is_reflect) {
            etap = (cos_theta_o > 0) ? m_eta : (1 / m_eta);
        }

        math::Vector<T, 3> wm = wi * etap + wo;
        if (math::is_zero(wm.length()) || cos_theta_i == 0 || cos_theta_o == 0) {
            return radiometry::SampledSpectrum<T, N>::filled(0);
        }
        wm = wm.normalized();
        wm = math::Normal<T, 3>(wm).face_forward(math::Vector<T, 3>(0, 0, 1));

        // discard cases where wm is not consistent with wo and wi
        if (wm.dot(wi) * cos_theta_i < 0 || wm.dot(wo) * cos_theta_o < 0)
            return radiometry::SampledSpectrum<T,N>::filled(0);

        auto Re = fresnel_dielectric(wo.dot(wm), m_eta);
        auto Tr = T(1) - Re;

        if (is_reflect) {
            auto fr = m_microfacet_model.D(wm) * m_microfacet_model.G(wo, wi) * Re /
                       (T(4) * std::abs(geometry::cos_theta(wo)) * std::abs(geometry::cos_theta(wi)));
            return radiometry::SampledSpectrum<T, N>::filled(fr);
        } else {
            T denom = wi.dot(wm) + wo.dot(wm) / etap;
            denom = denom * denom;
            auto ft = m_microfacet_model.D(wm) * m_microfacet_model.G(wo, wi) * Tr *
                      std::abs(wi.dot(wm)) * std::abs(wo.dot(wm)) /
                      std::abs(denom * geometry::cos_theta(wo) * geometry::cos_theta(wi));

            if (mode == TransportMode::Radiance) {
                ft /= (etap * etap);
            }
            return radiometry::SampledSpectrum<T, N>::filled(ft);
        }
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

        math::Vector<T, 3> wm = m_microfacet_model.sample_wm(wo, u2d);
        auto Re = fresnel_dielectric(wo.dot(wm), m_eta);
        auto Tr = T(1) - Re;

        auto pr = Re, pt = Tr;
        if (!(sample_flags & BxDFReflTransFlags::Reflection)) pr = T(0);
        if (!(sample_flags & BxDFReflTransFlags::Transmission)) pt = T(0);
        if (pr == T(0) && pt == T(0)) {
            return std::nullopt;
        }
        T sum = pr + pt;
        pr /= sum, pt /= sum;

        if (uc < pr) {
            // reflection
            math::Vector<T, 3> wi = reflect(wo, wm);
            if (!is_same_hemisphere(wo, wi)) {
                return std::nullopt;
            }

            T pdf_wm = m_microfacet_model.pdf_wm(wo, wm);
            T pdf = pdf_wm / (T(4) * std::abs(wo.dot(wm))) * pr;

            auto fr = m_microfacet_model.D(wm) * m_microfacet_model.G(wo, wi) * Re /
                       (T(4) * std::abs(geometry::cos_theta(wo)) * std::abs(geometry::cos_theta(wi)));

            return BxDFSampleRecord<T, N>{
                .f = radiometry::SampledSpectrum<T, N>::filled(fr),
                .wi = wi,
                .pdf = pdf,
                .eta = T(1),
                .sampled_flags = BxDFFlags::GlossyReflection
            };

        } else {
            // transmission
            auto refract_result_opt = refract(wo, wm, m_eta);
            if (!refract_result_opt.has_value()) {
                return std::nullopt;
            }

            RefractResult<T> refract_result = refract_result_opt.value();
            auto wi = refract_result.wt;
            auto etap = refract_result.etap;
            if (is_same_hemisphere(wo, wi) || wi.z() == 0) {
                return std::nullopt;
            }
            // Compute PDF of rough dielectric transmission
            T denom = wi.dot(wm) + wo.dot(wm) / etap;
            denom = denom * denom;
            T dwm_dwi = std::abs(wi.dot(wm)) / denom;
            T pdf_wm = m_microfacet_model.pdf_wm(wo, wm);
            T pdf = pdf_wm * dwm_dwi * pt;

            auto ft = m_microfacet_model.D(wm) * m_microfacet_model.G(wo, wi) * Tr *
                      std::abs(wi.dot(wm)) * std::abs(wo.dot(wm)) /
                      std::abs(denom * geometry::cos_theta(wo) * geometry::cos_theta(wi));

            if (mode == TransportMode::Radiance) {
                ft /= (etap * etap);
            }

            return BxDFSampleRecord<T, N>{
                .f = radiometry::SampledSpectrum<T, N>::filled(ft),
                .wi = wi,
                .pdf = pdf,
                .eta = etap,
                .sampled_flags = BxDFFlags::GlossyTransmission
            };
        
        }
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        T cos_theta_o = geometry::cos_theta(wo), cos_theta_i = geometry::cos_theta(wi);
        bool is_reflect = is_same_hemisphere(wo, wi);
        T etap = 1;
        if (!is_reflect) {
            etap = (cos_theta_o > 0) ? m_eta : (1 / m_eta);
        }

        math::Vector<T, 3> wm = wi * etap + wo;
        if (math::is_zero(wm.length()) || cos_theta_i == 0 || cos_theta_o == 0) {
            return 0;
        }
        wm = wm.normalized();
        wm = math::Normal<T, 3>(wm).face_forward(math::Vector<T, 3>(0, 0, 1));

        // discard cases where wm is not consistent with wo and wi
        if (wm.dot(wi) * cos_theta_i < 0 || wm.dot(wo) * cos_theta_o < 0)
            return T(0);

        auto Re = fresnel_dielectric(wo.dot(wm), m_eta);
        auto Tr = T(1) - Re;

        auto pr = Re, pt = Tr;
        if (!(sample_flags & BxDFReflTransFlags::Reflection)) pr = 0;
        if (!(sample_flags & BxDFReflTransFlags::Transmission)) pt = 0;
        
        if (pr == T(0) && pt == T(0)) {
            return T(0);
        }
        T sum = pr + pt;
        pr /= sum, pt /= sum;

        if (is_reflect) {
            T pdf_wm = m_microfacet_model.pdf_wm(wo, wm);
            T pdf = pdf_wm / (T(4) * std::abs(wo.dot(wm))) * pr;
            return pdf;
        } else {
            T denom = (wi.dot(wm) + wo.dot(wm) / etap);
            denom = denom * denom;
            T dwm_dwi = std::abs(wi.dot(wm)) / denom;
            T pdf_wm = m_microfacet_model.pdf_wm(wo, wm);
            T pdf = pdf_wm * dwm_dwi * pt;
            return pdf;
        }
    }
};

} // namespace pbpt::material
