#pragma once

#include <cmath>
#include <cstdlib>
#include <optional>

#include "pbpt/geometry/spherical.hpp"
#include "pbpt/material/bxdf.hpp"
#include "pbpt/material/model.hpp"
#include "pbpt/math/function.hpp"
#include "pbpt/math/normal.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

#include "pbpt/material/optics.hpp"

namespace pbpt::material {

template<typename T, int N>
class ConductorRoughBxDF : public BxDF<ConductorRoughBxDF<T, N>, T, N> {
    friend class BxDF<ConductorRoughBxDF<T, N>, T, N>;
private:
    radiometry::SampledSpectrum<T, N> m_eta;
    radiometry::SampledSpectrum<T, N> m_k;
    MicrofacetModel<T> m_microfacet_model;

public:
    ConductorRoughBxDF(
        const radiometry::SampledSpectrum<T, N>& eta, 
        const radiometry::SampledSpectrum<T, N>& k,
        const MicrofacetModel<T>& microfacet_model
    ) : m_eta(eta), m_k(k), m_microfacet_model(microfacet_model) {}

    const radiometry::SampledSpectrum<T, N>& eta() const { return m_eta; }
    const radiometry::SampledSpectrum<T, N>& k() const { return m_k; }

private:
    BxDFFlags type_impl() const {
        return BxDFFlags::GlossyReflection;
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode
    ) const {
        math::Vector<T, 3> wm = wi + wo;
        if (math::is_zero(wm.length())) return radiometry::SampledSpectrum<T, N>::filled(0);
        wm = wm.normalized();

        auto F = fresnel_conductor(
            std::abs(wo.dot(wm)),
            m_eta,
            m_k
        );

        auto cos_theta_i = geometry::cos_theta(wi), abs_cos_theta_i = std::abs(cos_theta_i);
        auto cos_theta_o = geometry::cos_theta(wo), abs_cos_theta_o = std::abs(cos_theta_o);

        if (abs_cos_theta_i == 0 || abs_cos_theta_o == 0) 
            return radiometry::SampledSpectrum<T, N>::filled(0);
        
        return m_microfacet_model.D(wm) * m_microfacet_model.G(wo, wi) * F /
                (T(4) * abs_cos_theta_i * abs_cos_theta_o);
   
    };

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T,
        const math::Point<T, 2>& u2d,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) {
            return std::nullopt;
        }

        // Sample microfacet normal wm
        math::Vector<T, 3> wm = m_microfacet_model.sample_wm(wo, u2d);
        // Compute reflected direction wi
        math::Vector<T, 3> wi = reflect(wo, wm);
        if (!is_same_hemisphere(wo, wi)) {
            return std::nullopt;
        }

        T pdf_wm = m_microfacet_model.pdf_wm(wo, wm);
        T pdf = pdf_wm / (T(4) * std::abs(wo.dot(wm)));

        BxDFSampleRecord<T, N> record;
        record.wi = wi;
        record.pdf = pdf;
        record.f = this->f_impl(swl, wo, wi, mode);
        record.sampled_flags = BxDFFlags::GlossyReflection;
        record.eta = T(1);
        return record;
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) {
            return 0;
        }

        math::Vector<T, 3> wm = wi + wo;
        if (math::is_zero(wm.length())) {
            return 0;
        }

        wm = wm.normalized();
        wm = math::Normal<T, N>(wm).face_forward(math::Vector<T, 3>(0, 0, 1));

        return m_microfacet_model.pdf_wm(wo, wm) / (T(4) * std::abs(wo.dot(wm)));
    }
};

} // namespace pbpt::material
