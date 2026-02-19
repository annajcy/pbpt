#pragma once

#include <cmath>
#include <cstdlib>
#include <optional>

#include "pbpt/geometry/spherical.hpp"
#include "pbpt/material/bxdf.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

#include "pbpt/material/optics.hpp"

namespace pbpt::material {

template <typename T, int N>
class ConductorSpecularBxDF : public BxDF<ConductorSpecularBxDF<T, N>, T, N> {
    friend class BxDF<ConductorSpecularBxDF<T, N>, T, N>;

private:
    radiometry::SampledSpectrum<T, N> m_eta;
    radiometry::SampledSpectrum<T, N> m_k;

public:
    ConductorSpecularBxDF(const radiometry::SampledSpectrum<T, N>& eta, const radiometry::SampledSpectrum<T, N>& k)
        : m_eta(eta), m_k(k) {}

    const radiometry::SampledSpectrum<T, N>& eta() const { return m_eta; }
    const radiometry::SampledSpectrum<T, N>& k() const { return m_k; }

private:
    BxDFFlags type_impl() const { return BxDFFlags::SpecularReflection; }

    radiometry::SampledSpectrum<T, N> f_impl(const radiometry::SampledWavelength<T, N>&, const math::Vector<T, 3>& wo,
                                             const math::Vector<T, 3>& wi, TransportMode mode) const {
        // due to the delta distribution, f is zero because the probability of sampling the exact reflection direction
        // is zero
        return radiometry::SampledSpectrum<T, N>::filled(0);
    };

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl, const math::Vector<T, 3>& wo, const T,
        const math::Point<T, 2>& u2d, TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) {
            return std::nullopt;
        }

        math::Vector<T, 3> wi = math::Vector<T, 3>(-wo.x(), -wo.y(), wo.z());
        auto abs_cos_theta = std::abs(geometry::cos_theta(wi));

        BxDFSampleRecord<T, N> record;
        record.wi = wi;
        record.pdf = 1.0;  // delta distribution in reflection direction, pdf is 1 under the condition of hitting the
                           // exact direction
        record.f = fresnel_conductor(abs_cos_theta, m_eta, m_k) *
                   radiometry::SampledSpectrum<T, N>::filled(1.0 / abs_cos_theta);
        record.sampled_flags = type_impl();
        return record;
    }

    T pdf_impl(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wi, TransportMode,
               const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All) const {
        // due to the delta distribution, the PDF is zero because the probability of sampling the exact reflection
        // direction is zero
        return 0;
    }
};

}  // namespace pbpt::material
