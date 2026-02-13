#pragma once

#include <cstdlib>
#include <optional>
#include <variant>

#include "pbpt/material/bxdf.hpp"
#include "pbpt/material/model.hpp"
#include "pbpt/material/plugin/bxdf/conductor_rough_bxdf.hpp"
#include "pbpt/material/plugin/bxdf/conductor_specular_bxdf.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

template<typename T, int N>
class ConductorBxDF : public BxDF<ConductorBxDF<T, N>, T, N> {
    friend class BxDF<ConductorBxDF<T, N>, T, N>;
private:
    using AnyConductorBxDF = std::variant<
        ConductorSpecularBxDF<T, N>,
        ConductorRoughBxDF<T, N>
    >;

    AnyConductorBxDF m_bxdf;

public:
    ConductorBxDF(
        const radiometry::SampledSpectrum<T, N>& eta, 
        const radiometry::SampledSpectrum<T, N>& k,
        const MicrofacetModel<T>& microfacet_model
    ) : m_bxdf(microfacet_model.is_delta()
               ? AnyConductorBxDF(ConductorSpecularBxDF<T, N>(eta, k))
               : AnyConductorBxDF(ConductorRoughBxDF<T, N>(eta, k, microfacet_model))) {}

private:
    BxDFFlags type_impl() const {
        return std::visit([](const auto& bxdf) { return bxdf.type(); }, m_bxdf);
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode
    ) const {
        return std::visit([&](const auto& bxdf) {
            return bxdf.f(swl, wo, wi, mode);
        }, m_bxdf);
    };

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T uc,
        const math::Point<T, 2>& u2d,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        return std::visit([&](const auto& bxdf) {
            return bxdf.sample_f(swl, wo, uc, u2d, mode, sample_flags);
        }, m_bxdf);
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        return std::visit([&](const auto& bxdf) {
            return bxdf.pdf(wo, wi, mode, sample_flags);
        }, m_bxdf);
    }
};

} // namespace pbpt::material
