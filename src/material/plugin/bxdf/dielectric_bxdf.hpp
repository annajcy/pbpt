#pragma once

#include <optional>
#include <variant>

#include "material/bxdf.hpp"
#include "material/model.hpp"
#include "material/plugin/bxdf/dielectric_rough_bxdf.hpp"
#include "material/plugin/bxdf/dielectric_specular_bxdf.hpp"
#include "material/plugin/bxdf/null_bxdf.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

template<typename T, int N>
class DielectricBxDF : public BxDF<DielectricBxDF<T, N>, T, N> {
    friend class BxDF<DielectricBxDF<T, N>, T, N>;
private:
    using AnyDielectricBxDF = std::variant<
        DielectricSpecularBxDF<T, N>,
        DielectricRoughBxDF<T, N>,
        NullBxDF<T, N>
    >;

    AnyDielectricBxDF m_bxdf;

public:
    DielectricBxDF(T eta, const MicrofacetModel<T>& microfacet_model)
        : m_bxdf(select_bxdf(eta, microfacet_model)) {}

private:
    static AnyDielectricBxDF select_bxdf(T eta, const MicrofacetModel<T>& microfacet_model) {
        if (microfacet_model.is_delta()) {
            return AnyDielectricBxDF(DielectricSpecularBxDF<T, N>(eta));
        }
        if (eta == T(1)) {
            return AnyDielectricBxDF(NullBxDF<T, N>());
        }
        return AnyDielectricBxDF(DielectricRoughBxDF<T, N>(eta, microfacet_model));
    }

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
