#pragma once

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/conductor_rough_bxdf.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::material {

/**
 * @brief Rough conductor material (microfacet reflection with complex IOR).
 */
template<typename T>
class ConductorRoughMaterial : public Material<ConductorRoughMaterial<T>, T> {
private:
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_eta_dist;
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_k_dist;
    MicrofacetModel<T> m_microfacet_model;

public:
    ConductorRoughMaterial(
        radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist,
        radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist,
        const MicrofacetModel<T>& microfacet_model
    ) : m_eta_dist(std::move(eta_dist)),
        m_k_dist(std::move(k_dist)),
        m_microfacet_model(microfacet_model) {}

    template<int N>
    BSDF<T, N> compute_bsdf_impl(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::ShadingInfo<T>& shading,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const std::optional<geometry::SurfaceDifferentials<T>>&
    ) const {
        auto eta = m_eta_dist.template sample<N>(wavelengths);
        auto k = m_k_dist.template sample<N>(wavelengths);
        return BSDF<T, N>(si, shading, ConductorRoughBxDF<T, N>(eta, k, m_microfacet_model));
    }
};

}  // namespace pbpt::material
