#pragma once

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/conductor_specular_bxdf.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::material {

/**
 * @brief Specular conductor material (perfect mirror with complex IOR).
 */
template <typename T>
class ConductorSpecularMaterial : public Material<ConductorSpecularMaterial<T>, T> {
private:
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_eta_dist;
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_k_dist;

public:
    ConductorSpecularMaterial(radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist,
                              radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist)
        : m_eta_dist(std::move(eta_dist)), m_k_dist(std::move(k_dist)) {}

    template <int N>
    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si, const geometry::ShadingInfo<T>& shading,
                                 const radiometry::SampledWavelength<T, N>& wavelengths,
                                 const std::optional<geometry::SurfaceDifferentials<T>>&) const {
        auto eta = m_eta_dist.template sample<N>(wavelengths);
        auto k = m_k_dist.template sample<N>(wavelengths);
        return BSDF<T, N>(si, shading, ConductorSpecularBxDF<T, N>(eta, k));
    }
};

}  // namespace pbpt::material
