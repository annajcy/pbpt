#pragma once

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/dielectric_rough_bxdf.hpp"

namespace pbpt::material {

/**
 * @brief Rough dielectric material (microfacet reflection + transmission).
 */
template<typename T>
class DielectricRoughMaterial : public Material<DielectricRoughMaterial<T>, T> {
private:
    T m_eta;
    MicrofacetModel<T> m_microfacet_model;

public:
    DielectricRoughMaterial(T eta, const MicrofacetModel<T>& microfacet_model)
        : m_eta(eta), m_microfacet_model(microfacet_model) {}

    template<int N>
    BSDF<T, N> compute_bsdf_impl(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::ShadingInfo<T>& shading,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const std::optional<geometry::SurfaceDifferentials<T>>&
    ) const {
        return BSDF<T, N>(si, shading, DielectricRoughBxDF<T, N>(m_eta, m_microfacet_model));
    }
};

}  // namespace pbpt::material
