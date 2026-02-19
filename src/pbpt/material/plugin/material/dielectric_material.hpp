#pragma once

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/dielectric_bxdf.hpp"

namespace pbpt::material {

/**
 * @brief Dielectric material that selects specular/rough/transparent via microfacet parameters.
 */
template <typename T>
class DielectricMaterial : public Material<DielectricMaterial<T>, T> {
private:
    T m_eta;
    MicrofacetModel<T> m_microfacet_model;

public:
    DielectricMaterial(T eta, const MicrofacetModel<T>& microfacet_model)
        : m_eta(eta), m_microfacet_model(microfacet_model) {}

    template <int N>
    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si, const geometry::ShadingInfo<T>& shading,
                                 const radiometry::SampledWavelength<T, N>& wavelengths,
                                 const std::optional<geometry::SurfaceDifferentials<T>>&) const {
        return BSDF<T, N>(si, shading, DielectricBxDF<T, N>(m_eta, m_microfacet_model));
    }
};

}  // namespace pbpt::material
