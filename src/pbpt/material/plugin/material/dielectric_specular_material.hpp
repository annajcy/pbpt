#pragma once

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/dielectric_specular_bxdf.hpp"

namespace pbpt::material {

/**
 * @brief Specular dielectric material (perfect reflection + refraction).
 */
template <typename T>
class DielectricSpecularMaterial : public Material<DielectricSpecularMaterial<T>, T> {
private:
    T m_eta;

public:
    explicit DielectricSpecularMaterial(T eta) : m_eta(eta) {}

    template <int N>
    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si, const geometry::ShadingInfo<T>& shading,
                                 const radiometry::SampledWavelength<T, N>& wavelengths,
                                 const std::optional<geometry::SurfaceDifferentials<T>>&) const {
        return BSDF<T, N>(si, shading, DielectricSpecularBxDF<T, N>(m_eta));
    }
};

}  // namespace pbpt::material
