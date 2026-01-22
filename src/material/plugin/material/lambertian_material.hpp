#pragma once

#include "material/material.hpp"
#include "material/plugin/bxdf/lambertian_bxdf.hpp"
#include "radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::material {

/**
 * @brief Diffuse (Lambertian) material implementation.
 */
template<typename T>
class LambertianMaterial : public Material<LambertianMaterial<T>, T> {
private:
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_albedo_dist;

public:
    template<typename SpectrumDistributionType>
    explicit LambertianMaterial(const SpectrumDistributionType& albedo_distribution)
        : m_albedo_dist(albedo_distribution) {}

    template<int N>
    BSDF<T, N> compute_bsdf_impl(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::ShadingInfo<T>& shading,
        const radiometry::SampledWavelength<T, N>& wavelengths
    ) const {
        auto albedo = m_albedo_dist.template sample<N>(wavelengths);
        std::vector<AnyBxDF<T, N>> lobes;
        lobes.emplace_back(LambertianBxDF<T, N>(albedo));
        return make_bsdf(si, shading, std::move(lobes));
    }
};

}  // namespace pbpt::material
