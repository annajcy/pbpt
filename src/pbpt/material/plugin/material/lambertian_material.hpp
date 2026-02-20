#pragma once

#include <algorithm>
#include <type_traits>
#include <variant>

#include "pbpt/material/bsdf.hpp"
#include "pbpt/material/material.hpp"
#include "pbpt/material/plugin/bxdf/lambertian_bxdf.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/texture/plugin/texture/texture_type.hpp"

namespace pbpt::material {

/**
 * @brief Diffuse (Lambertian) material implementation.
 */
template <typename T>
class LambertianMaterial : public Material<LambertianMaterial<T>, T> {
private:
    using ReflectanceSource = std::variant<radiometry::PiecewiseLinearSpectrumDistribution<T>, texture::AnyTexture<T>>;
    ReflectanceSource m_reflectance_source;
    int m_texture_id{-1};

public:
    explicit LambertianMaterial(const radiometry::PiecewiseLinearSpectrumDistribution<T>& albedo_distribution)
        : m_reflectance_source(albedo_distribution) {}

    explicit LambertianMaterial(const texture::AnyTexture<T>& reflectance_texture)
        : m_reflectance_source(reflectance_texture) {}

    LambertianMaterial(int texture_id, const texture::AnyTexture<T>& tex)
        : m_reflectance_source(tex), m_texture_id(texture_id) {}

    const ReflectanceSource& reflectance_source() const { return m_reflectance_source; }
    int texture_id() const { return m_texture_id; }

    template <int N>
    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si, const geometry::ShadingInfo<T>& shading,
                                 const radiometry::SampledWavelength<T, N>& wavelengths,
                                 const std::optional<geometry::SurfaceDifferentials<T>>& differentials) const {
        auto albedo = std::visit(
            [&](const auto& source) {
                using SourceT = std::decay_t<decltype(source)>;
                if constexpr (std::is_same_v<SourceT, radiometry::PiecewiseLinearSpectrumDistribution<T>>) {
                    return source.template sample<N>(wavelengths);
                } else {
                    texture::TextureEvalContext<T> tex_ctx{};
                    tex_ctx.uv = si.uv();
                    tex_ctx.differentials = differentials;
                    tex_ctx.dpdu = si.dpdu();
                    tex_ctx.dpdv = si.dpdv();

                    return std::visit(
                        [&](const auto& tex) -> radiometry::SampledSpectrum<T, N> {
                            using TexT = std::decay_t<decltype(tex)>;
                            if constexpr (std::is_same_v<TexT, texture::RSPSpectrumTexture<T>>) {
                                auto rsp = tex.eval(tex_ctx);
                                auto albedo_spectrum = radiometry::RGBAlbedoSpectrumDistribution<
                                    T, radiometry::RGBSigmoidPolynomialNormalized>(rsp);
                                return albedo_spectrum.template sample<N>(wavelengths);
                            } else {
                                auto rgb = tex.eval(tex_ctx);
                                rgb = radiometry::RGB<T>(std::clamp(rgb.r(), T(0.05), T(0.95)),
                                                         std::clamp(rgb.g(), T(0.05), T(0.95)),
                                                         std::clamp(rgb.b(), T(0.05), T(0.95)));
                                auto albedo_spectrum = radiometry::create_srgb_albedo_spectrum(rgb);
                                return albedo_spectrum.template sample<N>(wavelengths);
                            }
                        },
                        source);
                }
            },
            m_reflectance_source);
        return BSDF<T, N>(si, shading, LambertianBxDF<T, N>(albedo));
    }
};

}  // namespace pbpt::material
