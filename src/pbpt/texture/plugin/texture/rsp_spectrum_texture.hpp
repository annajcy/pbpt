#pragma once

#include <filesystem>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/rgb_sigmoid.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/rgb_spectra.hpp"
#include "pbpt/texture/mipmap.hpp"
#include "pbpt/texture/texture.hpp"
#include "pbpt/utils/image_io.hpp"

namespace pbpt::texture {

template <typename T>
class RSPSpectrumTexture : public Texture<RSPSpectrumTexture<T>, T, radiometry::RGBSigmoidPolynomialNormalized<T>> {
    friend class Texture<RSPSpectrumTexture<T>, T, radiometry::RGBSigmoidPolynomialNormalized<T>>;

private:
    // Store coefficients as Vector3 for MipMap compatibility (c0, c1, c2)
    MipMap<T, math::Vector<T, 3>> m_mipmap{};

public:
    RSPSpectrumTexture() = default;

    RSPSpectrumTexture(Image<math::Vector<T, 3>> rgb_image, WrapMode wrap_u = WrapMode::Repeat,
                       WrapMode wrap_v = WrapMode::Repeat) {
        // Convert RGB to RSP coefficients
        Image<math::Vector<T, 3>> rsp_image(rgb_image.width(), rgb_image.height());
        for (int y = 0; y < rgb_image.height(); ++y) {
            for (int x = 0; x < rgb_image.width(); ++x) {
                const auto& rgb = rgb_image.get_pixel(x, y);
                // Clamp RGB to safe range as per previous logic in material, and then lookup
                // Note: material clamped to [0.05, 0.95] before lookup.
                // We should probably preserve that behavior or rely on lookup handling it?
                // lookup_srgb_to_rsp handles clamping to >=0.
                // LambertianMaterial previously clamped to [0.05, 0.95].
                // If we do pre-calculation, we should apply this clamp here to match behavior.
                auto clamped_rgb =
                    radiometry::RGB<T>(std::clamp(rgb.x(), T(0.05), T(0.95)), std::clamp(rgb.y(), T(0.05), T(0.95)),
                                       std::clamp(rgb.z(), T(0.05), T(0.95)));

                auto rsp = radiometry::lookup_srgb_to_rsp(clamped_rgb);
                rsp_image.get_pixel(x, y) = math::Vector<T, 3>(rsp.c0, rsp.c1, rsp.c2);
            }
        }
        m_mipmap = MipMap<T, math::Vector<T, 3>>(std::move(rsp_image), wrap_u, wrap_v);
    }

    RSPSpectrumTexture(const std::filesystem::path& filename, WrapMode wrap_u = WrapMode::Repeat,
                       WrapMode wrap_v = WrapMode::Repeat)
        : RSPSpectrumTexture(utils::read_image<T>(filename), wrap_u, wrap_v) {}

    radiometry::RGBSigmoidPolynomialNormalized<T> eval_impl(const TextureEvalContext<T>& ctx) const {
        const auto coeffs = m_mipmap.sample(ctx, FilterMode::Trilinear);
        return radiometry::RGBSigmoidPolynomialNormalized<T>(coeffs.x(), coeffs.y(), coeffs.z());
    }

    // Double conversion interfaces
    static RSPSpectrumTexture from_bitmap_image(Image<math::Vector<T, 3>> image, WrapMode wrap_u = WrapMode::Repeat,
                                                WrapMode wrap_v = WrapMode::Repeat) {
        return RSPSpectrumTexture(std::move(image), wrap_u, wrap_v);
    }

    Image<math::Vector<T, 3>> to_bitmap_image() const {
        if (m_mipmap.level_count() == 0)
            return {};
        const auto& base_level = m_mipmap.level(0);
        Image<math::Vector<T, 3>> rgb_image(base_level.width(), base_level.height());

        // We integrate against CIE D65 and XYZ matching functions to reconstruct RGB
        // Using sample<N> here might be faster than full integration if we choose N wisely,
        // but let's use the provided XYZ::from_reflectance which handles it correctly (likely tabulated).

        // This requires including standard_color_spaces.hpp and color.hpp
        // We do this fully qualified to avoid pollution if possible.

        for (int y = 0; y < base_level.height(); ++y) {
            for (int x = 0; x < base_level.width(); ++x) {
                const auto& coeffs = base_level.get_pixel(x, y);
                // Create normalized polynomial from coefficients
                radiometry::RGBSigmoidPolynomialNormalized<T> poly(coeffs.x(), coeffs.y(), coeffs.z());
                radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized> albedo(poly);

                // Get XYZ under D65
                auto xyz = radiometry::XYZ<T>::from_reflectance(albedo, radiometry::constant::CIE_D65_ilum<T>);

                // Convert XYZ to Linear RGB (sRGB primaries)
                auto rgb = radiometry::constant::sRGB<T>.to_rgb(xyz);

                // Clamp to [0,1] or safe range
                rgb_image.get_pixel(x, y) =
                    math::Vector<T, 3>(std::max(T(0), rgb.r()), std::max(T(0), rgb.g()), std::max(T(0), rgb.b()));
            }
        }
        return rgb_image;
    }
};

}  // namespace pbpt::texture
