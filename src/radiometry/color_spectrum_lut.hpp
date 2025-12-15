/**
 * @file
 * @brief Precomputed RGB-to-spectrum lookup using PBRT's rgb2spec tables.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>

#include "color.hpp"
#include "spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Wrapper around the 5D rgb2spec coefficient table.
 *
 * The table layout mirrors PBRT's design:
 *   [partition][z][y][x][coeff]
 * where partition selects the dominant RGB component to avoid
 * discontinuities, z is sampled non-uniformly, and x/y are linear.
 */
class RGBToSpectrumLUT {
public:
    static constexpr int res = 64;
    using CoefficientArray = float[3][res][res][res][3];

private:
    const float* m_z_nodes;
    const CoefficientArray* m_coeffs;

public:
    RGBToSpectrumLUT(const float* zNodes, const CoefficientArray* coeffs) :
        m_z_nodes(zNodes), m_coeffs(coeffs) {}

    /**
     * @brief Look up sigmoid-polynomial coefficients for an RGB triple.
     *
     * Returns normalized coefficients that can be fed directly into
     * @c RGBSigmoidPolynomialNormalized.
     */
    template <typename T>
    RGBSigmoidPolynomialNormalized<T> lookup(const RGB<T>& rgb) const {
        // Handle grayscale specially to avoid interpolation artifacts.
        if (rgb.r() == rgb.g() && rgb.g() == rgb.b()) {
            T v = rgb.r();
            if (v <= 0) {
                return RGBSigmoidPolynomialNormalized<T>(
                    -std::numeric_limits<T>::infinity(), 0, 0);
            }
            if (v >= 1) {
                return RGBSigmoidPolynomialNormalized<T>(
                    std::numeric_limits<T>::infinity(), 0, 0);
            }
            T c = (v - T(0.5)) / std::sqrt(v * (T(1) - v));
            return RGBSigmoidPolynomialNormalized<T>(c, 0, 0);
        }

        auto clamp_zero = [](T v) { return std::max<T>(0, v); };
        RGB<T> safe_rgb(clamp_zero(rgb.r()), clamp_zero(rgb.g()), clamp_zero(rgb.b()));

        int maxc = (safe_rgb[0] > safe_rgb[1])
            ? ((safe_rgb[0] > safe_rgb[2]) ? 0 : 2)
            : ((safe_rgb[1] > safe_rgb[2]) ? 1 : 2);

        float z = static_cast<float>(safe_rgb[maxc]);
        float x = static_cast<float>(safe_rgb[(maxc + 1) % 3]) * (res - 1) / z;
        float y = static_cast<float>(safe_rgb[(maxc + 2) % 3]) * (res - 1) / z;

        int xi = std::min(static_cast<int>(x), res - 2);
        int yi = std::min(static_cast<int>(y), res - 2);

        auto find_interval = [&](float value) {
            auto begin = m_z_nodes;
            auto end = m_z_nodes + res;
            auto upper = std::upper_bound(begin, end, value);
            int idx = static_cast<int>(std::max<ptrdiff_t>(0, upper - begin - 1));
            return std::min(idx, res - 2);
        };

        int zi = find_interval(z);
        float dx = x - xi;
        float dy = y - yi;
        float dz = (z - m_z_nodes[zi]) / (m_z_nodes[zi + 1] - m_z_nodes[zi]);

        std::array<float, 3> coeff{};
        for (int i = 0; i < 3; ++i) {
            auto co = [&](int dx_o, int dy_o, int dz_o) {
                return (*m_coeffs)[maxc][zi + dz_o][yi + dy_o][xi + dx_o][i];
            };

            float c00 = std::lerp(co(0, 0, 0), co(1, 0, 0), dx);
            float c10 = std::lerp(co(0, 1, 0), co(1, 1, 0), dx);
            float c01 = std::lerp(co(0, 0, 1), co(1, 0, 1), dx);
            float c11 = std::lerp(co(0, 1, 1), co(1, 1, 1), dx);

            float c0 = std::lerp(c00, c10, dy);
            float c1 = std::lerp(c01, c11, dy);
            coeff[i] = std::lerp(c0, c1, dz);
        }

        // PBRT stores coefficients in lambda space with quadratic term first.
        std::array<double, 3> unnormalized{
            static_cast<double>(coeff[2]),
            static_cast<double>(coeff[1]),
            static_cast<double>(coeff[0]),
        };

        constexpr double lambda0 = 360.0;
        constexpr double inv_range = 1.0 / (830.0 - 360.0);
        constexpr double inv_range_sq = inv_range * inv_range;
        constexpr double lambda_inv = lambda0 * inv_range;

        double A = unnormalized[2] / inv_range_sq;                                   // quadratic term
        double B = (unnormalized[1] + 2.0 * A * lambda0 * inv_range_sq) / inv_range; // linear term
        double C = unnormalized[0] + B * lambda_inv - A * lambda_inv * lambda_inv;   // constant term

        return RGBSigmoidPolynomialNormalized<T>(
            static_cast<T>(C), static_cast<T>(B), static_cast<T>(A)
        );
    }
};

///sRGB to spectrum lookup table data generated from PBRT's rgb2spec
extern const int sRGBToSpectrumTable_Res;
extern const float sRGBToSpectrumTable_Scale[64];
extern const float sRGBToSpectrumTable_Data[3][64][64][64][3];

/// Access the global sRGB rgb2spec lookup table.
inline const RGBToSpectrumLUT& srgb_to_spectrum_table() {
    static_assert(RGBToSpectrumLUT::res == 64, "Unexpected LUT resolution.");
    static const RGBToSpectrumLUT lut(sRGBToSpectrumTable_Scale, &sRGBToSpectrumTable_Data);
    return lut;
}

/**
 * @brief Convenience wrapper for sRGB lookups.
 */
template <typename T>
inline RGBSigmoidPolynomialNormalized<T> lookup_srgb_to_rsp(const RGB<T>& rgb) {
    return srgb_to_spectrum_table().lookup(rgb);
}

/**
 * @brief Create an albedo spectrum from an sRGB color using the fitted model by lookup LUT and interpolation.
 *
 * The resulting @c RGBAlbedoSpectrumDistribution encodes a reflectance
 * spectrum whose perceived color matches the given RGB (approximately)
 * under the reference illuminant.
 */
template<typename T>
radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized> create_srgb_albedo_spectrum(
    const radiometry::RGB<T>& rgb
) {
    return radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(
        lookup_srgb_to_rsp(rgb)
    );
}

}  // namespace pbpt::radiometry
