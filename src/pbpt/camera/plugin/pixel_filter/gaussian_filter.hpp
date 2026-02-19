#pragma once

#include "pbpt/camera/pixel_filter.hpp"

namespace pbpt::camera {

/**
 * @brief Gaussian pixel filter.
 *
 * Uses a Gaussian kernel f(x, y) = exp(-(x^2 + y^2) / (2 * sigma^2)).
 * The kernel is truncated at m_filter_radius.
 *
 * Samples are generated using an exact 2D Gaussian distribution.
 * Because the sampling PDF shape matches the kernel shape perfectly,
 * the Monte Carlo weight (f / p) is effectively constant inside the
 * valid radius, leading to very low variance.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template <typename T>
class GaussianFilter : public PixelFilter<T, GaussianFilter<T>> {
    friend class PixelFilter<T, GaussianFilter<T>>;
    using PixelFilter<T, GaussianFilter<T>>::m_filter_radius;

private:
    T m_sigma;              ///< Standard deviation of the Gaussian.
    T m_alpha;              ///< Precomputed falloff coefficient: 1 / (2 * sigma^2).
    T m_exp_normalization;  ///< Value to subtract to ensure continuity at edges (optional, usually 0 for pure
                            ///< importance sampling).

public:
    /**
     * @brief Construct a Gaussian filter.
     * * @param filter_radius The hard cutoff radius (samples outside this are discarded).
     * @param sigma         Standard deviation (controls the "blurriness").
     * A common choice is sigma = radius / 3.
     */
    GaussianFilter(T filter_radius = T(1.5), T sigma = T(0.5))
        : PixelFilter<T, GaussianFilter<T>>(filter_radius), m_sigma(sigma) {
        // Avoid division by zero
        if (m_sigma <= T(0))
            m_sigma = T(1e-6);

        // alpha = 1 / (2 * sigma^2)
        m_alpha = T(1) / (T(2) * m_sigma * m_sigma);
    }

    /**
     * @brief Sample an offset using the Gaussian sampler.
     * Note: This may generate samples outside m_filter_radius.
     * These will be rejected (weight=0) by the sample_film_position logic
     * because filter_kernel_impl will return 0.
     *
     * @param uv 2D uniform sample.
     * @return Offset vector.
     */
    math::Vector<T, 2> sample_offset_impl(const math::Point<T, 2>& uv) const {
        return sampler::sample_gaussian_2d(uv, math::Point<T, 2>(0, 0),          // Mean
                                           math::Vector<T, 2>(m_sigma, m_sigma)  // Stddev
                                           )
            .to_vector();
    }

    /**
     * @brief PDF of the Gaussian sampling strategy.
     * Must match the distribution used in sample_offset_impl EXACTLY.
     *
     * @param offset Offset from pixel center.
     * @return PDF value.
     */
    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return sampler::sample_gaussian_2d_pdf(math::Point<T, 2>(offset.x(), offset.y()), math::Point<T, 2>(0, 0),
                                               math::Vector<T, 2>(m_sigma, m_sigma));
    }

    /**
     * @brief Evaluate the Gaussian kernel.
     * Returns exp(-alpha * r^2) if r < radius, else 0.
     *
     * @param offset Offset from pixel center.
     * @return Kernel weight.
     */
    T filter_kernel_impl(const math::Vector<T, 2>& offset) const {
        // 1. Check support (Box approximation for efficiency, or circular check)
        // Since Gaussian is radially symmetric, circular check is more accurate for truncation,
        // but checking X and Y independently is faster and common in separable filters.
        // Here we strictly respect the radius.
        if (std::abs(offset.x()) > m_filter_radius || std::abs(offset.y()) > m_filter_radius) {
            return T(0);
        }

        // 2. Compute Gaussian value
        T r2 = offset.length_squared();

        // Strictly speaking, we should check if r2 > radius^2 for a circular filter,
        // but for a separable Gaussian filter implemented on a grid, checking x/y bounds is sufficient.
        // Let's stick to the box bounds consistent with the Box/Tent implementation style.

        return std::exp(-m_alpha * r2);
    }
};

}  // namespace pbpt::camera
