#pragma once

#include "camera/pixel_filter.hpp"

namespace pbpt::camera {

/**
 * @brief Tent (linear) pixel filter.
 *
 * Uses a separable 2D tent kernel
 * f(x, y) = (1 - |x|/r) (1 - |y|/r) for |x|, |y| ≤ r and 0 otherwise.
 * Sampling and PDF are matched to this kernel for importance sampling.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template <typename T>
class TentFilter : public PixelFilter<T, TentFilter<T>> {
    friend class PixelFilter<T, TentFilter<T>>;
    using PixelFilter<T, TentFilter<T>>::m_filter_radius;

public:
    /**
     * @brief Construct a tent filter with a given radius.
     *
     * The radius controls the support of the tent kernel in x and y
     * directions.
     *
     * @param filter_radius Support radius of the tent filter in pixels.
     */
    TentFilter(T filter_radius = T(0.5)) : PixelFilter<T, TentFilter<T>>(filter_radius) {}

    /**
     * @brief Sample an offset according to a 2D tent distribution.
     *
     * @param uv 2D uniform sample in [0, 1)^2.
     * @return Offset sampled from the tent distribution.
     */
    math::Vector<T, 2> sample_offset_impl(const math::Point<T, 2>& uv) const {
        return sampler::sample_tent_2d(
            uv,
            m_filter_radius,
            m_filter_radius
        ).to_vector();
    }

    /**
     * @brief PDF of the 2D tent sampling distribution.
     *
     * @param offset Offset from the pixel center.
     * @return Probability density for @p offset.
     */
    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return sampler::sample_tent_2d_pdf(
            math::Point<T, 2>(offset.x(), offset.y()),
            m_filter_radius,
            m_filter_radius
        );
    }

    /**
     * @brief Tent filter kernel.
     *
     * Returns (1 - |x|/r) (1 - |y|/r) when |x|, |y| ≤ r and 0 otherwise.
     *
     * @param offset Offset from the pixel center.
     * @return Filter kernel value.
     */
    T filter_kernel_impl(const math::Vector<T, 2>& offset) const {
        T ax = std::abs(offset.x()) / m_filter_radius;
        T ay = std::abs(offset.y()) / m_filter_radius;
        if (ax > T(1) || ay > T(1)) {
            return T(0);
        }
        return (T(1) - ax) * (T(1) - ay);
    }
};

} // namespace pbpt::camera
