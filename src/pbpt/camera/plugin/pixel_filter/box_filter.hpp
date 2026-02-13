#pragma once

#include "pbpt/camera/pixel_filter.hpp"

namespace pbpt::camera {

/**
 * @brief Box (uniform) pixel filter.
 *
 * Uses a uniform sampling distribution over a square region
 * [-r, r] × [-r, r], and a filter kernel that is 1 inside this region
 * and 0 outside.
 *
 * @tparam T Scalar type (e.g. float or double).
 */
template <typename T>
class BoxFilter : public PixelFilter<T, BoxFilter<T>> {
    friend class PixelFilter<T, BoxFilter<T>>;
    using PixelFilter<T, BoxFilter<T>>::m_filter_radius;

public:
    /**
     * @brief Construct a box filter with a given radius.
     *
     * The radius controls the half-width of the filter support in
     * both x and y directions.
     *
     * @param filter_radius Half-width of the box filter in pixels.
     */
    BoxFilter(T filter_radius = T(0.5)) : PixelFilter<T, BoxFilter<T>>(filter_radius) {}

    /**
     * @brief Sample an offset uniformly within the box support.
     *
     * @param uv 2D uniform sample in [0, 1)^2.
     * @return Offset in [-r, r] × [-r, r].
     */
    math::Vector<T, 2> sample_offset_impl(const math::Point<T, 2>& uv) const {
        return sampler::sample_uniform_2d(
            uv,
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius),
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius)
        ).to_vector();
    }

    /**
     * @brief PDF of the uniform box sampling distribution.
     *
     * @param offset Offset from the pixel center.
     * @return Probability density for @p offset.
     */
    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return sampler::sample_uniform_2d_pdf(
            math::Point<T, 2>(offset.x(), offset.y()),
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius),
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius)
        );
    }

    /**
     * @brief Box filter kernel.
     *
     * Returns 1 inside the support [-r, r] × [-r, r] and 0 outside.
     *
     * @param offset Offset from the pixel center.
     * @return Filter kernel value.
     */
    T filter_kernel_impl(const math::Vector<T, 2>& offset) const {
        if (std::abs(offset.x()) > m_filter_radius || std::abs(offset.y()) > m_filter_radius) {
            return T(0);
        }
        return T(1);
    }
};

} // namespace pbpt::camera
