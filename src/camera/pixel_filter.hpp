/**
 * @file
 * @brief Pixel reconstruction filters and related sampling utilities.
 */
#pragma once

#include <array>
#include <cmath>
#include <vector>
#include "math/point.hpp"
#include "sampler/2d.hpp"
#include "math/vector.hpp"

namespace pbpt::camera {

/**
 * @brief CRTP base class for pixel filters.
 *
 * Provides a common interface for sampling film positions within a pixel
 * according to a reconstruction filter, and for evaluating the corresponding
 * filter kernel and sampling PDFs. The concrete sampling behavior is
 * implemented in the derived class.
 *
 * @tparam T       Scalar type (e.g. float or double).
 * @tparam Derived Concrete filter type using CRTP.
 */
template <typename T, typename Derived>
class PixelFilter {
public:
    /**
     * @brief Filtered camera sample on the film plane.
     *
     * Represents a sampled position on the film, the corresponding offset
     * from the pixel center, and the Monte Carlo weight associated with
     * the reconstruction filter.
     */
    struct FilteredCameraSample {
        /// Sampled film position (pixel center plus offset).
        math::Point<T, 2> film_position;
        /// Offset from the pixel center in film coordinates.
        math::Vector<T, 2> offset;
        /// Monte Carlo weight, typically f(x) / q(x) where f(x) is the reconstruction filter and q(x) is the sampling PDF.
        T weight;
    };

protected:
    /**
     * @brief Filter radius in pixels.
     *
     * For box filters this is the half width; for tent filters this is
     * the support radius. The filter kernel is usually non-zero only
     * inside [-m_filter_radius, m_filter_radius]^2.
     */
    T m_filter_radius{};
    
public:
    /**
     * @brief Construct a pixel filter with a given radius.
     * @param filter_radius Filter radius in pixels (default 0.5).
     */
    PixelFilter(T filter_radius = T(0.5)) : m_filter_radius(filter_radius) {}

    /**
     * @brief Get the filter radius (read-only).
     * @return Constant reference to the filter radius.
     */
    const T& filter_radius() const {
        return m_filter_radius;
    }

    /**
     * @brief Get the filter radius (writable).
     * @return Reference to the filter radius.
     */
    T& filter_radius() {
        return m_filter_radius;
    }

    /**
     * @brief Evaluate the sampling PDF of an offset.
     *
     * For a given offset from the pixel center, returns the probability
     * density under the sampling strategy used by the filter.
     *
     * @param offset Offset from the pixel center.
     * @return Probability density at the given offset.
     */
    T sample_offset_pdf(const math::Vector<T, 2>& offset) {
        return as_derived().sample_offset_pdf_impl(offset);
    }

    /**
     * @brief Evaluate the filter kernel at a given offset.
     *
     * @param offset Offset from the pixel center.
     * @return Filter kernel value f(offset).
     */
    T filter_kernel(const math::Vector<T, 2>& offset) {
        return as_derived().filter_kernel_impl(offset);
    }

    /**
     * @brief Sample an offset from the filter's sampling distribution.
     *
     * @param uv 2D uniform sample in [0, 1)^2.
     * @return Offset relative to the pixel center.
     */
    math::Vector<T, 2> sample_offset(const math::Point<T, 2>& uv) {
        return as_derived().sample_offset_impl(uv);
    }

    /**
     * @brief Sample a filtered film position within a pixel.
     *
     * Uses @p uv to sample an offset according to the filter's sampling
     * distribution, then computes the Monte Carlo weight f/q where f is
     * the filter kernel and q is the sampling PDF.
     *
     * @param pixel Integer pixel coordinates.
     * @param uv    2D uniform sample in [0, 1)^2.
     * @return Filtered camera sample containing film position, offset and weight.
     */
    FilteredCameraSample sample_film_position(
        const math::Point<int, 2>& pixel,
        const math::Point<T, 2>& uv
    ) const {
        math::Point<T, 2> pixel_center(
            static_cast<T>(pixel.x()) + T(0.5),
            static_cast<T>(pixel.y()) + T(0.5)
        );
        // Sample offset according to q, then compute MC weight f/q.
        auto offset = as_derived().sample_offset_impl(uv);
        T f = as_derived().filter_kernel_impl(offset);
        T q = as_derived().sample_offset_pdf_impl(offset);
        T weight = (q > T(0)) ? (f / q) : T(0);
        return {pixel_center + offset, offset, weight};
    }

    /**
     * @brief Generate a regular grid of UV samples over [0, 1)^2.
     *
     * The samples correspond to the centers of a @p u_sample_count by
     * @p v_sample_count grid in UV space.
     *
     * @param u_sample_count Number of samples in the u direction.
     * @param v_sample_count Number of samples in the v direction.
     * @return Vector of UV points in [0, 1)^2.
     */
    std::vector<math::Point<T, 2>> get_uv_samples(
        int u_sample_count, 
        int v_sample_count
    ) const {
        std::vector<math::Point<T, 2>> uvs;
        double u_stride = 1.0 / static_cast<T>(u_sample_count);
        double v_stride = 1.0 / static_cast<T>(v_sample_count);
        uvs.reserve(u_sample_count * v_sample_count);
        for (int v = 0; v < v_sample_count; ++v) {
            for (int u = 0; u < u_sample_count; ++u) {
                T u_coord = 0.5 * u_stride + static_cast<T>(u) * u_stride;
                T v_coord = 0.5 * v_stride + static_cast<T>(v) * v_stride;
                uvs.emplace_back(u_coord, v_coord);
            }
        }
        return uvs;
    }

    /**
    * @brief Generate a regular grid of UV samples over [0, 1)^2.
    *
    * The samples correspond to the centers of a @p USampleCount by
    * @p VSampleCount grid in UV space.
    *
    * @tparam USampleCount Number of samples in the u direction.
    * @tparam VSampleCount Number of samples in the v direction.
    * @return Array of UV points in [0, 1)^2.
    */
    template<int USampleCount, int VSampleCount>
    std::array<math::Point<T, 2>, USampleCount * VSampleCount> get_uv_samples() const {
        std::array<math::Point<T, 2>, USampleCount * VSampleCount> uvs{};
        double u_stride = 1.0 / static_cast<T>(USampleCount);
        double v_stride = 1.0 / static_cast<T>(VSampleCount);
        for (int v = 0; v < VSampleCount; ++v) {
            for (int u = 0; u < USampleCount; ++u) {
                T u_coord = 0.5 * u_stride + static_cast<T>(u) * u_stride;
                T v_coord = 0.5 * v_stride + static_cast<T>(v) * v_stride;
                uvs[v * USampleCount + u] = math::Point<T, 2>(u_coord, v_coord);
            }
        }
        return uvs;
    }

    /**
     * @brief Generate multiple filtered camera samples within a pixel.
     *
     * Builds a regular UV grid using @c get_uv_samples() and, for each
     * UV point, calls @c sample_film_position() to obtain a filtered
     * camera sample.
     *
     * @param pixel           Integer pixel coordinates.
     * @param u_sample_count  Number of samples in the u direction.
     * @param v_sample_count  Number of samples in the v direction.
     * @return Vector of filtered camera samples for the pixel.
     */
    std::vector<FilteredCameraSample> get_camera_samples(
        const math::Point<int, 2>& pixel,
        int u_sample_count,
        int v_sample_count
    ) const {
        std::vector<FilteredCameraSample> samples;
        auto uvs = get_uv_samples(u_sample_count, v_sample_count);
        samples.reserve(uvs.size());
        for (const auto& uv : uvs) {
            samples.push_back(sample_film_position(pixel, uv));
        }
        return samples;
    }

    /**
     * @brief Generate multiple filtered camera samples within a pixel.
     * @tparam USampleCount  Number of samples in the u direction.
     * @tparam VSampleCount  Number of samples in the v direction.
     * @param pixel           Integer pixel coordinates.
     * @return Array of filtered camera samples for the pixel.
     */
    template<int USampleCount, int VSampleCount>
    std::array<FilteredCameraSample, USampleCount * VSampleCount> get_camera_samples(
        const math::Point<int, 2>& pixel
    ) const {
        std::array<FilteredCameraSample, USampleCount * VSampleCount> samples;
        auto uvs = get_uv_samples<USampleCount, VSampleCount>();
        for (int i = 0; i < USampleCount * VSampleCount; ++i) {
            samples[i] = sample_film_position(pixel, uvs[i]);
        }
        return samples;
    }

protected:
    /**
     * @brief Access the derived implementation (non-const).
     *
     * Helper for CRTP to dispatch calls to the derived class.
     *
     * @return Reference to the derived type.
     */
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    /**
     * @brief Access the derived implementation (const).
     *
     * Helper for CRTP to dispatch calls to the derived class.
     *
     * @return Const reference to the derived type.
     */
    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

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
    T m_sigma;        ///< Standard deviation of the Gaussian.
    T m_alpha;        ///< Precomputed falloff coefficient: 1 / (2 * sigma^2).
    T m_exp_normalization; ///< Value to subtract to ensure continuity at edges (optional, usually 0 for pure importance sampling).

public:
    /**
     * @brief Construct a Gaussian filter.
     * * @param filter_radius The hard cutoff radius (samples outside this are discarded).
     * @param sigma         Standard deviation (controls the "blurriness"). 
     * A common choice is sigma = radius / 3.
     */
    GaussianFilter(T filter_radius = T(1.5), T sigma = T(0.5)) 
        : PixelFilter<T, GaussianFilter<T>>(filter_radius)
        , m_sigma(sigma) 
    {
        // Avoid division by zero
        if (m_sigma <= T(0)) m_sigma = T(1e-6);
        
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
        return sampler::sample_gaussian_2d(
            uv,
            math::Point<T, 2>(0, 0),      // Mean
            math::Vector<T, 2>(m_sigma, m_sigma) // Stddev
        ).to_vector();
    }

    /**
     * @brief PDF of the Gaussian sampling strategy.
     * Must match the distribution used in sample_offset_impl EXACTLY.
     *
     * @param offset Offset from pixel center.
     * @return PDF value.
     */
    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return sampler::sample_gaussian_2d_pdf(
            math::Point<T, 2>(offset.x(), offset.y()),
            math::Point<T, 2>(0, 0),
            math::Vector<T, 2>(m_sigma, m_sigma)
        );
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

};
