#pragma once

#include <cmath>
#include <vector>
#include "math/point.hpp"
#include "math/sampling.hpp"
#include "math/vector.hpp"

namespace pbpt::camera {

template <typename T, typename Derived>
class PixelFilter {
public:
    struct FilteredCameraSample {
        math::Point<T, 2> film_position;
        math::Vector<T, 2> offset;
        T weight;
    };

protected:
    T m_filter_radius{};
    
public:
    PixelFilter(T filter_radius = T(0.5)) : m_filter_radius(filter_radius) {}

    const T& filter_radius() const {
        return m_filter_radius;
    }

    T& filter_radius() {
        return m_filter_radius;
    }

    T sample_offset_pdf(const math::Vector<T, 2>& offset) {
        return as_derived().sample_offset_pdf_impl(offset);
    }

    T filter_kernel(const math::Vector<T, 2>& offset) {
        return as_derived().filter_kernel_impl(offset);
    }

    math::Vector<T, 2> sample_offset(const math::Point<T, 2>& uv) {
        return as_derived().sample_offset_impl(uv);
    }

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

    std::vector<math::Point<T, 2>> get_uv_samples(int u_sample_count, int v_sample_count) const {
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

protected:
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

template <typename T>
class BoxFilter : public PixelFilter<T, BoxFilter<T>> {
    friend class PixelFilter<T, BoxFilter<T>>;
    using PixelFilter<T, BoxFilter<T>>::m_filter_radius;

public:
    BoxFilter(T filter_radius = T(0.5)) : PixelFilter<T, BoxFilter<T>>(filter_radius) {}

    math::Vector<T, 2> sample_offset_impl(const math::Point<T, 2>& uv) const {
        return math::sample_uniform_2d(
            uv, 
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius), 
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius)
        ).to_vector();
    }

    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return math::sample_uniform_2d_pdf(
            math::Point<T, 2>(offset.x(), offset.y()),
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius),
            math::Vector<T, 2>(-m_filter_radius, m_filter_radius)
        );
    }

    T filter_kernel_impl(const math::Vector<T, 2>& offset) const {
        if (std::abs(offset.x()) > m_filter_radius || std::abs(offset.y()) > m_filter_radius) {
            return T(0);
        }
        return T(1);
    }
};

template <typename T>
class TentFilter : public PixelFilter<T, TentFilter<T>> {
    friend class PixelFilter<T, TentFilter<T>>;
    using PixelFilter<T, TentFilter<T>>::m_filter_radius;

public:
    TentFilter(T filter_radius = T(0.5)) : PixelFilter<T, TentFilter<T>>(filter_radius) {}

    math::Vector<T, 2> sample_offset_impl(const math::Point<T, 2>& uv) const {
        return math::sample_tent_2d(
            uv, 
            m_filter_radius, 
            m_filter_radius
        ).to_vector();
    }

    T sample_offset_pdf_impl(const math::Vector<T, 2>& offset) const {
        return math::sample_tent_2d_pdf(
            math::Point<T, 2>(offset.x(), offset.y()), 
            m_filter_radius, 
            m_filter_radius
        );
    }

    T filter_kernel_impl(const math::Vector<T, 2>& offset) const {
        T ax = std::abs(offset.x()) / m_filter_radius;
        T ay = std::abs(offset.y()) / m_filter_radius;
        if (ax > T(1) || ay > T(1)) {
            return T(0);
        }
        return (T(1) - ax) * (T(1) - ay);
    }
};

}
