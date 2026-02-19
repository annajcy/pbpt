#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "pbpt/texture/image.hpp"
#include "pbpt/texture/texture.hpp"

namespace pbpt::texture {

template <typename T, typename Texel>
class MipMap {
private:
    std::vector<Image<Texel>> m_levels;
    WrapMode m_wrap_u{WrapMode::Repeat};
    WrapMode m_wrap_v{WrapMode::Repeat};

public:
    MipMap() = default;

    MipMap(Image<Texel> level0, WrapMode wrap_u = WrapMode::Repeat, WrapMode wrap_v = WrapMode::Repeat)
        : m_wrap_u(wrap_u), m_wrap_v(wrap_v) {
        build_levels(std::move(level0));
    }

    std::size_t level_count() const { return m_levels.size(); }

    const Image<Texel>& level(std::size_t index) const { return m_levels.at(index); }

    Texel sample(const TextureEvalContext<T>& ctx, FilterMode = FilterMode::Trilinear) const {
        return sample_trilinear(ctx);
    }

private:
    static Texel lerp_texel(const Texel& a, const Texel& b, T t) { return a * (T(1) - t) + b * t; }

    static T wrap_coord(T x, WrapMode mode) {
        if (mode == WrapMode::Repeat) {
            T wrapped = x - std::floor(x);
            if (wrapped < T(0))
                wrapped += T(1);
            return wrapped;
        }
        return std::clamp(x, T(0), T(1));
    }

    static int wrap_index(int i, int size, WrapMode mode) {
        if (size <= 0)
            return 0;
        if (mode == WrapMode::Repeat) {
            int m = i % size;
            if (m < 0)
                m += size;
            return m;
        }
        return std::clamp(i, 0, size - 1);
    }

    const Texel& fetch_texel(const Image<Texel>& img, int x, int y) const {
        const int ix = wrap_index(x, img.width(), m_wrap_u);
        const int iy = wrap_index(y, img.height(), m_wrap_v);
        return img.get_pixel(ix, iy);
    }

    Texel sample_bilinear(std::size_t level_index, const math::Point<T, 2>& uv) const {
        const auto& img = m_levels[level_index];
        if (img.width() <= 0 || img.height() <= 0) {
            throw std::runtime_error("MipMap level has invalid resolution.");
        }

        const T u = wrap_coord(uv.x(), m_wrap_u);
        const T v = wrap_coord(uv.y(), m_wrap_v);
        const T s = u * static_cast<T>(img.width()) - T(0.5);
        const T t = v * static_cast<T>(img.height()) - T(0.5);

        const int x0 = static_cast<int>(std::floor(s));
        const int y0 = static_cast<int>(std::floor(t));
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;

        const T tx = s - static_cast<T>(x0);
        const T ty = t - static_cast<T>(y0);

        const auto& c00 = fetch_texel(img, x0, y0);
        const auto& c10 = fetch_texel(img, x1, y0);
        const auto& c01 = fetch_texel(img, x0, y1);
        const auto& c11 = fetch_texel(img, x1, y1);

        const auto c0 = lerp_texel(c00, c10, tx);
        const auto c1 = lerp_texel(c01, c11, tx);
        return lerp_texel(c0, c1, ty);
    }

    T estimate_lambda(const TextureEvalContext<T>& ctx) const {
        if (m_levels.empty())
            return T(0);
        if (!ctx.differentials.has_value())
            return T(0);

        const auto& d = *ctx.differentials;
        const T width = static_cast<T>(m_levels.front().width());
        const T height = static_cast<T>(m_levels.front().height());
        const T dudx = d.dudx * width;
        const T dvdx = d.dvdx * height;
        const T dudy = d.dudy * width;
        const T dvdy = d.dvdy * height;

        const T len_x = std::sqrt(dudx * dudx + dvdx * dvdx);
        const T len_y = std::sqrt(dudy * dudy + dvdy * dvdy);
        const T rho = std::max(std::max(len_x, len_y), T(1e-8));

        const T max_level = static_cast<T>(m_levels.size() - 1);
        return std::clamp(std::log2(rho), T(0), max_level);
    }

    Texel sample_trilinear(const TextureEvalContext<T>& ctx) const {
        if (m_levels.empty()) {
            throw std::runtime_error("MipMap has no levels.");
        }

        const T lambda = estimate_lambda(ctx);
        const int level0 = static_cast<int>(std::floor(lambda));
        const int level1 = std::min(level0 + 1, static_cast<int>(m_levels.size()) - 1);
        const T t = lambda - static_cast<T>(level0);

        const auto texel0 = sample_bilinear(static_cast<std::size_t>(level0), ctx.uv);
        if (level0 == level1 || std::abs(t) <= std::numeric_limits<T>::epsilon()) {
            return texel0;
        }
        const auto texel1 = sample_bilinear(static_cast<std::size_t>(level1), ctx.uv);
        return lerp_texel(texel0, texel1, t);
    }

    void build_levels(Image<Texel> level0) {
        if (level0.width() <= 0 || level0.height() <= 0) {
            throw std::runtime_error("MipMap input level0 must be non-empty.");
        }
        m_levels.clear();
        m_levels.push_back(std::move(level0));

        while (m_levels.back().width() > 1 || m_levels.back().height() > 1) {
            const auto& prev = m_levels.back();
            const int next_w = std::max(1, (prev.width() + 1) / 2);
            const int next_h = std::max(1, (prev.height() + 1) / 2);
            Image<Texel> next(next_w, next_h);

            for (int y = 0; y < next_h; ++y) {
                for (int x = 0; x < next_w; ++x) {
                    const int x0 = std::min(prev.width() - 1, 2 * x);
                    const int x1 = std::min(prev.width() - 1, x0 + 1);
                    const int y0 = std::min(prev.height() - 1, 2 * y);
                    const int y1 = std::min(prev.height() - 1, y0 + 1);

                    const Texel c00 = prev.get_pixel(x0, y0);
                    const Texel c10 = prev.get_pixel(x1, y0);
                    const Texel c01 = prev.get_pixel(x0, y1);
                    const Texel c11 = prev.get_pixel(x1, y1);
                    next.get_pixel(x, y) = (c00 + c10 + c01 + c11) / static_cast<T>(4);
                }
            }

            m_levels.push_back(std::move(next));
        }
    }
};

}  // namespace pbpt::texture
