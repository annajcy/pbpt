#pragma once

#include "vector.hpp"
#include "point.hpp"

#include <type_traits>

namespace pbpt::math {

template<typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T>)
class BoundingBox {
private:
    Point<T, N> m_min;
    Point<T, N> m_max;

public:

    constexpr BoundingBox() noexcept : 
    m_min(Point<T, N>::filled(std::numeric_limits<T>::max())), 
    m_max(Point<T, N>::filled(std::numeric_limits<T>::lowest())) {}

    template<typename ...Args>
    requires (std::is_same_v<Point<T, N>, Args> && ...) && (sizeof...(Args) > 0)
    constexpr explicit BoundingBox(const Args&... args) noexcept {
        // 将点解包到数组中以访问第一个元素
        const Point<T, N> points[] = {args...};
        m_min = points[0];
        m_max = points[0];
        // 从第二个元素开始合并
        for (size_t i = 1; i < sizeof...(args); ++i) {
            unite(points[i]);
        }
    }

    constexpr BoundingBox& unite(const Point<T, N>& point) noexcept {
        for (int i = 0; i < N; ++i) {
            m_min[i] = std::min(m_min[i], point[i]);
            m_max[i] = std::max(m_max[i], point[i]);
        }
        return *this;
    }

    constexpr BoundingBox& unite(const BoundingBox& box) noexcept {
        unite(box.min());
        unite(box.max());
        return *this;
    }

    constexpr BoundingBox united(const Point<T, N>& point) const noexcept {
        BoundingBox<T, N> box(*this);
        box.unite(point);
        return box;
    }

    constexpr BoundingBox& united(const BoundingBox& box) noexcept {
        BoundingBox<T, N> box_united(*this);
        box_united.unite(box.min());
        box_united.unite(box.max());
        return box_united;
    }

    constexpr bool is_overlapped(const BoundingBox& box) const noexcept {
        return contains(box.min()) || contains(box.max());
    }

    constexpr BoundingBox overlapped_box(const BoundingBox& box) const noexcept {
        BoundingBox<T, N> overlapped_box(*this);
        for (int i = 0; i < N; ++i) {
            overlapped_box.m_min[i] = std::max(overlapped_box.m_min[i], box.m_min[i]);
            overlapped_box.m_max[i] = std::min(overlapped_box.m_max[i], box.m_max[i]);
        }
        return overlapped_box;
    }

    constexpr bool contains(const Point<T, N>& point) const noexcept {
        for (int i = 0; i < N; ++i) {
            if (point[i] < m_min[i] || point[i] > m_max[i]) {
                return false;
            }
        }
        return true;
    }

    constexpr int max_extent() const noexcept {
        int max_extent = 0;
        T max_diff = m_max[0] - m_min[0];
        for (int i = 1; i < N; ++i) {
            T diff = m_max[i] - m_min[i];
            if (diff > max_diff) {
                max_diff = diff;
                max_extent = i;
            }
        }
        return max_extent;
    }

    constexpr Vector<T, N> offset(const Point<T, N>& p) const noexcept {
        Vector<T, N> offset;
        for (int i = 0; i < N; ++i) {
            offset[i] = (p[i] - m_min[i]) / (m_max[i] - m_min[i]);
        }
        return offset;
    }

    constexpr const Point<T, N>& min() const noexcept { return m_min; }
    constexpr const Point<T, N>& max() const noexcept { return m_max; }

    constexpr Point<T, N> center() const noexcept {
        return m_min.mid(m_max);
    }

    constexpr Vector<T, N> diagonal() const noexcept {
        return m_max - m_min;
    }

    constexpr T volume() const noexcept {
        return diagonal().product();
    }

    constexpr T surface_area() const noexcept requires (N == 3) {
        auto diag = diagonal();
        return 2 * (diag.x() * diag.y() + diag.y() * diag.z() + diag.z() * diag.x());
    }

    friend std::ostream& operator<<(std::ostream& os, const BoundingBox& box) {
        os << "BoundingBox(" << box.m_min << ", " << box.m_max << ")";
        return os;
    }
    
};

using Bound3 = BoundingBox<Float, 3>;
using Bound2 = BoundingBox<Float, 2>;

}

