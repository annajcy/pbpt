#pragma once

#include "point.hpp"
#include "type_alias.hpp"
#include "vector.hpp"

namespace pbpt {
namespace math {

template<typename T, int N>
class Ray {
private:
    Point<T, N> m_origin{};
    Vec<T, N> m_direction{};

public:
    // --- 构造函数 ---
    constexpr Ray() noexcept 
        : m_origin(Point<T, N>::zeros()), m_direction(Vec<T, N>::zeros()) {
        m_direction.x() = 1.0; 
    }

    constexpr Ray(const Point<T, N>& origin, const Vec<T, N>& direction)
        : m_origin(origin), m_direction(direction.normalized()) {}

    constexpr Ray(const Point<T, N>& origin, const Point<T, N>& target)
        : m_origin(origin), m_direction((target - origin).normalized()) {}

    // --- 访问器 ---
    constexpr const Point<T, N>& origin() const noexcept { return m_origin; }
    constexpr const Vec<T, N>& direction() const noexcept { return m_direction; }

    // --- 核心方法 ---
    constexpr Point<T, N> at(T t) const noexcept {
        return m_origin + m_direction * t;
    }
};

// --- 类型别名 ---
using Ray3 = Ray<Float, 3>;
using Ray2 = Ray<Float, 2>;

} // namespace math
} // namespace pbpt