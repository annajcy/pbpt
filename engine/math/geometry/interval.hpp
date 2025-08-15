#pragma once

#include <concepts>
#include <algorithm>
#include <limits>
#include <optional>
#include <ostream>
#include "math/global/operator.hpp"
#include "math/global/utils.hpp"

namespace pbpt::math {

template <std::floating_point T>
class Interval {
    
public:
    T m_low{}, m_high{};

    // 默认构造：空区间
    constexpr Interval<T>() : m_low(1), m_high(0) {}

    template<typename U>
    constexpr explicit Interval(const Interval<U>& o)
        : m_low(static_cast<T>(o.m_low)), m_high(static_cast<T>(o.m_high)) {}

    // 单值构造
    constexpr Interval<T>(T v) : m_low(v), m_high(v) {}

    // 显式区间构造
    constexpr Interval<T>(T low, T high) {
        if (is_greater(low, high)) {
            m_low = 1;
            m_high = 0;
        } else {
            m_low = low;
            m_high = high;
        }
    }

    // 工厂函数：从值 ± ε 创建
    static constexpr Interval<T> from_value_with_error(T v, T eps) {
        return Interval(v - eps, v + eps);
    }

    static constexpr Interval<T> from_value(T v) {
        return Interval(v);
    }

    // 工厂函数：创建空区间
    static constexpr Interval<T> empty() {
        return Interval<T>(1, 0);
    }

    constexpr T midpoint() const {
        assert_if(is_empty(), "Cannot compute midpoint of an empty interval");
        return (m_low + m_high) * T(0.5);
    }

    constexpr T width() const {
        assert_if(is_empty(), "Cannot compute width of an empty interval");
        return is_empty() ? T(0) : m_high - m_low;
    }

    template<typename U>
    constexpr bool contains(U v) const {
        if (is_empty()) return false;
        return is_greater_equal(v, m_low) && is_less_equal(v, m_high);
    }

    constexpr bool is_empty() const {
        return is_greater(m_low, m_high);
    }

    constexpr bool is_point() const {
        return !is_empty() && is_equal(m_low, m_high);
    }

    template<typename U>
    constexpr bool is_intersected(const Interval<U>& other) const {
        if (is_empty() || other.is_empty()) return false;
        return is_greater_equal(m_high, other.m_low) && is_less_equal(m_low, other.m_high);
    }


    template<typename U>
    constexpr auto united(const Interval<U>& other) const {
        using R = std::common_type_t<T, U>;
        if (is_empty()) {
            return Interval<R>(other.m_low, other.m_high);
        }
        if (other.is_empty()) {
            return Interval<R>(m_low, m_high);
        }
        return Interval<R>(std::min<R>(m_low, other.m_low), std::max<R>(m_high, other.m_high));
    }

    template<typename U>
    constexpr auto intersection(const Interval<U>& other) const {
        using R = std::common_type_t<T, U>;
        if (!is_intersected(other)) {
            return Interval<R>::empty();
        }
        return Interval<R>(std::max<R>(m_low, other.m_low), std::min<R>(m_high, other.m_high));
    }

    // —— 空传播修正 ——
    template<typename U>
    constexpr auto operator+(const Interval<U>& other) const {
        using R = std::common_type_t<T, U>;
        if (is_empty() || other.is_empty()) return Interval<R>::empty();
        return Interval<R>(m_low + other.m_low, m_high + other.m_high);
    }

    template<typename U>
    constexpr auto operator-(const Interval<U>& other) const {
        using R = std::common_type_t<T, U>;
        if (is_empty() || other.is_empty()) return Interval<R>::empty();
        return Interval<R>(m_low - other.m_high, m_high - other.m_low);
    }

    template<typename U>
    constexpr auto operator+(U scalar) const {
        using R = std::common_type_t<T, U>;
        if (is_empty()) return Interval<R>::empty();
        return Interval<R>(m_low + scalar, m_high + scalar);
    }

    template<typename U>
    constexpr auto operator-(U scalar) const {
        using R = std::common_type_t<T, U>;
        if (is_empty()) return Interval<R>::empty();
        return Interval<R>(m_low - scalar, m_high - scalar);
    }

    // —— 乘法 / 除法 ——
    template<typename U>
    constexpr auto operator*(const Interval<U>& other) const {
        using R = std::common_type_t<T, U>;
        if (is_empty() || other.is_empty()) return Interval<R>::empty();
        const R a = static_cast<R>(m_low),  b = static_cast<R>(m_high);
        const R c = static_cast<R>(other.m_low), d = static_cast<R>(other.m_high);
        const R p1 = a * c, p2 = a * d, p3 = b * c, p4 = b * d;
        return Interval<R>(std::min({p1,p2,p3,p4}), std::max({p1,p2,p3,p4}));
    }

    template<typename U>
    constexpr auto operator*(U s) const {
        using R = std::common_type_t<T, U>;
        if (is_empty()) return Interval<R>::empty();
        const R a = static_cast<R>(m_low), b = static_cast<R>(m_high), k = static_cast<R>(s);
        return (k >= R(0)) ? Interval<R>(a * k, b * k) : Interval<R>(b * k, a * k);
    }

    template<typename U>
    constexpr auto operator/(U s) const {
        using R = std::common_type_t<T, U>;
        if (is_empty()) return Interval<R>::empty();
        const R k = static_cast<R>(s);
        assert_if(is_equal(k, R(0)), "Division by zero in Interval");
        return (k > R(0))
            ? Interval<R>(static_cast<R>(m_low) / k, static_cast<R>(m_high) / k)
            : Interval<R>(static_cast<R>(m_high) / k, static_cast<R>(m_low)  / k);
    }

    // —— 便捷扩张 —— 
    constexpr Interval expand(T eps) const {
        return is_empty() ? *this : Interval(m_low - eps, m_high + eps);
    }

    // 复合赋值运算符
    template<typename U>
    constexpr Interval& operator+=(const Interval<U>& other) {
        *this = *this + other;
        return *this;
    }

    template<typename U>
    constexpr Interval& operator-=(const Interval<U>& other) {
        *this = *this - other;
        return *this;
    }

    template<std::floating_point U>
    constexpr Interval& operator+=(U scalar) {
        *this = *this + scalar;
        return *this;
    }

    template<std::floating_point U>
    constexpr Interval& operator-=(U scalar) {
        *this = *this - scalar;
        return *this;
    }

    template<typename U>
    constexpr bool operator<(const Interval<U>& other) const {
        if (is_empty() || other.is_empty()) return false;
        return is_less(m_high, other.m_low);
    }


    template<typename U>
    constexpr bool operator<=(const Interval<U>& other) const {
        if (is_empty() || other.is_empty()) return false;
        return is_less_equal(m_high, other.m_low);           
    }

    template<typename U>
    constexpr bool operator>(const Interval<U>& other) const {
        if (is_empty() || other.is_empty()) return false;
        return is_greater(m_low, other.m_high);
    }

    template<typename U>
    constexpr bool operator>=(const Interval<U>& other) const {
        if (is_empty() || other.is_empty()) return false;
        return is_greater_equal(m_low, other.m_high);  
    }

    template<typename U>
    constexpr bool operator==(const Interval<U>& other) const {
        if (is_empty() && other.is_empty()) return true;
        return !is_empty() && !other.is_empty() && is_equal(m_low, other.m_low) && is_equal(m_high, other.m_high);
    }

    template<typename U>
    constexpr bool operator!=(const Interval<U>& other) const {
        return !(*this == other);
    }

    // 标量比较
    template<std::floating_point U>
    constexpr bool operator<(U scalar) const {
        if (is_empty()) return false;
        return is_less(m_high, scalar);
    }

    template<std::floating_point U>
    constexpr bool operator<=(U scalar) const {
        if (is_empty()) return false;
        return is_less_equal(m_high, scalar);
    }

    template<std::floating_point U>
    constexpr bool operator>(U scalar) const {
        if (is_empty()) return false;
        return is_greater(m_low, scalar);
    }

    template<std::floating_point U>
    constexpr bool operator>=(U scalar) const {
        if (is_empty()) return false;
        return is_greater_equal(m_low, scalar);
    }

    constexpr Interval operator-() const {
        if (is_empty()) return *this;
        return Interval(-m_high, -m_low);
    }

    friend std::ostream& operator<<(std::ostream& os, const Interval& t) {
        if (t.is_empty()) {
            os << "Interval(empty)";
            return os;
        }

        os << "Interval(" << t.m_low << ", " << t.m_high << ")";
        return os;
    }

};

// 全局操作符 - 标量在左侧的情况
template<std::floating_point U, std::floating_point T>
constexpr auto operator+(U scalar, const Interval<T>& interval) {
    return interval + scalar;
}

template<std::floating_point U, std::floating_point T2>
constexpr auto operator-(U scalar, const Interval<T2>& interval) {
    using R = std::common_type_t<T2, U>;
    if (interval.is_empty()) return Interval<R>::empty();
    return Interval<R>(scalar - interval.m_high, scalar - interval.m_low);
}

using Interv = Interval<Float>;

} // namespace pbpt::math
