#pragma once

#include "../global/type_alias.hpp"
#include "../global/function.hpp"  
#include "../global/operator.hpp"
#include "../global/utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <type_traits>
#include <iostream>

namespace pbpt::math {

template<typename T, int N>
requires (N > 0) && (std::is_floating_point_v<T> || std::is_integral_v<T>)
class Vector {
private:
    std::array<T, N> m_data{};

public:

    template<std::convertible_to<T> U>
    static constexpr Vector<T, N> filled(U value) noexcept {
        Vector<T, N> vec;
        vec.m_data.fill(static_cast<T>(value));
        return vec;
    }

    static constexpr Vector<T, N> zeros() noexcept { return filled(0.0); }
    static constexpr Vector<T, N> ones() noexcept { return filled(1.0); }

    static constexpr Vector<T, N> from_array(const std::array<T, N>& arr) {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i) result[i] = arr[i];
        return result;
    }

    constexpr std::array<T, N> to_array() const {
        return m_data;
    }

    constexpr Vector<T, N>() noexcept = default;

    template<std::convertible_to<T>... Args>
    requires(sizeof...(Args) == N)
    explicit constexpr Vector<T, N>(Args&&... args) noexcept  {
        m_data = {static_cast<T>(args)...};
    }

    template<std::convertible_to<T> U, int M>
    explicit constexpr Vector<T, N>(const Vector<U, M>& other) noexcept {
        auto casted = other.template cast<T, N>();
        m_data = casted.m_data;
    }
    
    constexpr T& x() noexcept requires(N > 0) { return m_data[0]; }
    constexpr T& y() noexcept requires(N > 1) { return m_data[1]; }
    constexpr T& z() noexcept requires(N > 2) { return m_data[2]; }
    constexpr T& w() noexcept requires(N > 3) { return m_data[3]; }

    constexpr const T& x() const noexcept requires(N > 0) { return m_data[0]; }
    constexpr const T& y() const noexcept requires(N > 1) { return m_data[1]; }
    constexpr const T& z() const noexcept requires(N > 2) { return m_data[2]; }
    constexpr const T& w() const noexcept requires(N > 3) { return m_data[3]; }

    constexpr int dims() const noexcept { return N; }

    constexpr const T& operator[](int index) const {
        assert_if([&index]() { return index < 0 || index >= N; }, "Index out of range");
        return m_data[index];
    }

    constexpr T& operator[](int index) {
        assert_if( [&index]() { return index < 0 || index >= N; }, "Index out of range");
        return m_data[index];
    }

    constexpr const T& at(int index) const {
        assert_if([&index]() { return index < 0 || index >= N; }, "Index out of range");
        return m_data[index];
    }

    constexpr Vector<T, N> operator-() const noexcept {
        Vector<T, N> result{};
        for (int i = 0; i < N; i++) result[i] = -(*this)[i];
        return result;
    }

    constexpr Vector<T, N>& operator+=(const Vector<T, N>& rhs) noexcept {
        for (int i = 0; i < N; i++) (*this)[i] += rhs[i];
        return *this;
    }

    constexpr Vector<T, N>& operator-=(const Vector<T, N>& rhs) noexcept {
        for (int i = 0; i < N; i++) (*this)[i] -= rhs[i];
        return *this;
    }

    constexpr Vector<T, N>& operator*=(const T& rhs) noexcept {
        for (int i = 0; i < N; i++) (*this)[i] *= rhs;
        return *this;
    }

    constexpr Vector<T, N>& operator*=(const Vector<T, N>& rhs) noexcept {
        for (int i = 0; i < N; i++) (*this)[i] *= rhs[i];
        return *this;
    }

    constexpr Vector<T, N>& operator/=(const Vector<T, N>& rhs) {
        for (int i = 0; i < N; i++) {
            assert_if([&rhs, i]() { return is_equal(rhs[i], 0.0); }, 
            "Division by zero in vector division");
            (*this)[i] /= rhs[i];
        }
        return *this;
    }

    template<typename U>
    constexpr bool operator==(const Vector<U, N>& rhs) const noexcept {
        for (int i = 0; i < N; i++) {
            if (is_not_equal(m_data[i], rhs[i])) return false;
        }
        return true;
    }

    template<typename U>
    constexpr bool operator!=(const Vector<U, N>& rhs) const noexcept {
        for (int i = 0; i < N; i++) {
            if (is_equal(m_data[i], rhs[i])) return false;
        }
        return true;
    }

    constexpr bool is_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_equal(m_data[i], 0.0)) return false;
        }
        return true;
    }

    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int i = 0; i < N; ++i)
                if (std::isnan(m_data[i])) return true;
        }
        return false;
    }

    constexpr T length_squared() const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) 
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr auto length() const {
        return std::sqrt(static_cast<promote_scalar_t<T>>(length_squared()));
    }

    constexpr auto normalized() const {
        using ResultType = promote_scalar_t<T>;
        Vector<ResultType, N> result = *this;
        auto len = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero vector");
        for (int i = 0; i < N; i++) result[i] /= static_cast<ResultType>(len);
        return result;
    }

    constexpr bool is_normalized() const {
        return is_equal(length(), T(1.0));
    }
    
    template<std::convertible_to<T> U>
    constexpr T dot(const Vector<U, N>& rhs) const noexcept {
        T result = 0;
        for (int i = 0; i < N; i++) result += (*this)[i] * static_cast<T>(rhs[i]);
        return result;
    }

    constexpr T product() const noexcept {
        T result = 1;
        for (int i = 0; i < N; i++) result *= (*this)[i];
        return result;
    }

    template <std::invocable<T&, int> F>
    constexpr void apply(F&& f) {
        for (int i = 0; i < N; ++i) f((*this)[i], i);
    }

    template<typename U>
    constexpr auto operator+(const Vector<U, N>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>((*this)[i]) + static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template<typename U>
    constexpr auto operator-(const Vector<U, N>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>((*this)[i]) - static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template<typename U>
    requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Vector<T, N>& rhs) noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>(value) * static_cast<ResultType>(rhs.m_data[i]);
        }
        return result;
    }

    template<typename U>
    constexpr auto operator*(U value) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>(this->m_data[i]) * static_cast<ResultType>(value);
        }
        return result;
    }

    template<typename U>
    constexpr auto operator/(U value) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&value]() { return is_equal(value, 0.0); }, "Division by zero in vector division");
            result[i] = static_cast<ResultType>(this->m_data[i]) / static_cast<ResultType>(value);
        }
        return result;
    }

    template<typename U>
    constexpr auto operator*(const Vector<U, N>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>((*this)[i]) * static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    template<typename U>
    constexpr auto operator/(const Vector<U, N>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<ResultType>((*this)[i]) / static_cast<ResultType>(rhs[i]);
        }
        return result;
    }

    constexpr auto inv() const {
        using ResultType = promote_scalar_t<T>;
        Vector<ResultType, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&]() { return is_equal((*this)[i], 0.0); }, "Cannot invert zero component");
            result[i] = static_cast<ResultType>(1) / (*this)[i];
        }
        return result;
    }

    constexpr auto abs() const noexcept { 
        Vector<T, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = pbpt::math::abs((*this)[i]);
        }
        return result;
    }

    int max_dim() const {
        int max_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], (*this)[max_i])) max_i = i;
        return max_i;
    }

    T max() const {
        T max_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], max_v)) max_v = (*this)[i];
        return max_v;
    }

    int min_dim() const {
        int min_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], (*this)[min_i])) min_i = i;
        return min_i;
    }

    T min() const {
        T min_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], min_v)) min_v = (*this)[i];
        return min_v;
    }

    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector<T, N> permuted(Args ...args) const {
        Vector<T, N> result;
        int i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    template<typename ...Args>
    requires (sizeof...(Args) == N)
    constexpr Vector<T, N>& permute(Args ...args) {
        *this = permuted(args...);
        return *this;
    }

    template<typename U, int M>
    requires (M > 0) && (std::is_convertible_v<T, U>)
    constexpr Vector<U, M> cast() const {
        Vector<U, M> result;
        for (int i = 0; i < std::min(N, M); i++) result[i] = static_cast<U>((*this)[i]);
        return result;
    }

    template<typename U>
    requires (std::is_convertible_v<T, U>)
    constexpr Vector<U, N> type_cast() const {
        return cast<U, N>();
    }

    template<int M>
    requires (M > 0)
    constexpr Vector<T, M> dim_cast() const {
        return cast<T, M>();
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector& vec) {
        os << "Vec" << N << "(";
        for (int i = 0; i < N; ++i) {
            os << vec[i] << (i == N - 1 ? "" : ", ");
        }
        os << ')';
        return os;
    }
};

template<typename T>
constexpr Vector<T, 3> cross(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs) noexcept {
    return Vector<T, 3>(
        std::fma(lhs.y(), rhs.z(), -lhs.z() * rhs.y()),
        std::fma(lhs.z(), rhs.x(), -lhs.x() * rhs.z()),
        std::fma(lhs.x(), rhs.y(), -lhs.y() * rhs.x())
    );
}

template<typename T>
constexpr promote_scalar_t<T> angle_between(const Vector<T, 3>& v1, const Vector<T, 3>& v2)  { 
    assert_if([&v1, &v2]() { return v1.is_zero() || v2.is_zero(); }, 
    "Cannot compute angle between zero vectors");
    assert_if([&v1, &v2]() { return !v1.is_normalized() || !v2.is_normalized(); },
    "Vectors must be normalized to compute angle between them");
    if (v1.dot(v2) < 0) {
        return pi_v<T> - 2 * safe_asin((v1 + v2).length() / 2);
    } else {
        return 2 * safe_asin((v1 - v2).length() / 2);
    }
}

template <std::floating_point T>
constexpr std::pair<Vector<T, 3>, Vector<T, 3>> coordinate_system(const Vector<T, 3>& v1) {
    assert_if([&v1]() { return !v1.is_normalized(); }, 
    "Input vector to coordinate_system_stable() must be normalized");

    const T sign = std::copysign(T(1), v1.z());
    const T a = T(-1) / (sign + v1.z());
    const T b = v1.x() * v1.y() * a;

    Vector<T, 3> v2(
        T(1) + sign * v1.x() * v1.x() * a,
        sign * b,
        -sign * v1.x()
    );

    Vector<T, 3> v3(
        b,
        sign + v1.y() * v1.y() * a,
        -v1.y()
    );

    return {v2.normalized(), v3.normalized()};
}

using Vec1 = Vector<Float, 1>;
using Vec2 = Vector<Float, 2>;
using Vec3 = Vector<Float, 3>;
using Vec4 = Vector<Float, 4>;

using Vec1i = Vector<Int, 1>;
using Vec2i = Vector<Int, 2>;
using Vec3i = Vector<Int, 3>;
using Vec4i = Vector<Int, 4>;

} // namespace math