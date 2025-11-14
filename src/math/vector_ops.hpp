#pragma once

#include <cmath>
#include <type_traits>

#include "tuple.hpp"

namespace pbpt::math {

template <template <typename, int> typename Derived, typename T, int N>
class VectorOps : public Tuple<Derived, T, N> {
protected:
    using Base = Tuple<Derived, T, N>;
    using Base::m_data;
    
public:
    using Base::Base;
    using Base::operator-;

    template <typename U>
    constexpr auto& operator+=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] += static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator-=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] -= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator*=(const U& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs);
        return *this;
    }

    template <typename U>
    constexpr auto& operator/=(const U& rhs) {
        assert_if([&rhs]() { return is_equal(rhs, 0.0); }, "Division by zero in vector division");
        for (int i = 0; i < N; i++)
            (*this)[i] /= static_cast<T>(rhs);
        return *this;
    }

    template <typename U>
    constexpr auto& operator*=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs[i]);
        return *this;
    }

    template <typename U>
    constexpr auto& operator/=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++) {
            assert_if([&rhs, i]() { return is_equal(rhs[i], 0.0); }, "Division by zero in vector division");
            (*this)[i] /= static_cast<T>(rhs[i]);
        }
        return *this;
    }

    constexpr auto length_squared() const {
        auto result = T(0);
        for (int i = 0; i < N; i++)
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr auto length() const { return std::sqrt(static_cast<promote_int_to_float_t<T>>(length_squared())); }

    constexpr auto normalized() const {
        using R             = promote_int_to_float_t<T>;
        Derived<R, N> result{};
        auto         len    = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero vector");
        for (int i = 0; i < N; i++)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(len);
        return result;
    }

    constexpr bool is_normalized() const { 
        return is_equal(length(), T(1.0)); 
    }

    template <typename U>
    constexpr auto dot(const Derived<U, N>& rhs) const {
        using R  = std::common_type_t<T, U>;
        R result = 0;
        for (int i = 0; i < N; i++)
            result += static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    constexpr T product() const {
        T result = 1;
        for (int i = 0; i < N; i++)
            result *= (*this)[i];
        return result;
    }

    template <typename U>
    constexpr auto operator+(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) + static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) - static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Derived<T, N>& rhs) {
        return rhs * value;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator*(U value) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>(this->m_data[i]) * static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator/(U value) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&value]() { return is_equal(value, 0.0); }, "Division by zero in vector division");
            result[i] = static_cast<R>(this->m_data[i]) / static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator/(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(rhs[i]);
        }
        return result;
    }

    constexpr auto inv() const {
        using R = promote_int_to_float_t<T>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            assert_if([&]() { return is_equal((*this)[i], 0.0); }, "Cannot invert zero component");
            result[i] = static_cast<R>(1) / (*this)[i];
        }
        return result;
    }
};

}  // namespace pbpt::math
