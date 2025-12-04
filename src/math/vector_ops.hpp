#pragma once

#include <cmath>
#include <type_traits>

#include "tuple.hpp"

namespace pbpt::math {

/**
 * @brief CRTP mixin that adds vector-style operations to Tuple-like types.
 *
 * This template provides arithmetic operators, length, normalization
 * and dot product for any `Derived<T,N>` that stores its data in
 * `Tuple<Derived,T,N>`. It is used as a base for `Vector`, `Normal`
 * and `Homogeneous`.
 *
 * @tparam Derived CRTP derived template (e.g. Vector).
 * @tparam T       Scalar type.
 * @tparam N       Dimension.
 */
template <template <typename, int> typename Derived, typename T, int N>
class VectorOps : public Tuple<Derived, T, N> {
protected:
    /// Base tuple type that stores the underlying array.
    using Base = Tuple<Derived, T, N>;
    using Base::m_data;
    
public:
    using Base::Base;
    using Base::operator-;

    /// Component-wise addition-assignment with another vector-like object.
    template <typename U>
    constexpr auto& operator+=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] += static_cast<T>(rhs[i]);
        return *this;
    }

    /// Component-wise subtraction-assignment with another vector-like object.
    template <typename U>
    constexpr auto& operator-=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] -= static_cast<T>(rhs[i]);
        return *this;
    }

    /// Multiplies all components by a scalar in-place.
    template <typename U>
    constexpr auto& operator*=(const U& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs);
        return *this;
    }

    /// Divides all components by a scalar in-place (checking for zero).
    template <typename U>
    constexpr auto& operator/=(const U& rhs) {
        assert_if([&rhs]() { return is_equal(rhs, 0.0); }, "Division by zero in vector division");
        for (int i = 0; i < N; i++)
            (*this)[i] /= static_cast<T>(rhs);
        return *this;
    }

    /// Component-wise multiplication-assignment with another vector-like object.
    template <typename U>
    constexpr auto& operator*=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++)
            (*this)[i] *= static_cast<T>(rhs[i]);
        return *this;
    }

    /// Component-wise division-assignment with another vector-like object.
    template <typename U>
    constexpr auto& operator/=(const Derived<U, N>& rhs) {
        for (int i = 0; i < N; i++) {
            assert_if([&rhs, i]() { return is_equal(rhs[i], 0.0); }, "Division by zero in vector division");
            (*this)[i] /= static_cast<T>(rhs[i]);
        }
        return *this;
    }

    /// Squared Euclidean length of the vector.
    constexpr auto length_squared() const {
        auto result = T(0);
        for (int i = 0; i < N; i++)
            result += m_data[i] * m_data[i];
        return result;
    }

    /// Euclidean length (norm) of the vector.
    constexpr auto length() const { return std::sqrt(static_cast<promote_int_to_float_t<T>>(length_squared())); }

    /**
     * @brief Returns a normalized copy of the vector.
     *
     * The result has unit length. An assertion guards against
     * normalizing the zero vector.
     */
    constexpr auto normalized() const {
        using R             = promote_int_to_float_t<T>;
        Derived<R, N> result{};
        auto         len    = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero vector");
        for (int i = 0; i < N; i++)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(len);
        return result;
    }

    /// Returns true if the vector is approximately of unit length.
    constexpr bool is_normalized() const { 
        return is_equal(length(), T(1.0)); 
    }

    /// Dot product with another vector-like object.
    template <typename U>
    constexpr auto dot(const Derived<U, N>& rhs) const {
        using R  = std::common_type_t<T, U>;
        R result = 0;
        for (int i = 0; i < N; i++)
            result += static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    /// Product of all components.
    constexpr T product() const {
        T result = 1;
        for (int i = 0; i < N; i++)
            result *= (*this)[i];
        return result;
    }

    /// Component-wise addition; returns a new vector.
    template <typename U>
    constexpr auto operator+(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) + static_cast<R>(rhs[i]);
        }
        return result;
    }

    /// Component-wise subtraction; returns a new vector.
    template <typename U>
    constexpr auto operator-(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) - static_cast<R>(rhs[i]);
        }
        return result;
    }

    /// Scalar multiplication from the left (friend).
    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Derived<T, N>& rhs) {
        return rhs * value;
    }

    /// Scalar multiplication; returns a new vector.
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

    /// Scalar division; returns a new vector (checking for zero).
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

    /// Component-wise multiplication with another vector-like object.
    template <typename U>
    constexpr auto operator*(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        }
        return result;
    }

    /// Component-wise division with another vector-like object.
    template <typename U>
    constexpr auto operator/(const Derived<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Derived<R, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(rhs[i]);
        }
        return result;
    }

    /**
     * @brief Component-wise reciprocal (1/x_i) with floating-point promotion.
     *
     * An assertion guards against zero components.
     */
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
