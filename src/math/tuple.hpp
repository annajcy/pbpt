#pragma once

#include <array>
#include <type_traits>

#include "operator.hpp"
#include "utils.hpp"

namespace pbpt::math {

/**
 * @brief Generic fixed-size tuple of numeric values.
 *
 * This CRTP base provides storage and common utilities for vector-,
 * point-, and normal-like types. It is specialized via a derived
 * template that wraps the underlying data.
 *
 * @tparam Derived CRTP derived template (e.g. Vector, Point).
 * @tparam T       Scalar type.
 * @tparam N       Dimension.
 */
template <template <typename, int> typename Derived, typename T, int N>
class Tuple {
protected:
    /// Underlying storage for the tuple components.
    std::array<T, N> m_data{};

public:
    /**
     * @brief Creates a tuple with all components set to @p value.
     */
    static constexpr Derived<T, N> filled(T value) {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = value;
        return t;
    }

    /// Returns a tuple with all components set to zero.
    static constexpr Derived<T, N> zeros() { return filled(T(0)); }
    /// Returns a tuple with all components set to one.
    static constexpr Derived<T, N> ones() { return filled(T(1)); }

    /**
     * @brief Creates a tuple filled with random values.
     *
     * Uses the `rand<T>()` helper from `utils.hpp`.
     */
    static Derived<T, N> random() {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = rand<T>();
        return t;
    }

    /**
     * @brief Creates a tuple from a std::array of values.
     */
    static constexpr Derived<T, N> from_array(const std::array<T, N>& arr) {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = arr[i];
        return t;
    }

    // -- Constructors --
    constexpr Tuple() = default;

    /// Constructs a tuple from N scalar arguments.
    template<typename ...Args>
        requires(sizeof...(Args) == N)
    explicit constexpr Tuple(Args&&... args) : m_data{ static_cast<T>(args)... } {}

    /**
     * @brief Converting copy-constructor from another Tuple-like type.
     *
     * Copies each component from @p other, converting to type @p T when needed.
     */
    template<template <typename, int> typename OtherDerived, typename U>
    constexpr Tuple(const Tuple<OtherDerived, U, N>& other) {
        for (int i = 0; i < N; ++i) {
            m_data[i] = static_cast<T>(other[i]);
        }
    }

    /// Returns a mutable reference to the underlying array.
    constexpr std::array<T, N>& to_array() { return m_data; }
    /// Returns a copy of the underlying array.
    constexpr std::array<T, N> to_array() const { return m_data; }
    /// Number of elements in the tuple.
    constexpr int dims() const { return N; }

    /// Returns true if any component is NaN (floating-point types only).
    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int i = 0; i < N; ++i)
                if (std::isnan(m_data[i]))
                    return true;
        }
        return false;
    }

    // -- Accessors --
    /// Indexed element access with bounds checking (mutable).
    constexpr T& operator[](int i) {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }

    /// Indexed element access with bounds checking (const).
    constexpr const T& operator[](int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }

    /// Bounds-checked read-only access to component @p i.
    constexpr const T& at(int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::at index out of range");
        return m_data[i];
    }

    /// Shortcut accessor for the first component (mutable).
    constexpr T& x() requires (N > 0) { return m_data[0]; }
    /// Shortcut accessor for the second component (mutable).
    constexpr T& y() requires (N > 1) { return m_data[1]; }
    /// Shortcut accessor for the third component (mutable).
    constexpr T& z() requires (N > 2) { return m_data[2]; }
    /// Shortcut accessor for the fourth component (mutable).
    constexpr T& w() requires (N > 3) { return m_data[3]; }

    /// Shortcut accessor for the first component (const).
    constexpr const T& x() const requires (N > 0) { return m_data[0]; }
    /// Shortcut accessor for the second component (const).
    constexpr const T& y() const requires (N > 1) { return m_data[1]; }
    /// Shortcut accessor for the third component (const).
    constexpr const T& z() const requires (N > 2) { return m_data[2]; }
    /// Shortcut accessor for the fourth component (const).
    constexpr const T& w() const requires (N > 3) { return m_data[3]; }

    /**
     * @brief Component-wise equality comparison with another Tuple.
     */
    template <typename U>
    constexpr bool operator==(const Tuple<Derived, U, N>& rhs) const {
        for (int i = 0; i < N; ++i)
            if (is_not_equal(m_data[i], rhs[i]))
                return false;
        return true;
    }

    /// Component-wise inequality comparison with another Tuple.
    template <typename U>
    constexpr bool operator!=(const Tuple<Derived, U, N>& rhs) const {
        return !(*this == rhs);
    }

    /// Unary minus: returns a tuple with all components negated.
    constexpr Derived<T, N> operator-() const {
        Derived<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = -(*this)[i];
        return result;
    }
   
    // -- Type Conversion --
    /**
     * @brief Casts the tuple to a different scalar type and dimension.
     *
     * The first min(N,M) components are converted; remaining components
     * are filled with zero when M > N.
     */
    template <typename U, int M>
        requires(M > 0) && std::is_convertible_v<T, U>
    constexpr Derived<U, M> cast() const {
        Derived<U, M> result;
        int k = std::min(N, M);
        for (int i = 0; i < k; ++i)
            result[i] = static_cast<U>(m_data[i]);
        for (int i = k; i < M; ++i)
            result[i] = U(0);
        return result;
    }

    /// Returns a tuple with all components converted to type U.
    template <typename U>
        requires std::is_convertible_v<T, U>
    constexpr Derived<U, N> type_cast() const {
        return cast<U, N>();
    }

    /// Returns a tuple with the same scalar type but a different dimension.
    template <int M>
        requires(M > 0)
    constexpr Derived<T, M> dim_cast() const {
        return cast<T, M>();
    }

    /// Visits each component with its index (mutable).
    template <std::invocable<T&, int> F>
    constexpr void visit(F&& f) {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    /// Visits each component with its index (const).
    template <std::invocable<const T&, int> F>
    constexpr void visit(F&& f) const {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    /// Returns the index of the largest component.
    int max_dim() const {
        int max_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], (*this)[max_i]))
                max_i = i;
        return max_i;
    }

    /// Returns the largest component value.
    T max() const {
        T max_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], max_v))
                max_v = (*this)[i];
        return max_v;
    }

    /// Returns the index of the smallest component.
    int min_dim() const {
        int min_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], (*this)[min_i]))
                min_i = i;
        return min_i;
    }

    /// Returns the smallest component value.
    T min() const {
        T min_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], min_v))
                min_v = (*this)[i];
        return min_v;
    }

    /// Returns a permuted copy of this tuple according to the given indices.
    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr Derived<T, N> permuted(Args... args) const {
        Derived<T, N> result;
        int          i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    /// Permutes components in-place according to the given indices.
    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr Derived<T, N>& permute(Args... args) {
        *this = static_cast<Tuple<Derived, T, N>>(permuted(args...));
        return static_cast<Derived<T, N>&>(*this);
    }

    /// Returns true if all components are zero (within numeric tolerance).
    constexpr bool is_all_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_zero(m_data[i]))
                return false;
        }
        return true;
    }

    /// Component-wise absolute value.
    constexpr Derived<T, N> abs() const {
        Derived<T, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = pbpt::math::abs((*this)[i]);
        }
        return result;
    }

    /// Sum of all components.
    constexpr T sum() const {
        T result = T(0);
        for (int i = 0; i < N; i++) {
            result += (*this)[i];
        }
        return result;
    }

    /// Average (mean) of all components.
    constexpr T average() const {
        T result = T(0);
        for (int i = 0; i < N; i++) {
            result += (*this)[i];
        }
        return result / N;
    }

    /**
     * @brief Clamps each component between low[i] and high[i].
     *
     * Defaults to the range [0,1] in every dimension.
     */
    constexpr Derived<T, N> clamp(
        const Derived<T, N>& low = Derived<T, N>::zeros(), 
        const Derived<T, N>& high = Derived<T, N>::ones()
    ) const {
        Derived<T, N> out;
        for (int i = 0; i < N; ++i) {
            auto v = this->m_data[i];
            auto l = static_cast<T>(low[i]);
            auto h = static_cast<T>(high[i]);
            out[i] = v < l ? l : (v > h ? h : v);
        }
        return out;
    }

    /// Returns this object as the derived type (const).
    constexpr const Derived<T, N>& as_derived() const {
        return *static_cast<const Derived<T, N>&>(*this);
    }

    /// Returns this object as the derived type (mutable).
    constexpr Derived<T, N>& as_derived() {
        return *static_cast<Derived<T, N>&>(*this);
    }
};

}  // namespace pbpt::math
