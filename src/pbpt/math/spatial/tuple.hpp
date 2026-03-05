/**
 * @file
 * @brief CRTP base for fixed-size numeric tuples shared by vectors, points and normals.
 *
 * `Derived` is now a fully-instantiated concrete type (e.g. `Vector<float, 3>`),
 * not a template-template parameter.  Type-varying casts use `rebind_t<Derived,U,M>`
 * from `rebind.hpp`.  Compound-assignment helpers in the `protected` section are
 * used by Vector/Normal member operators in Phase 3.
 */
#pragma once

#include <array>
#include <cmath>
#include <type_traits>

#include "pbpt/math/basic/comparison.hpp"
#include "pbpt/math/basic/rebind.hpp"
#include "pbpt/math/basic/utils.hpp"

namespace pbpt::math {

/**
 * @brief Generic fixed-size tuple of numeric values.
 *
 * @tparam Derived  Fully-instantiated derived type (e.g. `Vector<float, 3>`).
 * @tparam T        Scalar element type.
 * @tparam N        Number of components.
 */
template <typename Derived, typename T, int N>
class Tuple {
protected:
    std::array<T, N> m_data{};

    // -----------------------------------------------------------------------
    // Protected compound-assignment helpers (used by Vector/Normal operators).
    // -----------------------------------------------------------------------

    template <typename OtherDerived, typename U>
    constexpr Derived& add_assign_impl(const Tuple<OtherDerived, U, N>& rhs) {
        for (int i = 0; i < N; ++i)
            m_data[i] += static_cast<T>(rhs[i]);
        return static_cast<Derived&>(*this);
    }

    template <typename OtherDerived, typename U>
    constexpr Derived& sub_assign_impl(const Tuple<OtherDerived, U, N>& rhs) {
        for (int i = 0; i < N; ++i)
            m_data[i] -= static_cast<T>(rhs[i]);
        return static_cast<Derived&>(*this);
    }

    template <typename U>
    constexpr Derived& mul_scalar_impl(U rhs) {
        for (int i = 0; i < N; ++i)
            m_data[i] *= static_cast<T>(rhs);
        return static_cast<Derived&>(*this);
    }

    template <typename U>
    constexpr Derived& div_scalar_impl(U rhs) {
        assert_if([&rhs]() { return is_equal(rhs, 0.0); }, "Division by zero in scalar division");
        for (int i = 0; i < N; ++i)
            m_data[i] /= static_cast<T>(rhs);
        return static_cast<Derived&>(*this);
    }

    template <typename OtherDerived, typename U>
    constexpr Derived& mul_vec_impl(const Tuple<OtherDerived, U, N>& rhs) {
        for (int i = 0; i < N; ++i)
            m_data[i] *= static_cast<T>(rhs[i]);
        return static_cast<Derived&>(*this);
    }

    template <typename OtherDerived, typename U>
    constexpr Derived& div_vec_impl(const Tuple<OtherDerived, U, N>& rhs) {
        for (int i = 0; i < N; ++i) {
            assert_if([&rhs, i]() { return is_equal(rhs[i], 0.0); }, "Division by zero in component division");
            m_data[i] /= static_cast<T>(rhs[i]);
        }
        return static_cast<Derived&>(*this);
    }

public:
    // -----------------------------------------------------------------------
    // Static factory methods
    // -----------------------------------------------------------------------

    /// Returns a tuple with all components set to @p value.
    static constexpr Derived filled(T value) {
        Derived t;
        for (int i = 0; i < N; ++i)
            t[i] = value;
        return t;
    }

    /// Returns a tuple with all components set to zero.
    static constexpr Derived zeros() { return filled(T(0)); }

    /// Returns a tuple with all components set to one.
    static constexpr Derived ones() { return filled(T(1)); }

    /// Returns a tuple filled with random values via `rand<T>()`.
    static Derived random() {
        Derived t;
        for (int i = 0; i < N; ++i)
            t[i] = rand<T>();
        return t;
    }

    /// Constructs a tuple from a `std::array`.
    static constexpr Derived from_array(const std::array<T, N>& arr) {
        Derived t;
        for (int i = 0; i < N; ++i)
            t[i] = arr[i];
        return t;
    }

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------

    constexpr Tuple() = default;

    /// All components set to the same scalar value.
    constexpr explicit Tuple(T value) {
        for (int i = 0; i < N; ++i)
            m_data[i] = value;
    }

    /// Component-wise construction from N scalar arguments.
    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr Tuple(Args&&... args) : m_data{static_cast<T>(args)...} {}

    /// Converting copy-constructor from any same-dimensioned Tuple-like type.
    template <typename OtherDerived, typename U>
    constexpr Tuple(const Tuple<OtherDerived, U, N>& other) {
        for (int i = 0; i < N; ++i)
            m_data[i] = static_cast<T>(other[i]);
    }

    /// Converting constructor from a Tuple-like type with different dimension.
    /// Copies `min(N,M)` components; zero-fills the rest.
    template <typename OtherDerived, typename U, int M>
        requires(M > 0)
    constexpr Tuple(const Tuple<OtherDerived, U, M>& other) {
        const int count = (N < M) ? N : M;
        for (int i = 0; i < count; ++i)
            m_data[i] = static_cast<T>(other[i]);
        for (int i = count; i < N; ++i)
            m_data[i] = T(0);
    }

    // -----------------------------------------------------------------------
    // Storage access
    // -----------------------------------------------------------------------

    constexpr std::array<T, N>& to_array() { return m_data; }
    constexpr std::array<T, N> to_array() const { return m_data; }
    constexpr int dims() const { return N; }

    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int i = 0; i < N; ++i)
                if (std::isnan(m_data[i]))
                    return true;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // Element access
    // -----------------------------------------------------------------------

    constexpr T& operator[](int i) {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }
    constexpr const T& operator[](int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }
    constexpr const T& at(int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::at index out of range");
        return m_data[i];
    }

    constexpr T& x() requires(N > 0) { return m_data[0]; }
    constexpr T& y() requires(N > 1) { return m_data[1]; }
    constexpr T& z() requires(N > 2) { return m_data[2]; }
    constexpr T& w() requires(N > 3) { return m_data[3]; }

    constexpr const T& x() const requires(N > 0) { return m_data[0]; }
    constexpr const T& y() const requires(N > 1) { return m_data[1]; }
    constexpr const T& z() const requires(N > 2) { return m_data[2]; }
    constexpr const T& w() const requires(N > 3) { return m_data[3]; }

    // -----------------------------------------------------------------------
    // Comparison
    // -----------------------------------------------------------------------

    /// Component-wise equality with a same-family tuple of possibly different scalar type.
    template <typename U>
    constexpr bool operator==(const rebind_t<Derived, U, N>& rhs) const {
        for (int i = 0; i < N; ++i)
            if (is_not_equal(m_data[i], rhs[i]))
                return false;
        return true;
    }

    template <typename U>
    constexpr bool operator!=(const rebind_t<Derived, U, N>& rhs) const {
        return !(*this == rhs);
    }

    // -----------------------------------------------------------------------
    // Arithmetic
    // -----------------------------------------------------------------------

    /// Unary negation.
    constexpr Derived operator-() const {
        Derived result;
        for (int i = 0; i < N; ++i)
            result[i] = -(*this)[i];
        return result;
    }

    // -----------------------------------------------------------------------
    // Type conversion
    // -----------------------------------------------------------------------

    /// Cast to a different scalar type U and dimension M.
    template <typename U, int M>
        requires(M > 0) && std::is_convertible_v<T, U>
    constexpr rebind_t<Derived, U, M> cast() const {
        rebind_t<Derived, U, M> result;
        const int k = (N < M) ? N : M;
        for (int i = 0; i < k; ++i)
            result[i] = static_cast<U>(m_data[i]);
        for (int i = k; i < M; ++i)
            result[i] = U(0);
        return result;
    }

    /// Cast to a different scalar type, keeping the same dimension.
    template <typename U>
        requires std::is_convertible_v<T, U>
    constexpr rebind_t<Derived, U, N> type_cast() const {
        return cast<U, N>();
    }

    /// Cast to a different dimension, keeping the same scalar type.
    template <int M>
        requires(M > 0)
    constexpr rebind_t<Derived, T, M> dim_cast() const {
        return cast<T, M>();
    }

    // -----------------------------------------------------------------------
    // Visitors
    // -----------------------------------------------------------------------

    template <std::invocable<T&, int> F>
    constexpr void visit(F&& f) {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    template <std::invocable<const T&, int> F>
    constexpr void visit(F&& f) const {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    // -----------------------------------------------------------------------
    // Min / Max
    // -----------------------------------------------------------------------

    int max_dim() const {
        int max_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], (*this)[max_i]))
                max_i = i;
        return max_i;
    }

    T max() const {
        T max_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], max_v))
                max_v = (*this)[i];
        return max_v;
    }

    int min_dim() const {
        int min_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], (*this)[min_i]))
                min_i = i;
        return min_i;
    }

    T min() const {
        T min_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], min_v))
                min_v = (*this)[i];
        return min_v;
    }

    // -----------------------------------------------------------------------
    // Permutation
    // -----------------------------------------------------------------------

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr Derived permuted(Args... args) const {
        Derived result;
        int i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr Derived& permute(Args... args) {
        *this = static_cast<Tuple<Derived, T, N>>(permuted(args...));
        return static_cast<Derived&>(*this);
    }

    // -----------------------------------------------------------------------
    // Utilities
    // -----------------------------------------------------------------------

    constexpr bool is_all_zero() const {
        for (int i = 0; i < N; ++i)
            if (!is_zero(m_data[i]))
                return false;
        return true;
    }

    constexpr Derived abs() const {
        Derived result{};
        for (int i = 0; i < N; ++i)
            result[i] = pbpt::math::abs((*this)[i]);
        return result;
    }

    constexpr T sum() const {
        T result = T(0);
        for (int i = 0; i < N; ++i)
            result += (*this)[i];
        return result;
    }

    constexpr T average() const {
        return sum() / static_cast<T>(N);
    }

    constexpr Derived clamp(const Derived& low = Derived::zeros(),
                            const Derived& high = Derived::ones()) const {
        Derived out;
        for (int i = 0; i < N; ++i) {
            const T v = m_data[i];
            const T l = static_cast<T>(low[i]);
            const T h = static_cast<T>(high[i]);
            out[i] = v < l ? l : (v > h ? h : v);
        }
        return out;
    }

    constexpr const Derived& as_derived() const { return *static_cast<const Derived*>(this); }
    constexpr Derived& as_derived() { return *static_cast<Derived*>(this); }
};

}  // namespace pbpt::math
