/**
 * @file
 * @brief Surface normal vector type and helpers.
 */
#pragma once

#include <type_traits>

#include "pbpt/math/basic/algebra_concepts.hpp"
#include "pbpt/math/basic/comparison.hpp"
#include "pbpt/math/spatial/tuple.hpp"
#include "pbpt/math/spatial/vector.hpp"

namespace pbpt::math {

/**
 * @brief Geometric surface normal vector.
 *
 * Semantically distinct from a direction `Vector`.  Satisfies
 * `CovectorSpace<Normal, Vector, T>`: `dot(Normal, Vector)` is defined
 * but `dot(Normal, Normal)` is deliberately NOT declared.
 *
 * @tparam T  Scalar type.
 * @tparam N  Dimension.
 */
template <typename T, int N>
class Normal : public Tuple<Normal<T, N>, T, N> {
private:
    using Base = Tuple<Normal<T, N>, T, N>;

protected:
    using Base::m_data;

public:
    using Base::Base;
    using Base::operator-;

    // -----------------------------------------------------------------------
    // Compound assignment
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto& operator+=(const Normal<U, N>& rhs) {
        return this->add_assign_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator-=(const Normal<U, N>& rhs) {
        return this->sub_assign_impl(rhs);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator*=(U rhs) {
        return this->mul_scalar_impl(rhs);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto& operator/=(U rhs) {
        return this->div_scalar_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator*=(const Normal<U, N>& rhs) {
        return this->mul_vec_impl(rhs);
    }

    template <typename U>
    constexpr auto& operator/=(const Normal<U, N>& rhs) {
        return this->div_vec_impl(rhs);
    }

    // -----------------------------------------------------------------------
    // Binary arithmetic (return new object)
    // -----------------------------------------------------------------------

    template <typename U>
    constexpr auto operator+(const Normal<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) + static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Normal<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) - static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator*(U value) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>(m_data[i]) * static_cast<R>(value);
        return result;
    }

    /// Left scalar multiply (friend non-member).
    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(U value, const Normal<T, N>& rhs) {
        return rhs * value;
    }

    template <typename U>
    constexpr auto operator/(U value) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i) {
            assert_if([&value]() { return is_equal(value, 0.0); }, "Division by zero in normal division");
            result[i] = static_cast<R>(m_data[i]) / static_cast<R>(value);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Normal<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    template <typename U>
    constexpr auto operator/(const Normal<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(rhs[i]);
        return result;
    }

    // -----------------------------------------------------------------------
    // Length / normalisation
    // -----------------------------------------------------------------------

    constexpr auto length_squared() const {
        T result = T(0);
        for (int i = 0; i < N; ++i)
            result += m_data[i] * m_data[i];
        return result;
    }

    constexpr auto length() const {
        return std::sqrt(static_cast<promote_int_to_float_t<T>>(length_squared()));
    }

    constexpr auto normalized() const {
        using R = promote_int_to_float_t<T>;
        Normal<R, N> result{};
        auto len = length();
        assert_if([&len]() { return is_equal(len, 0.0); }, "Cannot normalize a zero normal");
        for (int i = 0; i < N; ++i)
            result[i] = static_cast<R>((*this)[i]) / static_cast<R>(len);
        return result;
    }

    constexpr bool is_normalized() const { return is_equal(length(), T(1.0)); }

    // -----------------------------------------------------------------------
    // Dot product — covector semantics (Normal · Vector only, NOT Normal · Normal)
    // -----------------------------------------------------------------------

    /// Dot product with a direction vector (covector semantics).
    template <typename U>
    constexpr auto dot(const Vector<U, N>& rhs) const {
        using R = std::common_type_t<T, U>;
        R result = R(0);
        for (int i = 0; i < N; ++i)
            result += static_cast<R>((*this)[i]) * static_cast<R>(rhs[i]);
        return result;
    }

    /// dot(Normal, Normal): deliberately NOT declared.
    /// Absence (not deletion) ensures !InnerProduct<Normal,S> in algebra_concepts.

    constexpr T product() const {
        T result = T(1);
        for (int i = 0; i < N; ++i)
            result *= (*this)[i];
        return result;
    }

    /// Component-wise reciprocal with float promotion.
    constexpr auto inv() const {
        using R = promote_int_to_float_t<T>;
        Normal<R, N> result{};
        for (int i = 0; i < N; ++i) {
            assert_if([&]() { return is_equal((*this)[i], 0.0); }, "Cannot invert zero component");
            result[i] = static_cast<R>(1) / static_cast<R>((*this)[i]);
        }
        return result;
    }

    // -----------------------------------------------------------------------
    // Normal-specific operations
    // -----------------------------------------------------------------------

    /// Constructs a normal from a plain vector by copying components.
    static constexpr Normal from_vector(const Vector<T, N>& vec) {
        Normal n;
        for (int i = 0; i < N; ++i)
            n[i] = vec[i];
        return n;
    }

    /// Converts this normal to a direction vector with the same components.
    constexpr Vector<T, N> to_vector() const {
        return Vector<T, N>::from_array(this->to_array());
    }

    /**
     * @brief Returns a copy oriented to face `vec`.
     *
     * Flips the normal if `dot(vec, this->to_vector()) < 0`.
     */
    constexpr Normal face_forward(const Vector<T, N>& vec) const {
        if (is_less(vec.dot(this->to_vector()), 0.0)) {
            Normal result = (*this);
            for (int i = 0; i < N; ++i)
                result[i] = -m_data[i];
            return result;
        }
        return (*this);
    }

    /// Returns a copy of the normal with all components negated.
    Normal flipped() const {
        Normal result = (*this);
        for (int i = 0; i < N; ++i)
            result[i] = -m_data[i];
        return result;
    }

    /// Negates all components in-place.
    Normal& flip() {
        for (int i = 0; i < N; ++i)
            m_data[i] = -m_data[i];
        return *this;
    }
};

// ---------------------------------------------------------------------------
// rebind_trait specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct rebind_trait<Normal<T, N>> {
    template <typename U, int M>
    using type = Normal<U, M>;
};

// ---------------------------------------------------------------------------
// algebra_traits specialization
// ---------------------------------------------------------------------------

template <typename T, int N>
struct algebra_traits<Normal<T, N>> {
    using scalar = T;
    static constexpr int dim = N;
};

// ---------------------------------------------------------------------------
// zero_v specialization (required by VectorSpace — normals form a vector space)
// ---------------------------------------------------------------------------

template <typename T, int N>
inline constexpr Normal<T, N> zero_v<Normal<T, N>> = Normal<T, N>::zeros();

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// length(n) — required by Normed<Normal>.
template <typename T, int N>
constexpr auto length(const Normal<T, N>& n) { return n.length(); }

/// length_squared(n) — required by Normed<Normal>.
template <typename T, int N>
constexpr auto length_squared(const Normal<T, N>& n) { return n.length_squared(); }

/// normalized(n) — required by Normed<Normal>.
template <typename T, int N>
constexpr auto normalized(const Normal<T, N>& n) { return n.normalized(); }

/// dot(Normal, Vector): cross-type inner product for CovectorSpace (required by concept).
template <typename T, typename U, int N>
constexpr auto dot(const Normal<T, N>& lhs, const Vector<U, N>& rhs) {
    return lhs.dot(rhs);
}

/// dot(Vector, Normal): cross-type inner product for CovectorSpace.
template <typename T, typename U, int N>
constexpr auto dot(const Vector<T, N>& lhs, const Normal<U, N>& rhs) {
    return rhs.dot(lhs);
}

/// dot(Normal, Normal): deliberately NOT declared.
/// Absence (not deletion) ensures !InnerProduct<Normal,S> in algebra_concepts.

/// cross(Normal, Normal): deleted — normals cannot be crossed.
template <typename T, int N>
auto cross(const Normal<T, N>&, const Normal<T, N>&) = delete;

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

using Normal2 = Normal<Float, 2>;
using Normal3 = Normal<Float, 3>;
using Normal4 = Normal<Float, 4>;

}  // namespace pbpt::math
