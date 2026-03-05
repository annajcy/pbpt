/**
 * @file
 * @brief Structural algebra traits/concepts for pbpt math objects.
 */
#pragma once

#include <concepts>
#include <type_traits>
#include "pbpt/math/basic/function.hpp"
#include "pbpt/math/basic/type_alias.hpp"

namespace pbpt::math {

template <typename T>
struct dependent_false : std::false_type {};

template <typename T>
inline constexpr bool dependent_false_v = dependent_false<T>::value;

template <typename T>
struct algebra_traits {
    static_assert(dependent_false_v<T>, "algebra_traits is not specialized for this type");
};

template <typename T>
concept HasAlgebraTraits = requires {
    typename algebra_traits<T>::scalar;
    { algebra_traits<T>::dim } -> std::convertible_to<int>;
};

template <typename T>
concept MathObject = HasAlgebraTraits<T> && requires(const T& a, const T& b) {
    { a == b } -> std::convertible_to<bool>;
    { a != b } -> std::convertible_to<bool>;
    // 在浮点数学中，通常需要一个容差比较函数
    { is_approx(a, b) } -> std::convertible_to<bool>;
};

template <typename T>
concept ArithmeticScalar = std::is_arithmetic_v<T>;

template <typename T>
concept FloatingScalar = std::floating_point<T>;

template <HasAlgebraTraits T>
using algebra_scalar_t = typename algebra_traits<T>::scalar;

template <HasAlgebraTraits T>
inline constexpr int algebra_dim_v = algebra_traits<T>::dim;

template <typename T>
inline constexpr T zero_v = [] {
    static_assert(dependent_false_v<T>, "zero_v is not specialized for this type");
    return T{};
}();

template <typename T>
    requires ArithmeticScalar<T>
inline constexpr T zero_v<T> = T(0);

template <typename T>
inline constexpr T identity_v = [] {
    static_assert(dependent_false_v<T>, "identity_v is not specialized for this type");
    return T{};
}();

template <typename T>
    requires ArithmeticScalar<T>
inline constexpr T identity_v<T> = T(1);

template <typename T>
inline constexpr T inverse(const T& value) {
    static_assert(dependent_false_v<T>, "inverse is not defined for this type");
    return T{};
}

template <typename T>
requires ArithmeticScalar<T>
inline constexpr T inverse(const T& value) {
    static_assert(!std::is_integral_v<T>, "Inverse is not defined for integral types");
    static_assert(!std::is_same_v<T, bool>, "Inverse is not defined for boolean types");
    assert_if([&value]() { return pbpt::math::is_zero(static_cast<T>(value)); },
              "Division by zero in inverse");
    return T(1) / value;
}

template <typename A, typename B>
concept SameDim = HasAlgebraTraits<A> && HasAlgebraTraits<B> && (algebra_dim_v<A> == algebra_dim_v<B>);

template <typename A, typename B>
concept SameScalarFamily =
    HasAlgebraTraits<A> && HasAlgebraTraits<B> &&
    (std::same_as<std::remove_cv_t<algebra_scalar_t<A>>, std::remove_cv_t<algebra_scalar_t<B>>> ||
     (std::floating_point<std::remove_cv_t<algebra_scalar_t<A>>> &&
      std::floating_point<std::remove_cv_t<algebra_scalar_t<B>>>));

template <typename A, typename B>
using promote_binary_t = std::common_type_t<A, B>;

template <typename A, typename B, typename C>
using promote_ternary_t = std::common_type_t<A, B, C>;

template <typename T>
using promote_int_to_float_t = std::conditional_t<std::is_integral_v<T>, Float, T>;

template <typename T>
using promote_norm_t = promote_int_to_float_t<T>;

template <typename T>
concept IndexedMathType = HasAlgebraTraits<T> && requires(const T& v) {
    { v[0] } -> std::convertible_to<algebra_scalar_t<T>>;
    { from_array(v.to_array()) } -> std::same_as<T>;
    { v.for_each(std::declval<void (*)(algebra_scalar_t<T>, int)>()) };
    { v.for_each(std::declval<void (*)(const algebra_scalar_t<T>&, int)>()) };
};

template <typename T>
concept Additive = requires(T a, T b) {
    { a + b } -> std::convertible_to<T>;
};

template<typename T>
concept AdditiveCommutative = Additive<T> && requires(T a, T b) {
    { a + b == b + a } -> std::convertible_to<bool>;
};

template <typename T>
concept AdditiveAssociative = Additive<T> && requires(T a, T b, T c) {
    { (a + b) + c == a + (b + c) } -> std::convertible_to<bool>;
};

template <typename T>
concept AdditiveIdentity = requires {
    { zero_v<T> } -> std::convertible_to<T>;
};

template <typename T>
concept AdditiveInverse = requires(T a, T b) {
    { a - zero_v<T> } -> std::convertible_to<T>;
    { a - b } -> std::convertible_to<T>;
    { -a } -> std::convertible_to<T>;
};

template<typename T>
concept AdditiveSemigroup = Additive<T> && AdditiveAssociative<T>;

template<typename T>
concept AdditiveMonoid = AdditiveSemigroup<T> && AdditiveIdentity<T>;

template<typename T>
concept AdditiveGroup = AdditiveMonoid<T> && AdditiveInverse<T>;

template <typename T>
concept AdditiveAbelianGroup = AdditiveGroup<T> && AdditiveCommutative<T>;

template<typename T>
concept Multiplicative = requires(T a, T b) {
    { a * b } -> std::convertible_to<T>;
};

template <typename T>
concept MultiplicativeIdentity = requires {
    { identity_v<T> } -> std::convertible_to<T>;
};

// assume multiplicative zero is the same as additive zero for simplicity, as is the case for most algebraic structures used in graphics
template <typename T>
concept MultiplicativeZero = requires {
    { zero_v<T> } -> std::convertible_to<T>;
};

template <typename T>
concept MultiplicativeCommutative = Multiplicative<T> && requires(T a, T b) {
    { a * b == b * a } -> std::convertible_to<bool>;
};

template <typename T>
concept MultiplicativeAssociative = Multiplicative<T> && requires(T a, T b, T c) {
    { (a * b) * c == a * (b * c) } -> std::convertible_to<bool>;
};

template<typename T>
concept MultiplicativeInverse = requires(T a) {
    { inverse(a) } -> std::convertible_to<T>;
};

template<typename T>
concept MultiplicativeSemigroup = Multiplicative<T> && MultiplicativeAssociative<T>;

template<typename T>
concept MultiplicativeMonoid = MultiplicativeSemigroup<T> && MultiplicativeIdentity<T>;

template<typename T>
concept MultiplicativeGroup = MultiplicativeMonoid<T> && MultiplicativeInverse<T>;

template <typename T>
concept MultiplicativeAbelianGroup = MultiplicativeGroup<T> && MultiplicativeCommutative<T>;

template <typename T>
concept Ring = AdditiveAbelianGroup<T> && MultiplicativeMonoid<T> && requires(T a, T b, T c) {
    { a * (b + c) == a * b + a * c } -> std::convertible_to<bool>;
    { (a + b) * c == a * c + b * c } -> std::convertible_to<bool>;
};

// assume multiplicative zero is the only zero and that it annihilates all elements.
// Commutative ring is also integral domain under this assumption, which is the case for most algebraic structures used in graphics.
template<typename T>
concept CommutativeRing = Ring<T> && MultiplicativeCommutative<T>;

template <typename T>
concept DivisionRing = Ring<T> && MultiplicativeGroup<T>;

template <typename T>
concept Field = CommutativeRing<T> && DivisionRing<T>;

template <typename T, typename S>
concept Scalable = requires(T a, S s) {
    { a * s } -> std::convertible_to<T>;
    { s * a } -> std::convertible_to<T>;
    { a / s } -> std::convertible_to<T>;
};

template<typename T, typename S>
concept VectorSpace = AdditiveGroup<T> && Scalable<T, S>;

template <typename T>
concept Normed = requires(const T& a) {
    { length(a) };
    { length_squared(a) };
    { normalized(a) };
};

template <typename T, typename S>
concept InnerProduct = requires(const T& a, const T& b) {
    { dot(a, b) } -> std::convertible_to<S>;
};

template<typename T, typename S>
concept MetricVectorSpace = 
    VectorSpace<T, S> && 
    Normed<T> && 
    InnerProduct<T, S>;

template <typename T>
concept CrossProductSpace = requires(const T& a, const T& b) {
    { cross(a, b) } -> std::convertible_to<T>;
};

template <typename T, typename S>
concept VectorSpace3D = 
    MetricVectorSpace<T, S> && 
    CrossProductSpace<T> && 
    (algebra_dim_v<T> == 3);

template <typename P, typename V, typename S>
concept AffineSpace = 
    !AdditiveMonoid<P> &&
    !MultiplicativeMonoid<P> &&
    !Scalable<P, S> &&
    MetricVectorSpace<V, S> && requires(P p, P q, V v, S t, P a, P b, P c, S t1, S t2, S t3) {
    { p + v } -> std::convertible_to<P>;
    { p - v } -> std::convertible_to<P>;
    { p - q } -> std::convertible_to<V>;
    // 新增：两点之间的距离
    { distance(p, q) } -> std::convertible_to<S>;
    { distance_squared(p, q) } -> std::convertible_to<S>;
    
    // 新增：点的仿射组合（线性插值），这在图形学中无处不在 (比如重心坐标计算)
    { lerp(p, q, t) } -> std::convertible_to<P>;
    { lerp(a, b, c, t1, t2, t3) } -> std::convertible_to<P>;
};

template <typename N, typename V, typename S>
concept CovectorSpace = 
    !InnerProduct<N, S> &&
    VectorSpace<N, S> && 
    Normed<N> &&
    MetricVectorSpace<V, S> && 
    requires(const N& n, const V& v) {
    { dot(n, v) } -> std::convertible_to<S>;
    { dot(v, n) } -> std::convertible_to<S>;
};

template <typename T, typename S = algebra_scalar_t<T>>
concept LinearAlgebra = VectorSpace<T, S> && Ring<T>;

template <typename T, typename S = algebra_scalar_t<T>>
concept DivisionAlgebra = LinearAlgebra<T, S> && DivisionRing<T>;

template <typename M, typename S = algebra_scalar_t<M>>
concept MatrixAlgebra = LinearAlgebra<M, S> && requires(const M& m) {
    // 矩阵自身的属性
    { transpose(m) } -> std::same_as<M>;
    { determinant(m) } -> std::convertible_to<S>;
    { trace(m) } -> std::convertible_to<S>;
};

}  // namespace pbpt::math
