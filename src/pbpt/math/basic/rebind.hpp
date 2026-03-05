/**
 * @file
 * @brief Type rebinding trait for CRTP-based math types.
 *
 * Provides `rebind_trait<Derived>` — a type-level mapping from a concrete
 * instantiation (e.g., `Vector<float, 3>`) back to its parametric family,
 * so `Tuple` can produce `Derived<U, M>` via `rebind_t<Derived, U, M>`.
 *
 * Each concrete Tuple-derived type must provide a specialization:
 * @code
 * template <typename T, int N>
 * struct rebind_trait<Vector<T, N>> {
 *     template <typename U, int M> using type = Vector<U, M>;
 * };
 * @endcode
 */
#pragma once

#include <type_traits>

namespace pbpt::math {

namespace detail {
template <typename T>
inline constexpr bool rebind_always_false = false;
}  // namespace detail

/**
 * @brief Primary template — must be specialized for each concrete derived type.
 *
 * The specialization must expose:
 *   `template <typename U, int M> using type = YourType<U, M>;`
 */
template <typename Derived>
struct rebind_trait {
    static_assert(detail::rebind_always_false<Derived>,
                  "rebind_trait must be specialized for this type. "
                  "Add: template<typename T, int N> struct rebind_trait<YourType<T,N>> { "
                  "  template<typename U, int M> using type = YourType<U,M>; "
                  "};");
};

/**
 * @brief Convenience alias: rebind Derived<T,N> to Derived<U,M>.
 *
 * Example: `rebind_t<Vector<float,3>, double, 4>` == `Vector<double,4>`
 */
template <typename Derived, typename U, int M>
using rebind_t = typename rebind_trait<Derived>::template type<U, M>;

}  // namespace pbpt::math
