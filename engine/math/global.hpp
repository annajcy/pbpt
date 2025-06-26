#pragma once

/**
 * @file global.hpp
 * @brief Defines the primary floating-point type alias for the math library.
 * @details This header centralizes the floating-point precision for the entire
 * pbpt::math namespace. By changing the definition here (or by defining
 * FLOAT_64BIT in the build system), the precision of all calculations
 * (in vectors, points, rays, etc.) can be switched between single and
 * double precision.
 */

namespace pbpt::math {

/**
 * @brief The primary floating-point type used throughout the math library.
 * @details This type alias controls the precision of geometric and mathematical
 * calculations.
 * - If the `FLOAT_64BIT` preprocessor macro is defined, `Float` will be a `double` (64-bit).
 * - Otherwise, `Float` will default to a `float` (32-bit).
 *
 * @note To use double-precision floating-point numbers, define the `FLOAT_64BIT`
 * macro in your project's build settings, for example, by passing the
 * `-DFLOAT_64BIT` flag to your C++ compiler.
 */
#ifdef FLOAT_64BIT
using Float = double;
#else
using Float = float;
#endif

/**
 * @brief The epsilon value used for floating-point comparisons.
 * @details This value is used to determine the precision of floating-point
 * comparisons. It is set to a small value (e.g., 1e-6) to account for floating-point
 * precision issues.
 */
#ifdef FLOAT_64BIT
#define EPSILON 1e-12
#else
#define EPSILON 1e-6
#endif

} // namespace math