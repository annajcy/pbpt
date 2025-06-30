#pragma once

/**
 * @file type_alias.hpp
 * @brief Defines the primary floating-point type alias for the math library.
 * @details This header centralizes the floating-point precision for the entire
 * pbpt::math namespace. By changing the definition here (or by defining
 * FLOAT_64BIT in the build system), the precision of all calculations
 * (in vectors, points, rays, etc.) can be switched between single and
 * double precision.
 */

namespace pbpt::math {

#ifdef FLOAT_64BIT
using Float = double;
#else
using Float = float;
#endif

#ifdef INT_64BIT
using Int = long long;
using UInt = unsigned long long;
#else
using Int = int;
using UInt = unsigned int;
#endif

#ifdef FLOAT_64BIT
#define EPSILON 1e-12
#else
#define EPSILON 1e-6
#endif

} // namespace math