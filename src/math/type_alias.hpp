/**
 * @file
 * @brief Fundamental math type aliases (Float, Int) and numeric constants.
 */
#pragma once

namespace pbpt::math {

#ifdef PBPT_FLOAT_64BIT
/// @brief Default floating-point type (double when PBPT_FLOAT_64BIT is defined).
using Float = double;
#else
/// @brief Default floating-point type (float unless PBPT_FLOAT_64BIT is defined).
using Float = float;
#endif

#ifdef PBPT_INT_64BIT
/// @brief Signed integer type for math utilities (64-bit when PBPT_INT_64BIT is defined).
using Int  = long long;
/// @brief Unsigned integer type for math utilities (64-bit when PBPT_INT_64BIT is defined).
using UInt = unsigned long long;
#else
/// @brief Signed integer type for math utilities (defaults to 32-bit).
using Int  = int;
/// @brief Unsigned integer type for math utilities (defaults to 32-bit).
using UInt = unsigned int;
#endif

/// @brief Epsilon threshold for double precision operations.
#define DOUBLE_EPS 1e-10
/// @brief Epsilon threshold for single precision operations.
#define FLOAT_EPS 1e-5

#ifndef M_PI
/// @brief π constant used when the platform does not provide it.
#define M_PI 3.14159265358979323846
#endif

}  // namespace pbpt::math
