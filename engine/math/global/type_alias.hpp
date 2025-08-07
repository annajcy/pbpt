#pragma once


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

template <typename T>
inline constexpr T epsilon_v = static_cast<T>(1e-5);  // float 默认

template <>
inline constexpr double epsilon_v<double> = 1e-10;    // double 特化

template <typename T>
inline constexpr T pi_v = static_cast<T>(3.14159265358979323846);

} // namespace math