#pragma once

namespace pbpt::math {

#ifdef PBPT_FLOAT_64BIT
using Float = double;
#else
using Float = float;
#endif

#ifdef PBPT_INT_64BIT
using Int  = long long;
using UInt = unsigned long long;
#else
using Int  = int;
using UInt = unsigned int;
#endif

#define DOUBLE_EPS 1e-10
#define FLOAT_EPS 1e-5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

}  // namespace pbpt::math