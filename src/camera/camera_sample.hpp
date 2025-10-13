#pragma once 

#include "math/point.hpp"

namespace pbpt::camera {

template<typename T>
struct CameraSample{
    math::Point<T, 2> p_film{};
};

template<typename T>
struct ThinLensCameraSample {
    math::Point<T, 2> p_film{};
    math::Point<T, 2> p_lens{};
};

};