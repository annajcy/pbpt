#pragma once

#include <cmath>
namespace pbpt::math {

template<typename T>
class Polynomial {
public:

    template<typename C>
    static constexpr T evaluate(T x, C c) {
        return c;
    }

    template<typename C, typename... Cs>
    static constexpr T evaluate(T x, C c, Cs... cs) {
        return std::fmal(x, evaluate(x, cs...), c);
    }
};

};