/**
 * @file
 * @brief Polynomial evaluation utilities implemented via Horner's rule.
 */
#pragma once

#include <cmath>
namespace pbpt::math {

/**
 * @brief Simple polynomial utilities implemented via Horner's rule.
 *
 * Provides static `evaluate` functions that compute a polynomial
 * given its coefficients and an input x. Coefficients are passed
 * as a parameter pack in increasing order of degree.
 *
 * @tparam T Scalar type of the evaluation result.
 */
template <typename T>
class Polynomial {
public:
    /**
     * @brief Evaluates a constant polynomial c.
     *
     * @param x Input value (unused).
     * @param c Constant coefficient.
     */
    template <typename C>
    static constexpr T evaluate(T x, C c) {
        return c;
    }

    /**
     * @brief Evaluates a polynomial using Horner's rule.
     *
     * The coefficients (c, cs...) represent
     *   c + x * (next coefficients...),
     * so the call is expanded recursively to build the full polynomial.
     *
     * @param x  Input value.
     * @param c  Current coefficient.
     * @param cs Remaining coefficients.
     */
    template <typename C, typename... Cs>
    static constexpr T evaluate(T x, C c, Cs... cs) {
        return std::fmal(x, evaluate(x, cs...), c);
    }
};

};  // namespace pbpt::math
