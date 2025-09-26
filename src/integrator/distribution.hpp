#pragma once

#include <cmath>
#include <stdexcept>

#include "math/point.hpp"
#include "math/type_alias.hpp"

namespace pbpt::integrator {

using namespace pbpt::math;

/**
 * @brief Base class for probability distributions
 * @tparam N Dimension of the distribution
 */
template <int N>
class Distribution {
public:
    virtual ~Distribution()                                        = default;
    virtual Point<Float, N> sample(const Point<Float, N>& u) const = 0;
    virtual Float           pdf(const Point<Float, N>& p) const    = 0;
};

/**
 * @brief 1D uniform distribution over interval [a, b]
 */
class UniformDistribution1D : public Distribution<1> {
private:
    Pt1 m_a, m_b;

public:
    UniformDistribution1D(const Pt1& a, const Pt1& b) : m_a(a), m_b(b) {
        if (a.x() >= b.x()) {
            throw std::invalid_argument("Lower bound must be less than upper bound");
        }
    }
    ~UniformDistribution1D() = default;

    Point<Float, 1> sample(const Point<Float, 1>& u) const override {
        if (u[0] < 0.0 || u[0] > 1.0) {
            throw std::invalid_argument("Input u must be in [0,1]");
        }
        return m_a + u[0] * (m_b - m_a);
    }

    Float pdf(const Point<Float, 1>& p) const override {
        if (p.x() < m_a.x() || p.x() > m_b.x()) {
            return 0.0;
        }
        return 1.0 / (m_b.x() - m_a.x());
    }

    const Pt1& lower_bound() const { return m_a; }
    const Pt1& upper_bound() const { return m_b; }
};

template <int N>
class UniformDistributionND : public Distribution<N> {
private:
    Point<Float, N> m_min, m_max;

public:
    UniformDistributionND(const Point<Float, N>& min_pt, const Point<Float, N>& max_pt) : m_min(min_pt), m_max(max_pt) {
        for (int i = 0; i < N; ++i) {
            if (min_pt[i] >= max_pt[i]) {
                throw std::invalid_argument(
                    "Lower bounds must be less than "
                    "upper bounds in all dimensions");
            }
        }
    }

    Point<Float, N> sample(const Point<Float, N>& u) const override {
        for (int i = 0; i < N; ++i) {
            if (u[i] < 0.0 || u[i] > 1.0) {
                throw std::invalid_argument("Input u must be in [0,1]^N");
            }
        }

        Point<Float, N> result;
        for (int i = 0; i < N; ++i) {
            result[i] = m_min[i] + u[i] * (m_max[i] - m_min[i]);
        }
        return result;
    }

    Float pdf(const Point<Float, N>& p) const override {
        for (int i = 0; i < N; ++i) {
            if (p[i] < m_min[i] || p[i] > m_max[i]) {
                return 0.0;
            }
        }

        Float volume = 1.0;
        for (int i = 0; i < N; ++i) {
            volume *= (m_max[i] - m_min[i]);
        }
        return 1.0 / volume;
    }

    const Point<Float, N>& lower_bounds() const { return m_min; }
    const Point<Float, N>& upper_bounds() const { return m_max; }
};

class NormalDistribution1D : public Distribution<1> {
private:
    Float m_mean, m_stddev;

public:
    NormalDistribution1D(Float mean, Float stddev) : m_mean(mean), m_stddev(stddev) {
        if (stddev <= 0.0) {
            throw std::invalid_argument("Standard deviation must be positive");
        }
    }

    Point<Float, 1> sample(const Point<Float, 1>& u) const override {
        if (u[0] < 0.0 || u[0] > 1.0) {
            throw std::invalid_argument("Input u must be in [0,1]");
        }

        // Use Box-Muller transform (simplified version)
        // For a more robust implementation, consider using inverse CDF
        Float z = std::sqrt(-2.0 * std::log(u[0])) * std::cos(2.0 * M_PI * 0.5);
        return Point<Float, 1>{m_mean + m_stddev * z};
    }

    Float pdf(const Point<Float, 1>& p) const override {
        Float x        = p.x();
        Float exponent = -0.5 * std::pow((x - m_mean) / m_stddev, 2);
        return std::exp(exponent) / (m_stddev * std::sqrt(2.0 * M_PI));
    }

    Float mean() const { return m_mean; }
    Float stddev() const { return m_stddev; }
};

}  // namespace pbpt::integrator