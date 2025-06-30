#pragma once

#include "math/geometry/point.hpp"

namespace pbpt::math {

template<int N>
class Distribution {
public:
    virtual ~Distribution() = default;
    virtual Point<Float, N> sample(const Point<Float, N>& u) const = 0;
    virtual Float pdf(const Point<Float, N>&) const = 0;
};

class UniformDistribution1D : public Distribution<1> {
private: 
    Pt1 m_a, m_b;

public:
    UniformDistribution1D(const Pt1& a, const Pt1& b) : m_a(a), m_b(b) {}
    ~UniformDistribution1D() = default;

    Point<Float, 1> sample(const Point<Float, 1>& u) const override {
        return m_a + u[0] * (m_b - m_a);
    }

    Float pdf(const Point<Float, 1>& p) const override {
        return 1.0 / (m_b.x() - m_a.x());
    }
};

}