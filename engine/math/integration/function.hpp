#pragma once

#include "math/geometry/point.hpp"
#include <functional>
#include <memory>
#include <type_traits>
#include <vector>

namespace pbpt::math {

template<typename T,  int N> 
requires std::floating_point<T> && (N > 0)
class Domain {
    virtual ~Domain() = default;
    virtual bool contains(const Point<T, N>& point) const = 0;
    virtual T volume() const = 0;
    virtual std::vector<Point<T, N>> uniform_sample(int count) const = 0;
};

template<typename T>
class SegmentDomain : public Domain<T, 1> {
private:
    Point<T, 1> m_lower;
    Point<T, 1> m_upper;

public:
    SegmentDomain(T lower, T upper) : m_lower(lower), m_upper(upper) {}

    bool contains(const Point<T, 1>& point) const override {
        return point >= m_lower && point <= m_upper;
    }

    T volume() const override {
        return m_upper[0] - m_lower[0];
    }
};

template<typename T, int N>
requires std::is_floating_point_v<T> && (N > 0)
class Function {
private:
    std::function<T(const Point<T, N>&)> m_function;
    std::shared_ptr<Domain<T, N>> m_domain;

public:
    explicit Function(
        std::function<T(const Point<T, N>&)> function,
        std::shared_ptr<Domain<T, N>> domain) : m_function(function), m_domain(domain) {}
    
    T operator()(const Point<T, N>& point) const {
        if (m_domain && !m_domain->contains(point)) {
            throw std::runtime_error("Point is outside the domain");
        }
        return m_function(point);
    }

    ~Function() = default;
};





}