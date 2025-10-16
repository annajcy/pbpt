#pragma once

#include "math/vector.hpp"

namespace pbpt::camera {

template<typename T>
class Film {
private:
    math::Vector<int, 2> m_resolution{};
    math::Vector<T, 2> m_physical_size{};

public:
    Film(const math::Vector<int, 2>& resolution, const math::Vector<T, 2>& physical_size)
        : m_resolution(resolution), m_physical_size(physical_size) {}

    const math::Vector<int, 2>& resolution() const { return m_resolution; }
    const math::Vector<T, 2>& physical_size() const { return m_physical_size; }
};

};