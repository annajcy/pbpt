#pragma once

#include "geometry/frame.hpp"
#include "math/normal.hpp"
namespace pbpt::material {

template<typename T>
class BSDF {
private: 
    geometry::Frame<T> m_shading_frame;
    geometry::Frame<T> m_geometric_frame;

public:
    BSDF(
        const geometry::Frame<T>& shading_frame,
        const geometry::Frame<T>& geometric_frame
    ) : m_shading_frame(shading_frame), 
        m_geometric_frame(geometric_frame) {}

    const geometry::Frame<T>& shading_frame() const {
        return m_shading_frame;
    }

    const geometry::Frame<T>& geometric_frame() const {
        return m_geometric_frame;
    }

};

};