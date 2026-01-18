#pragma once

#include <optional>
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/shape.hpp"

namespace pbpt::light {

/**
 * @brief Visibility tester between two points in render space.
 *
 * Used to check if the path between a source interaction
 * and a destination point is unoccluded.
 *
 * @tparam T Scalar type (e.g. float or double).
 * @tparam NormalInteractionType Type representing a surface interaction with normal.
 */
template<typename T, typename NormalInteractionType>
class VisibilityTester {
private:
    /// Points in render space, between which visibility is tested.
    /// from m_src_i to m_dst_p
    NormalInteractionType m_src_i;
    math::Point<T, 3> m_dst_p;

public:
    VisibilityTester(
        const NormalInteractionType& src_interaction, 
        const math::Point<T, 3>& dst_point
    ) : m_src_i(src_interaction), m_dst_p(dst_point) {}
   
    template<typename Aggregate>
    bool is_unoccluded(const Aggregate& aggregate) const {
        auto ray = m_src_i.spawn_ray_to(m_dst_p);
        return !aggregate.is_intersected(ray);        
    }

    const NormalInteractionType& src_interaction() const {
        return m_src_i;
    }

    const math::Point<T, 3>& dst_point() const {
        return m_dst_p;
    }
};

};