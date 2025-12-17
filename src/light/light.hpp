#pragma once

#include "geometry/transform.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/shape.hpp"
namespace pbpt::light {

template<typename T, typename Derived>
class Light {
public:
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    geometry::Transform<T> render_to_light_transform() const {
        return as_derived().render_to_light_transform_impl();
    }

    geometry::Transform<T> light_to_render_transform() const {
        return as_derived().light_to_render_transform_impl();
    }
};

template<typename T, template<typename> typename ShapeType>
class AreaLight : public Light<T, AreaLight<T, ShapeType>> {
    friend class Light<T, AreaLight<T, ShapeType>>;
private:
    const shape::TransformedShape<T, ShapeType>& m_transformed_shape;

public:
    AreaLight(const shape::TransformedShape<T, ShapeType>& transformed_shape) : m_transformed_shape(transformed_shape) {}

    const shape::TransformedShape<T, ShapeType>& transform_shape() const {
        return m_transformed_shape;
    }

private:
    geometry::Transform<T> render_to_light_transform_impl() const {
        return m_transformed_shape.world_to_object_transform();
    }

    geometry::Transform<T> light_to_render_transform_impl() const {
        return m_transformed_shape.object_to_world_transform();
    }
};

};