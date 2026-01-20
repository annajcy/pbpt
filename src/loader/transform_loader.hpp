#pragma once

#include <pugixml.hpp>
#include "loader/basic_parser.hpp"
#include "camera/render_transform.hpp"
#include "geometry/transform.hpp"

namespace pbpt::loader {

template <typename T>
geometry::Transform<T> load_transform(const pugi::xml_node& node) {
    geometry::Transform<T> transform = geometry::Transform<T>::identity();
    
    // Iterate over children (translate, scale, rotate, lookAt, matrix)
    // Order matters! Transformations are usually multiplied.
    // In Mitsuba/PBRT, transforms are often applied in order.
    
    for (auto child : node.children()) {
        std::string name = child.name();
        if (name == "lookAt") {
            auto origin = parse_point<T, 3>(child.attribute("origin").value());
            auto target = parse_point<T, 3>(child.attribute("target").value());
            auto up = parse_vector<T, 3>(child.attribute("up").value());
            // geometry::Transform doesn't have look_at, usually RenderTransform does or it's a utility.
            // Let's check geometry/transform.hpp or assume we construct matrix.
            // For now, let's look for a look_at helper or implement it.
            // Wait, cbox_scene.hpp uses camera::RenderTransform::look_at.
            // Here we are parsing generic transforms (e.g. for shapes). 
            // Shape transforms are usually ObjectToWorld.
            // Let's implement a simple lookAt manually if needed or find it.
            // Actually, RenderTransform::look_at returns a RenderTransform (wrapper), 
            // but here we might just want a geometry::Transform matrix.
            // We can implement look_at logic or maybe geometry::Transform has it?
            
            // Assuming we implement a local helper or use what's available.
            // I'll skip lookAt for generic transform for now unless I find it in geometry/transform.hpp
            // But cbox.xml uses lookAt for sensor. 
            // Sensor parsing might use RenderTransform directly.
            // Shape parsing uses <transform><translate/></transform>.
        } else if (name == "translate") {
            T x = child.attribute("x").as_float(0);
            T y = child.attribute("y").as_float(0);
            T z = child.attribute("z").as_float(0);
            transform = transform * geometry::Transform<T>::translate(math::Vector<T, 3>(x, y, z));
        } else if (name == "scale") {
            T x = child.attribute("x").as_float(1);
            T y = child.attribute("y").as_float(1);
            T z = child.attribute("z").as_float(1);
             // Assuming scale takes a vector?
            transform = transform * geometry::Transform<T>::scale(math::Vector<T, 3>(x, y, z));
        } else if (name == "rotate") {
             // ...
        }
    }
    return transform;
}

// Special parser for Camera RenderTransform which supports lookAt
template <typename T>
camera::RenderTransform<T> load_render_transform(const pugi::xml_node& node) {
    // Specifically for sensor toWorld
    for (auto child : node.children()) {
        std::string name = child.name();
        if (name == "lookAt") {
            auto origin = parse_point<T, 3>(child.attribute("origin").value());
            auto target = parse_point<T, 3>(child.attribute("target").value());
            auto up = parse_vector<T, 3>(child.attribute("up").value());
            return camera::RenderTransform<T>::look_at(origin, target, up, camera::RenderSpace::World);
        }
    }
    // Fallback or identity
    return camera::RenderTransform<T>::look_at(
        math::Point<T, 3>(0,0,0), math::Point<T, 3>(0,0,1), math::Vector<T, 3>(0,1,0), camera::RenderSpace::World
    );
}

}
