#pragma once

#include <optional>
#include <utility>
#include <variant>
#include "geometry/ray.hpp"
#include "shape.hpp" 
#include "shape/plugin/shape/shape_type.hpp"

namespace pbpt::shape {

template<typename T>
struct PrimitiveIntersectionRecord {
    IntersectionRecord<T> intersection;
    int material_id{-1};
    int light_id{-1};   
};

/**
 * @brief Represents a geometric instance in the scene.
 * Holds the geometry (Shape) and the material ID.
 */
template<typename T>
class Primitive {
private:
    // 直接持有 AnyShape，管理生命周期
    // 或者是 std::shared_ptr<ShapeBase> 如果你想用虚函数
    // 鉴于你的风格，这里推荐持有 Variant
    AnyShape<T> m_shape; 
    int m_material_id{-1};
    int m_light_id{-1};

public:
    // 构造函数：移动 shape 进来
    template <typename ConcreteShape>
    Primitive(ConcreteShape&& shape, int material_id, int light_id = -1)
        : m_shape(std::forward<ConcreteShape>(shape)), m_material_id(material_id), m_light_id(light_id) {}

    std::optional<PrimitiveIntersectionRecord<T>> intersect_ray(const geometry::Ray<T, 3>& ray) const {
        // 使用 std::visit 分发给具体的 Shape
        auto rec = std::visit([&](const auto& s) -> std::optional<IntersectionRecord<T>> { 
            return s.intersect_ray(ray); 
        }, m_shape);

        if (!rec) return std::nullopt;

        PrimitiveIntersectionRecord<T> record;
        record.intersection = *rec;
        record.material_id = m_material_id;
        record.light_id = m_light_id;
        return record;
    }

    std::optional<PrimitiveIntersectionRecord<T>> intersect_ray_differential(const geometry::RayDifferential<T, 3>& ray_diff) const {
        auto rec = std::visit([&](const auto& s) -> std::optional<IntersectionRecord<T>> {
            return s.intersect_ray_differential(ray_diff);
        }, m_shape);

        if (!rec) return std::nullopt;

        PrimitiveIntersectionRecord<T> record;
        record.intersection = *rec;
        record.material_id = m_material_id;
        record.light_id = m_light_id;
        return record;
    }

    std::optional<T> is_intersected_ray(const geometry::Ray<T, 3>& ray) const {
        return std::visit([&](const auto& s) -> std::optional<T> { 
            return s.is_intersected_ray(ray); 
        }, m_shape);
    }

    template<typename Func>
    void visit_shape(Func&& visitor) const {
        std::visit(visitor, m_shape);
    }

    int material_id() const { return m_material_id; }
    int light_id() const { return m_light_id; }
};

}  // namespace pbpt::shape
