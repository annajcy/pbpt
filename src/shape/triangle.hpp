/**
 * @file triangle.hpp
 * @brief Triangle mesh and triangle shape implementation.
 */
#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/function.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "sampler/3d.hpp"
#include "shape.hpp"

#include "utils/obj_loader.hpp"

namespace pbpt::shape {

// --- Helper Functions ---

template<typename T>
bool is_at_left(
    const math::Point<T, 3>& p,
    const math::Point<T, 3>& a,
    const math::Point<T, 3>& b
) {
    auto ab = b - a;
    auto ap = p - a;
    auto cross_product = math::cross(ab, ap);
    // Assuming z-up or consistent winding for 2D projection check
    // This helper might need context specific axis handling if used generally
    return cross_product.y() > T(0); 
}

// Möller-Trumbore Intersection Algorithm
template<typename T>
struct TriangleIntersectionResult {
    std::array<T, 3> barycentric_coords;
    T t_hit{};
};

template<typename T>
std::optional<TriangleIntersectionResult<T>> intersect_triangle_geometry(
    const geometry::Ray<T, 3>& ray,
    const math::Point<T, 3>& p0,
    const math::Point<T, 3>& p1,
    const math::Point<T, 3>& p2
) {
    auto e1 = p1 - p0;
    auto e2 = p2 - p0;
    auto s1 = math::cross(ray.direction(), e2);
    T denom = s1.dot(e1);

    if (std::abs(denom) < math::epsilon_v<T>) {
        return std::nullopt;
    }

    T inv_denom = T(1) / denom;
    auto s0 = ray.origin() - p0;
    auto v = s1.dot(s0) * inv_denom;
    
    if (v < T(0) || v > T(1)) return std::nullopt;

    auto s2 = math::cross(s0, e1);
    auto w = s2.dot(ray.direction()) * inv_denom;

    if (w < T(0) || v + w > T(1)) return std::nullopt;

    auto t_hit = s2.dot(e2) * inv_denom;
    auto u = T(1) - v - w;

    return std::make_optional(
        TriangleIntersectionResult<T>{
            std::array<T, 3>{u, v, w}, 
            t_hit
        }
    );
}

template<typename T>
class TriangleMesh {
private:
    int m_triangle_count;
    int m_vertices_count;

    // Data stored in Rendering Space
    std::vector<int> m_indices;
    std::vector<math::Point<T, 3>> m_positions;
    std::vector<math::Normal<T, 3>> m_normals;
    std::vector<math::Point<T, 2>> m_uvs;

    bool m_is_reverse_orientation;
    bool m_is_swap_handedness;
    bool m_should_flip_normal; // Pre-calculated XOR result

    TriangleMesh(
        const geometry::Transform<T>& object_to_render,
        const utils::ObjMeshData<T>& mesh_data,
        bool is_reverse_orientation
    ) : TriangleMesh(
            object_to_render,
            mesh_data.indices,
            mesh_data.positions,
            mesh_data.normals,
            mesh_data.uvs,
            is_reverse_orientation
        ) {}

public:
    TriangleMesh(
        const geometry::Transform<T>& object_to_render,
        const std::string& obj_path,
        bool is_reverse_orientation = false
    ) : TriangleMesh(object_to_render, utils::load_obj_mesh<T>(obj_path), is_reverse_orientation) {}

    TriangleMesh(
        const geometry::Transform<T>& object_to_render,
        const std::vector<int>& indices,
        const std::vector<math::Point<T, 3>>& positions,
        const std::vector<math::Normal<T, 3>>& normals = {},
        const std::vector<math::Point<T, 2>>& uvs = {},
        bool is_reverse_orientation = false
    ) : m_indices(indices), m_uvs(uvs), 
        m_is_reverse_orientation(is_reverse_orientation)
    {
        m_triangle_count = static_cast<int>(m_indices.size() / 3);
        m_vertices_count = static_cast<int>(positions.size());
        m_is_swap_handedness = object_to_render.is_swaps_handedness();
        
        // Pre-calculate flip logic: User ^ Matrix
        m_should_flip_normal = m_is_reverse_orientation ^ m_is_swap_handedness;

        // Pre-transform Positions
        m_positions.reserve(positions.size());
        for (const auto& pos : positions) {
            m_positions.push_back(object_to_render.transform_point(pos));
        }

        // Pre-transform Normals
        // Note: We only apply the mathematical transform here. 
        // Logic flipping is handled in Triangle::intersect based on m_should_flip_normal
        if (!normals.empty()) {
            m_normals.reserve(normals.size());
            for (const auto& n : normals) {
                m_normals.push_back(
                    object_to_render.transform_normal(n).normalized()
                );
            }
        }
    }

    bool should_flip_normal() const { return m_should_flip_normal; }
    bool has_normals() const { return !m_normals.empty(); }
    bool has_uvs() const { return !m_uvs.empty(); }
    
    // Accessors
    const std::vector<int>& indices() const { return m_indices; }
    const std::vector<math::Point<T, 3>>& positions() const { return m_positions; }
    const std::vector<math::Normal<T, 3>>& normals() const { return m_normals; }
    const std::vector<math::Point<T, 2>>& uvs() const { return m_uvs; }

    std::array<int, 3> triangle_indices(int triangle_index) const {
        int base = triangle_index * 3;
        return { m_indices[base], m_indices[base + 1], m_indices[base + 2] };
    }

    int triangle_count() const { return m_triangle_count; }
    int vertex_count() const { return m_vertices_count; } 
};

// --- Triangle Shape (Lightweight Handle) ---

template<typename T>
class Triangle: public Shape<Triangle<T>, T> {
    friend class Shape<Triangle<T>, T>;
private:
    const TriangleMesh<T>& m_mesh;
    int m_triangle_index;

public:
    Triangle(const TriangleMesh<T>& mesh, int triangle_index) 
        : m_mesh(mesh), m_triangle_index(triangle_index) {}


    // --- Helpers ---
    std::array<int, 3> get_indices() const {
        return m_mesh.triangle_indices(m_triangle_index);
    }

    // --- Shape Interface Implementation ---

    geometry::Transform<T> object_to_render_transform_impl() const {
        // Triangle itself does not hold transform; delegate to mesh
        // Note: This assumes mesh has consistent transform for all triangles
        return geometry::Transform<T>::identity();
    }

    geometry::Transform<T> render_to_object_transform_impl() const {
        // Triangle itself does not hold transform; delegate to mesh
        // Note: This assumes mesh has consistent transform for all triangles
        return geometry::Transform<T>::identity();
    }

    T area_impl() const {
        auto idx = get_indices();
        const auto& p0 = m_mesh.positions()[idx[0]];
        const auto& p1 = m_mesh.positions()[idx[1]];
        const auto& p2 = m_mesh.positions()[idx[2]];
        return T(0.5) * math::cross(p1 - p0, p2 - p0).length();
    }

    geometry::Bounds<T, 3> bounding_box_impl() const {
        auto idx = get_indices();
        geometry::Bounds<T, 3> bbox;
        bbox.unite(m_mesh.positions()[idx[0]]);
        bbox.unite(m_mesh.positions()[idx[1]]);
        bbox.unite(m_mesh.positions()[idx[2]]);
        return bbox;
    }

    geometry::DirectionalCone<T> normal_bounding_cone_impl() const {
        auto idx = get_indices();
        const auto& p0 = m_mesh.positions()[idx[0]];
        const auto& p1 = m_mesh.positions()[idx[1]];
        const auto& p2 = m_mesh.positions()[idx[2]];
        
        // 1. Geometric Normal
        auto n = math::cross(p1 - p0, p2 - p0).normalized();
        
        // 2. Enforce Consistency
        if (m_mesh.has_normals()) {
            auto ns = m_mesh.normals()[idx[0]] + 
                      m_mesh.normals()[idx[1]] + 
                      m_mesh.normals()[idx[2]];
            if (n.dot(ns.to_vector()) < 0) n = -n;
        } else if (m_mesh.should_flip_normal()) {
            n = -n;
        }
        
        return geometry::DirectionalCone<T>(n.to_vector(), math::deg2rad(180.0));
    }

    std::optional<IntersectionRecord<T>> intersect_impl(const geometry::Ray<T, 3>& ray) const {
        // 1. Get Vertices (Render Space)
        auto idx = get_indices();
        const auto& p0 = m_mesh.positions()[idx[0]];
        const auto& p1 = m_mesh.positions()[idx[1]];
        const auto& p2 = m_mesh.positions()[idx[2]];

        // 2. Ray-Triangle Intersection
        auto result_opt = intersect_triangle_geometry(ray, p0, p1, p2); 
        if (!result_opt.has_value()) return std::nullopt;
        
        auto& result = result_opt.value();
        T t_hit = result.t_hit;

        if (t_hit < ray.t_min() || t_hit > ray.t_max()) return std::nullopt;

        // 3. Barycentrics
        T b0 = result.barycentric_coords[0];
        T b1 = result.barycentric_coords[1];
        T b2 = result.barycentric_coords[2];

        // 4. Interpolate Hit Point
        // (Using interpolation is numerically more stable than ray(t) for triangles)
        auto p_hit_vec = b0 * p0.to_vector() + b1 * p1.to_vector() + b2 * p2.to_vector(); 
        auto p_hit = math::Point<T, 3>::from_vector(p_hit_vec);
        
        // Error bounds for robust intersection (gamma(7) approx)
        math::Vector<T, 3> p_error =
            (b0 * p0.to_vector()).abs() +
            (b1 * p1.to_vector()).abs() +
            (b2 * p2.to_vector()).abs();
        p_error = p_error * math::gamma<T>(7);

        // 5. Interpolate UVs
        math::Point<T, 2> uv_hit{T(0), T(0)};
        if (m_mesh.has_uvs()) {
            auto uv_vec = b0 * m_mesh.uvs()[idx[0]].to_vector() + 
                          b1 * m_mesh.uvs()[idx[1]].to_vector() + 
                          b2 * m_mesh.uvs()[idx[2]].to_vector();
            uv_hit = math::Point<T, 2>::from_vector(uv_vec);
        }

        // 6. Compute Geometric Normal
        auto dp02 = p0 - p2;
        auto dp12 = p1 - p2;
        auto ng = math::cross(dp02, dp12).normalized();
        
        // 7. Compute Shading Normal & FaceForward
        math::Normal<T, 3> ns;
        if (m_mesh.has_normals()) {
            ns = (b0 * m_mesh.normals()[idx[0]] + 
                  b1 * m_mesh.normals()[idx[1]] + 
                  b2 * m_mesh.normals()[idx[2]]).normalized();
            // Force geometric normal to match shading normal hemisphere
            if (ng.dot(ns.to_vector()) < 0) ng = -ng;
        } else {
            ns = ng;
            // Apply mesh flip logic if no explicit normals exist
            if (m_mesh.should_flip_normal()) {
                ng = -ng;
                ns = -ns;
            }
        }

        // 8. Compute Partial Derivatives (dpdu, dpdv)
        math::Vector<T, 3> dpdu, dpdv;
        math::Normal<T, 3> dndu{0,0,0}, dndv{0,0,0}; // Always 0 for flat triangle geometry
        math::Normal<T, 3> shading_dndu{0,0,0}, shading_dndv{0,0,0};

        if (m_mesh.has_uvs()) {
            const auto& uvs = m_mesh.uvs();
            auto duv1 = uvs[idx[1]] - uvs[idx[0]];
            auto duv2 = uvs[idx[2]] - uvs[idx[0]];
            auto dp1 = p1 - p0;
            auto dp2 = p2 - p0;

            T det = duv1.x() * duv2.y() - duv1.y() * duv2.x();
            if (std::abs(det) > 1e-8f) {
                T inv_det = T(1) / det;
                dpdu = (duv2.y() * dp1 - duv1.y() * dp2) * inv_det;
                dpdv = (-duv2.x() * dp1 + duv1.x() * dp2) * inv_det;
                
                // Compute Shading Normal Derivatives (Weingarten equations)
                if (m_mesh.has_normals()) {
                    const auto& normals = m_mesh.normals();
                    auto dn1 = normals[idx[1]] - normals[idx[0]];
                    auto dn2 = normals[idx[2]] - normals[idx[0]];
                    shading_dndu = (duv2.y() * dn1 - duv1.y() * dn2) * inv_det;
                    shading_dndv = (-duv2.x() * dn1 + duv1.x() * dn2) * inv_det;
                }
            } else {
                 // Degenerate UVs, create arbitrary coordinate system
                 auto frame = math::coordinate_system(ng);
                 dpdu = frame.first; dpdv = frame.second;
            }
        } else {
             // No UVs, create arbitrary coordinate system
             auto frame = math::coordinate_system(ng);
             dpdu = frame.first; dpdv = frame.second;
        }

        // 9. Orthogonalize Shading Tangents
        math::Vector<T, 3> shading_dpdu = dpdu;
        math::Vector<T, 3> shading_dpdv = dpdv;
        
        // Gram-Schmidt
        shading_dpdu = shading_dpdu - ns.to_vector() * shading_dpdu.dot(ns.to_vector());
        if (shading_dpdu.length_squared() > 0) shading_dpdu = shading_dpdu.normalized();
        
        // Ensure orthogonality for shading bitangent
        shading_dpdv = shading_dpdv - ns.to_vector() * shading_dpdv.dot(ns.to_vector());
        shading_dpdv = shading_dpdv - shading_dpdu * shading_dpdv.dot(shading_dpdu); // double orthogonalization
        if (shading_dpdv.length_squared() > 0) shading_dpdv = shading_dpdv.normalized();

        // 10. Construct Interaction
        geometry::SurfaceInteraction<T> interaction(
            p_hit - p_error, 
            p_hit + p_error,
            -ray.direction().normalized(),
            ng, ns, uv_hit,
            dpdu, dpdv, dndu, dndv,
            shading_dpdu, shading_dpdv, shading_dndu, shading_dndv
        );

        return IntersectionRecord<T>{interaction, t_hit};
    }

    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        auto idx = get_indices();
        auto result = intersect_triangle_geometry(ray, 
            m_mesh.positions()[idx[0]], 
            m_mesh.positions()[idx[1]], 
            m_mesh.positions()[idx[2]]);
        
        if (result && result->t_hit >= ray.t_min() && result->t_hit <= ray.t_max()) {
            return std::make_optional(result->t_hit);
        }
        return std::nullopt;
    }

    // --- Sampling ---

    ShapeSample<T> sample_on_shape_impl(const math::Point<T, 2>& u_sample) const {
        auto idx = get_indices();
        const auto& p0 = m_mesh.positions()[idx[0]];
        const auto& p1 = m_mesh.positions()[idx[1]];
        const auto& p2 = m_mesh.positions()[idx[2]];

        // Uniform Area Sampling
        auto b = sampler::sample_uniform_triangle_barycentric(u_sample); // b0, b1, b2
        auto p_vec = b[0] * p0.to_vector() + b[1] * p1.to_vector() + b[2] * p2.to_vector();
        auto p = math::Point<T, 3>::from_vector(p_vec);

        // Normal
        auto n = math::cross(p1 - p0, p2 - p0).normalized();
        if (m_mesh.has_normals()) {
             // Consistency check only, usually not interpolating normal for area light emission 
             // unless you have a specific requirement. For area sampling, geometric normal is often used.
             // But if consistent orientation is needed:
             auto ns = m_mesh.normals()[idx[0]] + m_mesh.normals()[idx[1]] + m_mesh.normals()[idx[2]];
             if (n.dot(ns.to_vector()) < 0) n = -n;
        } else if (m_mesh.should_flip_normal()) {
            n = -n;
        }

        // UV
        math::Point<T, 2> uv{0, 0};
        if (m_mesh.has_uvs()) {
            auto uv_vec = b[0] * m_mesh.uvs()[idx[0]].to_vector() +
                          b[1] * m_mesh.uvs()[idx[1]].to_vector() +
                          b[2] * m_mesh.uvs()[idx[2]].to_vector();
            uv = math::Point<T, 2>::from_vector(uv_vec);
        }

        auto pdf = T(1) / area_impl();
        return ShapeSample<T>{p, n, uv, pdf};
    }

    T sample_on_shape_pdf_impl(const math::Point<T, 3>& p) const {
        return T(1) / area_impl();
    }

    // -----------------------------------------------------------------------
    // Solid Angle Sampling Implementation
    // -----------------------------------------------------------------------
    ShapeSample<T> sample_on_solid_angle_impl(
        const math::Point<T, 3>& ref,
        const math::Point<T, 2>& u_sample
    ) const {
        // 如果你还没有实现复杂的球面三角形采样，
        // 一个简单的回退方案是：直接调用 Area Sampling。
        // 虽然方差大，但是是无偏的 (Unbiased)。
        // 配合上面的 pdf_impl (转换了度量)，MIS 依然能工作。
        
        // 1. 调用 Area Sampling
        auto ss = sample_on_shape_impl(u_sample);
        
        // 2. 转换 PDF: Area Measure -> Solid Angle Measure
        // PDF_solid = PDF_area * (dist^2 / cos_theta)
        // 注意：sample_on_shape_impl 返回的是 PDF_area (1/Area)
        
        auto wi = ss.point - ref;
        T dist_sq = wi.length_squared();
        if (dist_sq > 0) {
            wi /= std::sqrt(dist_sq);
            T abs_cos_theta = std::abs(ss.normal.dot(-wi));
            if (abs_cos_theta > 1e-8f) {
                ss.pdf = ss.pdf * dist_sq / abs_cos_theta;
            } else {
                ss.pdf = 0; 
            }
        }
        
        return ss;
    }

    T sample_on_solid_angle_pdf_impl(
        const math::Point<T, 3>& ref,
        const math::Point<T, 3>& p_surface
    ) const {
        // PDF = Distance^2 / (Area * |cos_theta|)
        // 这实际上是将 Area PDF 转换到了 Solid Angle Measure
        auto idx = get_indices();
        const auto& p0 = m_mesh.positions()[idx[0]];
        const auto& p1 = m_mesh.positions()[idx[1]];
        const auto& p2 = m_mesh.positions()[idx[2]];

        // 简单的 1/SolidAngle 估算比较难，通常我们反向转换 Area PDF
        T dist_sq = (p_surface - ref).length_squared();
        if (dist_sq == 0) return 0;
        
        // 计算几何法线
        auto n = math::cross(p1 - p0, p2 - p0).normalized();
        if (m_mesh.should_flip_normal()) n = -n;

        // 夹角余弦
        auto wi = (p_surface - ref).normalized();
        T abs_cos_theta = std::abs(n.dot(-wi));
        
        if (abs_cos_theta < 1e-8f) return 0;

        return dist_sq / (area_impl() * abs_cos_theta);
    }

};

} // namespace pbpt::shape
