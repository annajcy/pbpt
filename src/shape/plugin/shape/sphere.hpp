/**
 * @file sphere.hpp
 * @brief Sphere shape implementation with internal transformation handling.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "geometry/bounds.hpp"
#include "geometry/directional_cone.hpp"
#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "geometry/transform.hpp"
#include "math/function.hpp"
#include "math/normal.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "shape/shape.hpp"

namespace pbpt::shape {

/// Helper struct for quadratic solution
template<typename T>
struct BasicSphereIntersectionResult {
    T t0, t1;
};

/// Helper struct for detailed intersection result
template<typename T>
struct SphereIntersectionResult {
    math::Point<T, 3> p_obj; // Hit point in Object Space
    T t_hit;                 // Ray parameter t
    T phi;                   // Azimuthal angle in Object Space
};

/**
 * @brief Sphere shape supporting partial spheres, transformations, and analytic derivatives.
 *
 * This class handles its own object-to-render space transformations.
 * It computes full differential geometry (dpdu, dpdv, dndu, dndv) required for
 * advanced texturing and shading.
 *
 * @tparam T Scalar type.
 */
template<typename T>
class Sphere : public Shape<Sphere<T>, T> {
    friend class Shape<Sphere<T>, T>;
private:
    // Sphere parameters
    T m_radius{};
    T m_z_min{};
    T m_z_max{};
    T m_phi_max{};

    // Transformations
    geometry::Transform<T> m_object_to_render;
    geometry::Transform<T> m_render_to_object;
    
    // Normal orientation flag (XOR of UserReverse and MatrixHandedness)
    bool m_flip_normals; 

public:
    /**
     * @brief Constructs a partial sphere with transform.
     */
    Sphere(
        const geometry::Transform<T>& object_to_render,
        bool reverse_orientation,
        T radius,
        T z_min,
        T z_max,
        T phi_max
    ) : m_radius(radius),
        m_z_min(std::clamp(z_min, -radius, radius)),
        m_z_max(std::clamp(z_max, -radius, radius)),
        m_phi_max(std::clamp(phi_max, static_cast<T>(0), static_cast<T>(2 * math::pi_v<T>))),
        m_object_to_render(object_to_render),
        m_render_to_object(object_to_render.inversed())
    {
        if (m_z_min > m_z_max) {
            std::swap(m_z_min, m_z_max);
        }
        // Pre-calculate flip logic: User Request XOR Transform Handedness
        // Note: transform_surface_interaction handles matrix handedness automatically,
        // so we only need to track if we need an *additional* flip due to user request.
        // However, usually we handle everything consistently. 
        // Let's rely on standard PBRT logic: 
        // The SurfaceInteraction transform handles the matrix part. 
        // We only explicitly handle the user's ReverseOrientation.
        m_flip_normals = reverse_orientation; 
    }

    /**
     * @brief Constructs a full sphere.
     */
    Sphere(
        const geometry::Transform<T>& object_to_render, 
        bool reverse_orientation, 
        T radius
    ) : Sphere(object_to_render, reverse_orientation, radius, -radius, radius, static_cast<T>(2 * math::pi_v<T>)) {}

    T radius() const { return m_radius; }
    T z_min() const { return m_z_min; }
    T z_max() const { return m_z_max; }
    T phi_max() const { return m_phi_max; }

    T z_min_theta() const {
        return std::acos(std::clamp(m_z_min / m_radius, T(-1), T(1)));
    }

    T z_max_theta() const {
        return std::acos(std::clamp(m_z_max / m_radius, T(-1), T(1)));
    }

private:
    // -----------------------------------------------------------------------
    // Shape Interface Implementation
    // -----------------------------------------------------------------------

    geometry::Transform<T> object_to_render_transform_impl() const {
        return m_object_to_render;
    }

    geometry::Transform<T> render_to_object_transform_impl() const {
        return m_render_to_object;
    }

    geometry::Bounds<T, 3> bounding_box_impl() const {
        // 1. Create Object Space Bounds
        geometry::Bounds<T, 3> object_bounds(
            math::Point<T, 3>(-m_radius, -m_radius, m_z_min),
            math::Point<T, 3>(m_radius, m_radius, m_z_max)
        );
        // 2. Transform to Render Space
        return m_object_to_render.transform_bounds(object_bounds);
    }

    geometry::DirectionalCone<T> normal_bounding_cone_impl() const {
        // 1. 安全检查：如果有缩放，或者是全球，直接返回全集
        // 非均匀缩放会导致法线分布变成椭圆锥，简单变换轴和角度是不够的
        // 为了安全起见，如果存在缩放，我们回退到全方向
        if (m_object_to_render.has_scale() || (m_z_min <= -m_radius)) {
            return geometry::DirectionalCone<T>::entire_sphere();
        }

        // 2. 计算 Object Space 的参数
        // 对于 zMin 切割的球体，法线主要指向 +Z 方向
        // 锥体的张角由最低点的法线决定。
        // 最低点的 z 坐标是 m_z_min，对应的法线 z 分量是 m_z_min / m_radius
        math::Normal<T, 3> axis_obj(0, 0, 1);
        T cos_theta_obj = m_z_min / m_radius;

        // 3. 变换轴到 Render Space
        // 关键点：必须使用 transform_normal 而不是 transform_vector！
        // 因为这是法线向量，需要应用逆转置矩阵才能保持垂直性
        math::Normal<T, 3> axis_render = m_object_to_render.transform_normal(axis_obj);
        
        // 归一化是必须的，因为变换可能会改变长度
        axis_render = axis_render.normalized();

        // 4. 处理翻转逻辑
        // 我们在构造函数里计算的 m_flip_normals = ReverseOrientation ^ SwapsHandedness
        // 如果为真，说明实际的几何法线是指向内部的（或者反向的）
        // 所以我们的包围锥轴也要反过来
        if (m_flip_normals) {
            axis_render = -axis_render;
        }

        // 5. 返回结果
        // DirectionCone 构造函数接受 (中心轴, cosTheta)
        return geometry::DirectionalCone<T>(
            math::Vector<T, 3>(axis_render), // 转换回 Vector 传递给构造函数
            cos_theta_obj
        );
    }

    std::optional<T> is_intersected_impl(const geometry::Ray<T, 3>& ray) const {
        // 1. Transform Ray to Object Space
        auto ray_obj = m_render_to_object.transform_ray(ray);

        // 2. Solve intersection in Object Space
        auto result_opt = intersect_object_space(ray_obj);
        
        if (result_opt.has_value()) {
            return std::make_optional(result_opt->t_hit);
        }
        return std::nullopt;
    }

    std::optional<IntersectionRecord<T>> intersect_impl(const geometry::Ray<T, 3>& ray) const {
        // 1. Transform Ray to Object Space
        auto ray_obj = m_render_to_object.transform_ray(ray);

        // 2. Solve intersection in Object Space
        auto result_opt = intersect_object_space(ray_obj);
        if (!result_opt.has_value()) {
            return std::nullopt;
        }

        const auto& result = result_opt.value();

        // 3. Compute Differential Geometry (in Object Space)
        auto si_obj = compute_object_interaction(ray_obj, result);

        // 4. Transform Interaction to Render Space
        // This handles: Point, Error, Normal (with matrix handedness), Derivatives
        auto si_render = m_object_to_render.transform_surface_interaction(si_obj);

        // 5. Apply User-Requested Flip (ReverseOrientation)
        // Note: Matrix handedness swap is already handled inside transform_surface_interaction.
        // We only apply the extra flip if user explicitly requested ReverseOrientation.
        if (m_flip_normals) {
            si_render.flip_normal();
        }

        geometry::ShadingInfo<T> shading{si_render.n()};
        return std::make_optional(IntersectionRecord<T>{si_render, shading, std::nullopt, result.t_hit});
    }

    std::optional<IntersectionRecord<T>> intersect_impl(const geometry::RayDifferential<T, 3>& ray) const {
        auto hit = intersect_impl(ray.main_ray());
        if (!hit) return std::nullopt;
        hit->differentials = geometry::compute_surface_differentials(hit->interaction, ray);
        return hit;
    }

    T area_impl() const {
        // Approximate area in render space
        // Area = phi_max * r * (z_max - z_min) * transform_scale
        T object_area = m_phi_max * m_radius * (m_z_max - m_z_min);
        return m_object_to_render.transform_volume(object_area); 
    }

    ShapeSample<T> sample_on_shape_impl(const math::Point<T, 2>& u_sample) const {
        // 1. Sample in Object Space
        T z = m_z_min + u_sample.x() * (m_z_max - m_z_min);
        T phi = u_sample.y() * m_phi_max;

        T z_sq = z * z;
        T r_sq = m_radius * m_radius;
        T r_xy = std::sqrt(std::max(T(0), r_sq - z_sq));

        T x = r_xy * std::cos(phi);
        T y = r_xy * std::sin(phi);

        math::Point<T, 3> p_obj(x, y, z);
        math::Normal<T, 3> n_obj = math::Normal<T, 3>::from_vector((p_obj - math::Point<T, 3>(0,0,0)).normalized());
        
        if (m_flip_normals) n_obj = -n_obj;

        // Compute UV
        T u = phi / m_phi_max;
        T theta = std::acos(std::clamp(z / m_radius, T(-1), T(1)));
        T v = (theta - z_min_theta()) / (z_max_theta() - z_min_theta());

        // 2. Transform to Render Space
        auto p_render = m_object_to_render.transform_point(p_obj);
        auto n_render = m_object_to_render.transform_normal(n_obj).normalized();
        // transform_normal doesn't handle handedness flip, so we check matrix
        if (m_object_to_render.is_swaps_handedness()) n_render = -n_render;

        ShapeSample<T> sample;
        sample.point = p_render;
        sample.normal = n_render;
        sample.uv = math::Point<T, 2>(u, v);
        sample.pdf = T(1) / area_impl();

        return sample;
    }

    T sample_on_shape_pdf_impl(const math::Point<T, 3>& p_render) const {
        // Check if point is on sphere (approximate logic via bounds or inverse transform)
        // For simplicity, just return 1/Area assuming it's on surface
        return T(1) / area_impl();
    }

    // -----------------------------------------------------------------------
    // Solid Angle Sampling Implementation
    // -----------------------------------------------------------------------

    ShapeSample<T> sample_on_solid_angle_impl(
        const math::Point<T, 3>& ref,
        const math::Point<T, 2>& u_sample
    ) const {
        // 1. Transform reference point to Object Space
        auto p_ref_obj = m_render_to_object.transform_point(ref);

        // 2. Check conditions for Fallback to Area Sampling
        // Fallback if:
        // a. Point is inside the sphere (dist < radius)
        // b. Sphere is partial (cut by z_min, z_max, or phi_max)
        T dist_sq = (p_ref_obj - math::Point<T, 3>(0, 0, 0)).length_squared();
        T radius_sq = m_radius * m_radius;

        bool is_partial = (std::abs(m_phi_max - 2 * math::pi_v<T>) > 1e-4f) ||
                          (m_z_min > -m_radius + 1e-4f) ||
                          (m_z_max < m_radius - 1e-4f);

        if (dist_sq <= radius_sq || is_partial) {
            // --- Fallback: Area Sampling ---
            ShapeSample<T> ss = sample_on_shape_impl(u_sample); // Returns PDF w.r.t Area

            // Convert Area PDF to Solid Angle PDF:
            // PDF_omega = PDF_area * (dist^2 / cos_theta)
            auto wi = ss.point - ref;
            T dist_sq_surf = wi.length_squared();
            if (dist_sq_surf == 0) {
                ss.pdf = 0;
            } else {
                T dist = std::sqrt(dist_sq_surf);
                wi /= dist; // Normalize
                T abs_cos_theta = std::abs(ss.normal.dot(-wi));
                if (abs_cos_theta < 1e-8f) {
                    ss.pdf = 0;
                } else {
                    ss.pdf *= (dist_sq_surf / abs_cos_theta);
                }
            }
            return ss;
        }

        // --- Solid Angle Sampling (Cone Sampling) ---
        
        // 3. Compute Cone parameters
        T dist = std::sqrt(dist_sq);
        T sin_theta_max = m_radius / dist;
        T sin_theta_max_sq = sin_theta_max * sin_theta_max;
        T cos_theta_max = std::sqrt(std::max(T(0), T(1) - sin_theta_max_sq));

        // 4. Sample Uniform Cone around +Z axis first
        // PDF_cone = 1 / (2 * pi * (1 - cos_theta_max))
        T cos_theta = (T(1) - u_sample.x()) + u_sample.x() * cos_theta_max;
        T sin_theta = std::sqrt(std::max(T(0), T(1) - cos_theta * cos_theta));
        T phi = u_sample.y() * 2 * math::pi_v<T>;

        // 5. Compute Angle alpha for the intersection point
        // Using Law of Cosines on the triangle (Origin, Ref, HitPoint)
        // d_c = dist, r = radius
        // We need the distance 't' from Ref to HitPoint.
        // r^2 = d_c^2 + t^2 - 2 * d_c * t * cos_theta
        // This is a quadratic equation for t. 
        // Simpler approach derived for spheres (PBRT v4 Safe Ray Intersection):
        T d_c = dist;
        T ds = d_c * cos_theta - std::sqrt(std::max(T(0), m_radius * m_radius - d_c * d_c * sin_theta * sin_theta));
        
        // Compute angle alpha (angle from center to hit point, relative to Ref-Center line)
        T cos_alpha = (d_c * d_c + m_radius * m_radius - ds * ds) / (2 * d_c * m_radius);
        T sin_alpha = std::sqrt(std::max(T(0), T(1) - cos_alpha * cos_alpha));

        // 6. Construct Coordinate System for Center -> Ref
        // The surface normal should be oriented from center to the hit point.
        auto w_z = (p_ref_obj - math::Point<T, 3>(0, 0, 0)).normalized();
        auto [w_x, w_y] = math::coordinate_system(w_z); // Assuming you have this helper

        // 7. Compute Surface Normal and Point in Object Space
        // We rotate the sampled vector (sin_alpha, cos_alpha) into the frame
        math::Normal<T, 3> n_obj = math::Normal<T, 3>(
            sin_alpha * std::cos(phi) * w_x +
            sin_alpha * std::sin(phi) * w_y +
            cos_alpha * w_z
        ).normalized();
        
        // Point is simply radius * normal
        math::Point<T, 3> p_obj = math::Point<T, 3>(0,0,0) + n_obj.to_vector() * m_radius;

        // Apply User Flip (Object Space)
        if (m_flip_normals) n_obj = -n_obj;

        // 8. Compute UV (reuse compute logic or simplify)
        // Since we are full sphere here, mapping is standard
        T phi_surf = std::atan2(p_obj.y(), p_obj.x());
        if (phi_surf < 0) phi_surf += 2 * math::pi_v<T>;
        T u = phi_surf / m_phi_max;
        T theta_surf = std::acos(std::clamp(p_obj.z() / m_radius, T(-1), T(1)));
        T v = (theta_surf - z_min_theta()) / (z_max_theta() - z_min_theta());

        // 9. Transform to Render Space
        auto p_render = m_object_to_render.transform_point(p_obj);
        auto n_render = m_object_to_render.transform_normal(n_obj).normalized();
        if (m_object_to_render.is_swaps_handedness()) n_render = -n_render;

        // 10. Compute Solid Angle PDF
        // PDF = 1 / SolidAngle = 1 / (2 * pi * (1 - cos_theta_max))
        T pdf = T(1) / (2 * math::pi_v<T> * (1 - cos_theta_max));

        return ShapeSample<T>{p_render, n_render, math::Point<T, 2>(u, v), pdf};
    }

    T sample_on_solid_angle_pdf_impl(
        const math::Point<T, 3>& ref,
        const math::Point<T, 3>& p_surface
    ) const {
        // 1. Transform points to Object Space
        auto p_ref_obj = m_render_to_object.transform_point(ref);
        auto p_surf_obj = m_render_to_object.transform_point(p_surface);

        // 2. Check Fallback Conditions
        T dist_sq = (p_ref_obj - math::Point<T, 3>(0, 0, 0)).length_squared();
        T radius_sq = m_radius * m_radius;

        bool is_partial = (std::abs(m_phi_max - 2 * math::pi_v<T>) > 1e-4f) ||
                          (m_z_min > -m_radius + 1e-4f) ||
                          (m_z_max < m_radius - 1e-4f);

        if (dist_sq <= radius_sq || is_partial) {
            // --- Fallback PDF: Area PDF converted to Solid Angle ---
            // PDF_omega = (1/Area) * (dist^2 / cos_theta)
            
            // Re-calculate normal at p_surf_obj to get cos_theta
            auto n_obj = math::Normal<T, 3>::from_vector((p_surf_obj - math::Point<T, 3>(0,0,0)).normalized());
            if (m_flip_normals) n_obj = -n_obj;
            // Note: We need render space normal and direction for correct cos_theta if non-uniform scale
            // But doing it in object space is fine if we convert Area properly.
            // Safer to do generic conversion in Render Space:
            
            T area = area_impl();
            auto wi = p_surface - ref;
            T dist_sq_real = wi.length_squared();
            T dist_real = std::sqrt(dist_sq_real);
            wi /= dist_real;

            // Use the provided surface point logic or re-calculate normal?
            // Assuming p_surface is on the shape, we can get normal via sample_on_shape_pdf logic 
            // but that function usually just returns 1/Area.
            // Let's compute render space normal:
            auto n_render = m_object_to_render.transform_normal(n_obj).normalized();
            if (m_object_to_render.is_swaps_handedness()) n_render = -n_render;

            T abs_cos_theta = std::abs(n_render.dot(-wi));
            if (abs_cos_theta < 1e-8f) return 0;
            
            return dist_sq_real / (area * abs_cos_theta);
        }

        // --- Solid Angle PDF ---
        T dist = std::sqrt(dist_sq);
        T sin_theta_max = m_radius / dist;
        T cos_theta_max = std::sqrt(std::max(T(0), T(1) - sin_theta_max * sin_theta_max));

        return T(1) / (2 * math::pi_v<T> * (1 - cos_theta_max));
    }

private:
    // -----------------------------------------------------------------------
    // Internal Helper Methods
    // -----------------------------------------------------------------------

    std::optional<SphereIntersectionResult<T>> intersect_object_space(const geometry::Ray<T, 3>& ray) const {
        auto a = ray.direction().length_squared();
        auto origin_vec = ray.origin().to_vector();
        auto b = static_cast<T>(2) * ray.direction().dot(origin_vec);
        auto c = origin_vec.length_squared() - m_radius * m_radius;

        auto discriminant = b * b - static_cast<T>(4) * a * c;
        if (discriminant < static_cast<T>(0)) return std::nullopt;

        auto sqrt_discriminant = std::sqrt(discriminant);
        auto q = (b < 0) ? -0.5 * (b - sqrt_discriminant) : -0.5 * (b + sqrt_discriminant);
        auto t0 = q / a;
        auto t1 = c / q;
        if (t0 > t1) std::swap(t0, t1);

        if (t0 > ray.t_max() || t1 < ray.t_min()) return std::nullopt;

        auto t_hit = t0;
        if (t_hit < ray.t_min()) {
            t_hit = t1;
            if (t_hit > ray.t_max()) return std::nullopt;
        }

        auto p_hit = ray.at(t_hit);
        
        // Refine intersection point to sphere surface to reduce error
        auto p_refined = p_hit.to_vector() * (m_radius / (p_hit - math::Point<T, 3>(0,0,0)).length());
        if (p_refined.x() == 0 && p_refined.y() == 0 && p_refined.z() == 0) p_refined = p_hit; // Safety

        auto phi = std::atan2(p_refined.y(), p_refined.x());
        if (phi < 0) phi += 2 * math::pi_v<T>;

        if (p_refined.z() < m_z_min || p_refined.z() > m_z_max || phi > m_phi_max) return std::nullopt;

        return SphereIntersectionResult<T>{math::Point<T, 3>(p_refined), T(t_hit), T(phi)};
    }

    geometry::SurfaceInteraction<T> compute_object_interaction(
        const geometry::Ray<T, 3>& ray_obj, 
        const SphereIntersectionResult<T>& intersection
    ) const {
        // 1. Calculate UV and Angular parameters
        T phi = intersection.phi;
        T p_hit_z = std::clamp(intersection.p_obj.z(), m_z_min, m_z_max); // Safety clamp
        
        T u = phi / m_phi_max;
        T cos_theta = std::clamp(p_hit_z / m_radius, T(-1), T(1));
        T theta = std::acos(cos_theta);
        T v = (theta - z_min_theta()) / (z_max_theta() - z_min_theta());

        // 2. Compute Derivatives (dpdu, dpdv)
        T z_radius = std::sqrt(std::max(T(0), intersection.p_obj.x() * intersection.p_obj.x() + 
                                            intersection.p_obj.y() * intersection.p_obj.y()));
        T inv_z_radius = (z_radius == 0) ? T(0) : T(1) / z_radius;
        T cos_phi = intersection.p_obj.x() * inv_z_radius;
        T sin_phi = intersection.p_obj.y() * inv_z_radius;

        // dp/du: (-y, x, 0) scaled by phi range
        math::Vector<T, 3> dpdu(
            -m_phi_max * intersection.p_obj.y(),
            m_phi_max * intersection.p_obj.x(),
            0
        );

        // dp/dv: Tangent along longitude
        T theta_range = z_max_theta() - z_min_theta();
        T sin_theta = std::sqrt(std::max(T(0), T(1) - cos_theta * cos_theta));
        
        math::Vector<T, 3> dpdv(
             p_hit_z * cos_phi,
             p_hit_z * sin_phi,
            -m_radius * sin_theta
        );
        dpdv = dpdv * theta_range;

        // 3. Compute Normal
        math::Normal<T, 3> N = math::Normal<T, 3>::from_vector(
            (intersection.p_obj - math::Point<T, 3>(0,0,0)).normalized()
        );

        // 4. Compute Error Bounds (for robust intersection)
        math::Vector<T, 3> p_error = math::gamma<T>(5) * math::Vector<T, 3>(
            std::abs(intersection.p_obj.x()),
            std::abs(intersection.p_obj.y()),
            std::abs(intersection.p_obj.z())
        );

        return geometry::SurfaceInteraction<T>(
            intersection.p_obj - p_error,
            intersection.p_obj + p_error,
            -ray_obj.direction().normalized(),
            N,
            math::Point<T, 2>(u, v),
            dpdu,
            dpdv
        );
    }
};

} // namespace pbpt::shape
