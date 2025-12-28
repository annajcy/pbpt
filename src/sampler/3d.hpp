/**
 * @file
 * @brief Sampling routines for 1D/2D intervals, disks, spheres and hemispheres.
 */
#pragma once

#include <array>
#include "math/function.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

#include "2d.hpp"

namespace pbpt::sampler {

/**
 * @brief Sample a point uniformly distributed on a hemisphere.
 * For z, we directly use uv.x() in [0, 1] to ensure uniform distribution over the hemisphere.
 * For the radial distance r in the xy-plane, we use the relation r = sqrt(1 - z^2) to maintain uniformity.
 * For phi, we uniformly sample [0, 2pi) by multiplying uv.y() in [0, 1] by 2pi.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param radius Radius of the hemisphere.
 * @return math::Point<T, 3> Sampled point on the hemisphere.
 */
template<typename T>
inline math::Point<T, 3> sample_uniform_hemisphere(const math::Point<T, 2>& uv, T radius = 1.0) {
    T z = uv.x();
    T r = std::sqrt(std::max(T(0), T(1) - z * z));
    T phi = 2.0 * math::pi_v<T> * uv.y();
    return math::Point<T, 3>(radius * r * std::cos(phi), radius * r * std::sin(phi), radius * z);
}

/**
 * @brief Compute the probability density function for uniform hemisphere sampling.
 * The PDF is the reciprocal of the surface area of the hemisphere.
 * @tparam T Numeric type.
 * @param radius Radius of the hemisphere.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_hemisphere_pdf(T radius = 1.0) {
    return 1.0 / (2.0 * math::pi_v<T> * radius * radius);
}

/**
 * @brief Sample a point with cosine-weighted distribution on a hemisphere using concentric disk mapping.
 * This corresponds to **Malley's Method**.
 * * The algorithm works in two steps:
 * 1. Uniformly sample a point (u, v) on the unit disk using concentric mapping.
 * 2. Project this point vertically onto the hemisphere: z = sqrt(1 - u^2 - v^2).
 * * According to Malley's method, this geometric projection generates sample directions 
 * with a probability density proportional to the cosine of the zenith angle (theta).
 * This is perfect for importance sampling Lambertian (diffuse) surfaces, as it exactly 
 * cancels out the cosine term in the rendering equation.
 *
 * PDF(omega) = cos(theta) / pi
 *
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param radius Radius of the hemisphere (usually 1.0 for direction sampling).
 * @return math::Point<T, 3> Sampled point on the hemisphere.
 */
template<typename T>
math::Point<T, 3> sample_cosine_weighted_hemisphere(const math::Point<T, 2>& uv, T radius = 1.0) {
    auto d = sample_uniform_disk_concentric(uv);
    auto r2 = d.x() * d.x() + d.y() * d.y();
    auto z = std::sqrt(std::max(T(0), T(1) - r2));
    return math::Point<T, 3>{radius * d.x(), radius * d.y(), radius * z};
}

/**
 * @brief Compute the probability density function (PDF) for cosine-weighted hemisphere sampling.
 * * The PDF with respect to solid angle is proportional to the cosine of the zenith angle theta:
 * p(omega) = cos(theta) / pi
 * * Ideally, this function pairs with any sampler that generates a cosine-weighted distribution 
 * (like sample_cosine_hemisphere_concentric).
 * * @tparam T Numeric type.
 * @param p The point on the hemisphere to evaluate the PDF at.
 * @param radius Radius of the hemisphere (default 1.0).
 * @return T Probability density function value (w.r.t solid angle).
 */
template<typename T>
inline T sample_cosine_weighted_hemisphere_pdf(const math::Point<T, 3>& p, T radius = 1.0) {
    // Calculate cos(theta). For a point on a sphere, z = r * cos(theta).
    // Therefore, cos(theta) = z / r.
    T cos_theta = p.z() / radius;

    // The distribution is only defined for the upper hemisphere (z >= 0).
    if (cos_theta < 0) {
        return T(0);
    }

    // p(omega) = cos(theta) / pi
    return cos_theta / math::pi_v<T>;
}

/**
 * @brief Sample a point uniformly distributed on a sphere.
 * For z, we map uv.x() in [0, 1] to [-1, 1] using z = 1 - 2 * uv.x() to ensure uniform distribution over the sphere.
 * For the radial distance r in the xy-plane, we use the relation r = sqrt(1 - z^2) to maintain uniformity.
 * For phi, we uniformly sample [0, 2pi) by multiplying uv.y() in [0, 1] by 2pi.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param radius Radius of the sphere.
 * @return math::Point<T, 3> Sampled point on the sphere.
 */
template<typename T>
inline math::Point<T, 3> sample_uniform_sphere(const math::Point<T, 2>& uv, T radius = 1.0) {
    T z = 1.0 - 2.0 * uv.x();
    T r = std::sqrt(std::max(T(0), T(1) - z * z));
    T phi = 2.0 * math::pi_v<T> * uv.y();
    return math::Point<T, 3>(radius * r * std::cos(phi), radius * r * std::sin(phi), radius * z);
}

/**
 * @brief Compute the probability density function for uniform sphere sampling.
 * The PDF is the reciprocal of the surface area of the sphere.
 * @tparam T Numeric type.
 * @param radius Radius of the sphere.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_sphere_pdf(T radius = 1.0) {
    return 1.0 / (4.0 * math::pi_v<T> * radius * radius);
}
/**
 * @brief Uniformly samples a point on the surface of a 3D triangle.
 *
 * This function implements the Uniform Triangle Sampling algorithm using the
 * Inverse Transform Sampling method to ensure a constant probability density 
 * across the triangle's surface area.
 *
 * @details
 *
 * 1. The Naive Approach (Incorrect)
 * A naive strategy might be to sample b1 uniformly from [0, 1] and then sample 
 * b2 uniformly from [0, 1 - b1]. This is incorrect because the "width" 
 * (differential area) of the triangle decreases linearly as b1 increases. 
 * Uniformly sampling b1 would result in a higher density of points near the 
 * triangle's "tip" (where b1 is large).
 *
 * 2. Marginal Probability Density Function (PDF)
 * To achieve uniform distribution over the area, the probability of selecting a specific 
 * b1 must be proportional to the triangle's width at that point (which is 1 - b1).
 * Therefore, the marginal PDF for b1 is:
 * * p(b1) proportional to (1 - b1)
 *
 * 3. Inverse Transform Sampling
 * We seek a transformation b1 = T(u1) mapping a uniform random number u1 
 * to the target distribution.
 * First, we compute the Cumulative Distribution Function (CDF) by integrating the PDF:
 * * u1 proportional to integral(1 - b1) db1  =>  u1 = (1 - b1)^2
 * * Inverting this relationship gives the sampling formula for b1:
 * * b1 = 1 - sqrt(u1)
 *
 * 4. Conditional Sampling
 * Once b1 is fixed, b2 must be sampled uniformly along the remaining 
 * vertical segment length (1 - b1):
 * * b2 = u2 * (1 - b1)
 *
 * @tparam T Floating point type (float or double).
 * @param uv Two uniform random numbers (u1, u2) in range [0, 1).
 * @param v0 The first vertex of the triangle.
 * @param v1 The second vertex of the triangle.
 * @param v2 The third vertex of the triangle.
 * @return math::Point<T, 3> A random point uniformly distributed on the triangle surface.
 */
template<typename T>
inline math::Point<T, 3> sample_uniform_triangle(
    const math::Point<T, 2>& uv,
    const math::Point<T, 3>& v0,
    const math::Point<T, 3>& v1,
    const math::Point<T, 3>& v2
) {
    // 1. Sample the first barycentric coordinate b1 (associated with v1).
    // The sqrt pushes samples towards smaller values where the triangle is wider.
    T b1 = 1.0 - std::sqrt(uv.x());

    // 2. Sample the second barycentric coordinate b2 (associated with v2).
    // Sample uniformly within the remaining vertical height.
    T b2 = uv.y() * (1.0 - b1);

    // 3. Compute the remaining barycentric coordinate b0 (associated with v0).
    T b0 = 1.0 - b1 - b2;

    // 4. Interpolate vertices to get the final point P.
    // P = b0*v0 + b1*v1 + b2*v2
    return math::Point<T, 3>(
        b0 * v0.to_vector() + 
        b1 * v1.to_vector() + 
        b2 * v2.to_vector()
    );
}

/**
 * @brief Computes the Probability Density Function (PDF) for uniform triangle sampling.
 *
 * Since the sampling is uniform over the surface area, the PDF is constant 
 * across the entire triangle.
 *
 * @details
 * The PDF is defined as the reciprocal of the total area:
 * * p(x) = 1 / Area
 * * Where the area is calculated using the cross product:
 * * Area = 0.5 * ||(v1 - v0) cross (v2 - v0)||
 *
 * @tparam T Floating point type (float or double).
 * @param v0 The first vertex of the triangle.
 * @param v1 The second vertex of the triangle.
 * @param v2 The third vertex of the triangle.
 * @return T The PDF value (1.0 / Area). Returns 0 if the area is zero (degenerate triangle).
 */
template<typename T>
inline T sample_uniform_triangle_pdf(
    const math::Point<T, 3>& v0,
    const math::Point<T, 3>& v1,
    const math::Point<T, 3>& v2
) {
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;
    // Calculate triangle area: half of the magnitude of the cross product.
    auto area = math::cross(edge1, edge2).length() * 0.5;

    // Robustness check to prevent division by zero for degenerate triangles.
    if (math::is_zero(area)) {
        return 0;
    }

    return 1.0 / area;
}


template<typename T>
inline math::Point<T, 3> sample_uniform_triangle_barycentric(
    const math::Point<T, 2>& uv
) {
    // 1. Sample the first barycentric coordinate b1 (associated with v1).
    // The sqrt pushes samples towards smaller values where the triangle is wider.
    T b1 = 1.0 - std::sqrt(uv.x());

    // 2. Sample the second barycentric coordinate b2 (associated with v2).
    // Sample uniformly within the remaining vertical height.
    T b2 = uv.y() * (1.0 - b1);

    // 3. Compute the remaining barycentric coordinate b0 (associated with v0).
    T b0 = 1.0 - b1 - b2;

    return math::Point<T, 3>(b0, b1, b2);
}

template<typename T>
inline T sample_uniform_triangle_barycentric_pdf(const math::Point<T, 3>& barycentric) {
    return T(2);
}

}
