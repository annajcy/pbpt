/**
 * @file
 * @brief Sampling routines for 1D/2D intervals, disks, spheres and hemispheres.
 */
#pragma once

#include "point.hpp"
#include "vector.hpp"

namespace pbpt::math {

/**
 * @brief Sample a value uniformly distributed in [a, b].
 * @tparam T Numeric type.
 * @param u Uniform random variable in [0, 1].
 * @param a Lower bound of the interval.
 * @param b Upper bound of the interval.
 * @return T Sampled value in [a, b].
 */
template<typename T>
inline T sample_uniform(T u, T a = T(0), T b = T(1)) {
    return a + (b - a) * u;
}

/**
 * @brief Compute the probability density function for uniform sampling in [a, b].
 * The PDF is the reciprocal of the length of the interval.
 * @tparam T Numeric type.
 * @param x Value to evaluate the PDF at.
 * @param a Lower bound of the interval.
 * @param b Upper bound of the interval.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_pdf(T x, T a = T(0), T b = T(1)) {
    if (x < a || x > b) {
        return T(0);
    }
    return 1.0 / (b - a);
}

/**
 * @brief Sample a point uniformly distributed in a 2D box defined by x_range and y_range.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param x_range Range of the box in the x dimension.
 * @param y_range Range of the box in the y dimension.
 * @return math::Point<T, 2> Sampled point in the box.
 */
template <typename T>  
inline math::Point<T, 2> sample_uniform_2d(
    const math::Point<T, 2>& uv, 
    const math::Vector<T, 2>& x_range = math::Vector<T, 2>(T(0), T(1)), 
    const math::Vector<T, 2>& y_range = math::Vector<T, 2>(T(0), T(1))
) {
    T x = sample_uniform(uv.x(), x_range.x(), x_range.y());
    T y = sample_uniform(uv.y(), y_range.x(), y_range.y());
    return math::Point<T, 2>(x, y);
}

/**
 * @brief Compute the probability density function for uniform sampling in a 2D box defined by x_range and y_range.
 * The PDF is the product of the PDFs in each dimension.
 * @tparam T Numeric type.
 * @param p 2D point to evaluate the PDF at.
 * @param x_range Range of the box in the x dimension.
 * @param y_range Range of the box in the y dimension.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_2d_pdf(
    const math::Point<T, 2>& p,
    const math::Vector<T, 2>& x_range = math::Vector<T, 2>(T(0), T(1)), 
    const math::Vector<T, 2>& y_range = math::Vector<T, 2>(T(0), T(1))
) {
    return sample_uniform_pdf(p.x(), x_range.x(), x_range.y()) * sample_uniform_pdf(p.y(), y_range.x(), y_range.y());
} 

/**
 * @brief Sample a value from a tent distribution centered at 0 with range [-r, r].
 * The symmetric tent PDF is f(x) = (r - |x|) / r^2 for |x| ≤ r and zero elsewhere. Integrating
 * this PDF yields the CDF:
 *
 *   F(x) = ((x + r)^2) / (2 r^2),     x ∈ [-r, 0]
 *   F(x) = 1 - ((r - x)^2) / (2 r^2), x ∈ [0, r]
 *
 * Since F(0) = 1/2, inverting the CDF leads to two analytic branches governed by whether the
 * uniform variate u falls below 0.5. Solving F(x) = u on each branch produces
 * x = -r + r * sqrt(2u) for u < 0.5 and x = r - r * sqrt(2(1 - u)) for u ≥ 0.5.
 *
 * @tparam T Numeric type.
 * @param u Uniform random variable in [0, 1].
 * @param r Range of the tent distribution.
 * @return T Sampled value from the tent distribution.
 */
template <typename T>
inline T sample_tent(T u, T r = 1.0) {
    if (u < 0.5) {
        return -r + r * std::sqrt(2.0 * u);
    } else {
        return  r - r * std::sqrt(2.0 * (1.0 - u));
    }
}

/**
 * @brief Compute the probability density function for tent distribution centered at 0 with range [-r, r].
 * The PDF is triangular with unit area, reaching f(0) = 1 / r at the peak and dropping linearly to 0 at ±r,
 * which gives the closed form f(x) = (r - |x|) / r^2 for |x| ≤ r.
 * @tparam T Numeric type.
 * @param x Value to evaluate the PDF at.
 * @param r Range of the tent distribution.
 * @return T Probability density function value.
 */
template <typename T>
inline T sample_tent_pdf(T x, T r = 1.0) {
    if (std::abs(x) > r) {
        return T(0);
    }
    return 1.0 / r - std::abs(x) / (r * r);
}

/**
 * @brief Sample a 2D point from a tent distribution in each dimension.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param rx Range of the tent distribution in the x dimension.
 * @param ry Range of the tent distribution in the y dimension.
 * @return math::Point<T, 2> Sampled point from the 2D tent distribution.
 */
template<typename T>
inline math::Point<T, 2> sample_tent_2d(const math::Point<T, 2>& uv, T rx = 1.0, T ry = 1.0) {
    T x = sample_tent(uv.x(), rx);
    T y = sample_tent(uv.y(), ry);
    return math::Point<T, 2>(x, y);
}

/**
 * @brief Compute the probability density function for 2D tent distribution in each dimension.
 * The PDF is the product of the PDFs in each dimension.
 * @tparam T Numeric type.
 * @param p 2D point to evaluate the PDF at.
 * @param rx Range of the tent distribution in the x dimension.
 * @param ry Range of the tent distribution in the y dimension.
 * @return T Probability density function value.
 */template<typename T>
inline T sample_tent_2d_pdf(const math::Point<T, 2>& p, T rx = 1.0, T ry = 1.0) {
    return sample_tent_pdf(p.x(), rx) * sample_tent_pdf(p.y(), ry);
}

/**
 * @brief Sample a point uniformly distributed on a disk.
 * For theta, we uniformly sample [0, 2pi) by multiplying uv.y() in [0, 1] by 2pi.
 * For radius, we use the inverse transform sampling method. In order to preserve area uniformity, we calculate the integral of the differntial area element in polar coordinates:
 * A = ∫(0 to r) ∫(0 to 2pi) r' dr' dtheta = 2 * pi * ∫ (0 to r) r' dr' = pi * r^2
 * To sample r uniformly, we set A = u * A_max, where u is a uniform random variable in [0, 1] and A_max = pi * R^2 is the area of the disk with radius R.
 * Thus, we have: A = u * pi * R^2
 * => pi * r^2 = u * pi * R^2
 * => r^2 = u * R^2
 * => r = sqrt(u) * R
 * Then, we set r = sqrt(uv.x()) * radius.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param radius Radius of the disk.
 * @return math::Point<T, 2> Sampled point on the disk.
 */
template<typename T>
inline math::Point<T, 2> sample_uniform_disk(const math::Point<T, 2>& uv, T radius = 1.0) {
    T r = std::sqrt(uv.x()) * radius;
    T theta = 2.0 * math::pi_v<T> * uv.y();
    return math::Point<T, 2>(r * std::cos(theta), r * std::sin(theta));
}

/**
 * @brief Sample a point uniformly distributed on a disk using concentric mapping.
 * Concentric (Shirley-Chiu) mapping remaps the unit square to the unit disk by
 * first shifting uv into [-1, 1]^2 and treating each quadrant as a 45-degree
 * wedge whose radial distance equals the dominant absolute coordinate. This
 * preserves area while avoiding the polar singularity that the inverse CDF
 * approach suffers at the center, which results in significantly better
 * stratification for low-discrepancy sequences. The mapping can be written as:
 *
 *   s = 2 * uv - 1,  s ∈ [-1, 1]^2
 *   if |sx| > |sy|:  r = sx,  θ = (π / 4) * (sy / sx)
 *   else:            r = sy,  θ = (π / 2) - (π / 4) * (sx / sy)
 *
 * When sx = sy, the point lies on the branch boundaries which gives π / 4
 * When sx = -sy, the point lies on the branch boundaries which gives -π / 4
 * When sy = 0, the point lies on x-axis which gives 0 to  
 * When sx = 0, the point lies on y-axis which gives π / 2
 *
 * where the branch boundary |sx| = |sy| aligns with the diagonals that split
 * the square into eight triangular wedges. Each wedge is mapped to a sector of
 * angle π/4 with radial extent equal to the dominant coordinate, ensuring
 * r^2 is proportional to the area covered within the unit disk.
 *
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param radius Radius of the disk.
 * @return math::Point<T, 2> Sampled point on the disk.
 */
template<typename T>
inline math::Point<T, 2> sample_uniform_disk_concentric(const math::Point<T, 2>& uv, T radius = 1.0) {
    math::Point<T, 2> p_offset = 2.0 * uv.to_vector() - math::Vector<T, 2>(1, 1);

    if (p_offset.x() == 0 && p_offset.y() == 0) {
        return math::Point<T, 2>(0, 0);
    }

    T theta{}, r{};
    if (std::abs(p_offset.x()) > std::abs(p_offset.y())) {
        r = p_offset.x();
        theta = (math::pi_v<T> / 4.0) * (p_offset.y() / p_offset.x());
    } else {
        r = p_offset.y();
        theta = (math::pi_v<T> / 2.0) - (math::pi_v<T> / 4.0) * (p_offset.x() / p_offset.y());
    }

    return math::Point<T, 2>(radius * r * math::Vector<T, 2>(std::cos(theta), std::sin(theta)));
}

/**
 * @brief Compute the probability density function for uniform disk sampling.
 * The PDF is the reciprocal of the area of the disk.   
 * @tparam T Numeric type.
 * @param radius Radius of the disk.
 * @return T Probability density function value.
 */
template<typename T>
inline T sample_uniform_disk_pdf(T radius = 1.0) {
    return 1.0 / (math::pi_v<T> * radius * radius);
}

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
    auto d = math::sample_uniform_disk_concentric(uv);
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

}
