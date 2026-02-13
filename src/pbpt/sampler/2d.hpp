/**
 * @file
 * @brief Sampling routines for 1D/2D intervals, disks, spheres and hemispheres.
 */
#pragma once

#include "pbpt/math/function.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"

#include "1d.hpp"

namespace pbpt::sampler {

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
    const math::Point<T, 2>& u2d, 
    const math::Vector<T, 2>& x_range = math::Vector<T, 2>(T(0), T(1)), 
    const math::Vector<T, 2>& y_range = math::Vector<T, 2>(T(0), T(1))
) {
    T x = sample_uniform(u2d.x(), x_range.x(), x_range.y());
    T y = sample_uniform(u2d.y(), y_range.x(), y_range.y());
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
 * @brief Sample a 2D point from a tent distribution in each dimension.
 * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param rx Range of the tent distribution in the x dimension.
 * @param ry Range of the tent distribution in the y dimension.
 * @return math::Point<T, 2> Sampled point from the 2D tent distribution.
 */
template<typename T>
inline math::Point<T, 2> sample_tent_2d(const math::Point<T, 2>& u2d, T rx = 1.0, T ry = 1.0) {
    T x = sample_tent(u2d.x(), rx);
    T y = sample_tent(u2d.y(), ry);
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
inline math::Point<T, 2> sample_uniform_disk(const math::Point<T, 2>& u2d, T radius = 1.0) {
    T r = std::sqrt(u2d.x()) * radius;
    T theta = 2.0 * math::pi_v<T> * u2d.y();
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
inline math::Point<T, 2> sample_uniform_disk_concentric(const math::Point<T, 2>& u2d, T radius = 1.0) {
    math::Point<T, 2> p_offset = 2.0 * u2d.to_vector() - math::Vector<T, 2>(1, 1);

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
 * @brief Sample a 2D point from a Gaussian distribution with independent X and Y components.
 * This effectively performs 1D Gaussian sampling on each axis.
 * * @tparam T Numeric type.
 * @param uv 2D point with coordinates in [0, 1] used for sampling.
 * @param mean The center (mean) of the 2D distribution.
 * @param stddev The standard deviation for x and y dimensions respectively.
 * @return math::Point<T, 2> Sampled point.
 */
template <typename T>
inline math::Point<T, 2> sample_gaussian_2d(
    const math::Point<T, 2>& u2d,
    const math::Point<T, 2>& mean = math::Point<T, 2>(T(0), T(0)),
    const math::Vector<T, 2>& stddev = math::Vector<T, 2>(T(1), T(1))
) {
    // Treat X and Y independently
    T x = sample_gaussian(u2d.x(), mean.x(), stddev.x());
    T y = sample_gaussian(u2d.y(), mean.y(), stddev.y());
    return math::Point<T, 2>(x, y);
}

/**
 * @brief 使用 Box-Muller 变换生成 2D 高斯分布采样。
 * 先生成标准正态分布 N(0,1)，然后通过 X = mean + stddev * Z 进行线性变换。
 * * @tparam T Numeric type.
 * @param u1 Uniform random number in (0, 1].
 * @param u2 Uniform random number in [0, 1].
 * @param mean The center (mean) of the 2D distribution.
 * @param stddev The standard deviation for x and y dimensions respectively.
 * @return math::Point<T, 2> Sampled point.
 */
template <typename T>
inline math::Point<T, 2> sample_gaussian_2d_box_muller(
    const math::Point<T, 2>& u2d,
    const math::Point<T, 2>& mean = math::Point<T, 2>(T(0), T(0)),
    const math::Vector<T, 2>& stddev = math::Vector<T, 2>(T(1), T(1))
) {
    T u1 = u2d.x();
    T u2 = u2d.y();
    // 1. 防止 log(0)
    if (u1 < 1e-6) u1 = 1e-6;

    // 2. Box-Muller 基础公式生成标准正态分布 Z ~ N(0, 1)
    T r = std::sqrt(T(-2) * std::log(u1));
    T theta = T(2) * math::pi_v<T> * u2;
    
    T z0 = r * std::cos(theta);
    T z1 = r * std::sin(theta);

    // 3. 线性变换应用 mean 和 stddev
    // x = mu_x + sigma_x * z0
    // y = mu_y + sigma_y * z1
    return math::Point<T, 2>(
        mean.x() + stddev.x() * z0,
        mean.y() + stddev.y() * z1
    );
}

/**
 * @brief Compute the probability density function for the 2D Gaussian distribution.
 * Since dimensions are independent, PDF(x, y) = PDF(x) * PDF(y).
 * * @tparam T Numeric type.
 * @param p The point to evaluate.
 * @param mean The center (mean) of the 2D distribution.
 * @param stddev The standard deviation for x and y dimensions respectively.
 * @return T Probability density.
 */
template <typename T>
inline T sample_gaussian_2d_pdf(
    const math::Point<T, 2>& p,
    const math::Point<T, 2>& mean = math::Point<T, 2>(T(0), T(0)),
    const math::Vector<T, 2>& stddev = math::Vector<T, 2>(T(1), T(1))
) {
    return sample_gaussian_pdf(p.x(), mean.x(), stddev.x()) * 
            sample_gaussian_pdf(p.y(), mean.y(), stddev.y());
}

}
