#include <cmath>
#include <cstdlib>
#include <iostream>

#include "core/interaction.hpp"
#include "core/radiometric_integrals.hpp"
#include "core/radiometry.hpp"
#include "core/shape.hpp"
#include "core/spectrum.hpp"
#include "integrator/monte_carlo.hpp"
#include "geometry/bounds.hpp"
#include "geometry/frame.hpp"
#include "integrator/random_generator.hpp"
#include "math/homogeneous.hpp"
#include "math/interval.hpp"
#include "math/matrix.hpp"
#include "math/normal.hpp"
#include "math/octahedral.hpp"
#include "math/point.hpp"
#include "math/quaternion.hpp"
#include "geometry/ray.hpp"
#include "geometry/spherical.hpp"
#include "geometry/transform.hpp"
#include "math/format.hpp"
#include "math/vector.hpp"
#include "math/function.hpp"
#include "geometry/directional_cone.hpp"
#include "math/type_alias.hpp"
#include "ml/activation.hpp"
#include "ml/network.hpp"

using namespace pbpt;

int main() {
    std::cout << "Hello, World!" << std::endl;

    geometry::Sphere3 sp(math::Vec2{math::deg2rad(45.0), math::deg2rad(45.0)}, 1.0);
    std::cout << sp.to_cartesian() << std::endl;

    geometry::Ray3 ray(math::Pt3{0.0, 0.0, 0.0}, math::Vec3{1.0, 1.0, 1.0});
    std::cout << "Ray Origin: " << ray.origin() << ", Direction: " << ray.direction() << std::endl;

    std::array<geometry::Ray3, 2> rds = {
        geometry::Ray3(math::Pt3{0.1, 0.0, 0.0}, math::Vec3{1.0, 0.1, 0.0}),
        geometry::Ray3(math::Pt3{0.0, 0.1, 0.0}, math::Vec3{1.0, 0.0, 0.1})
    };

    geometry::RayDiff3 ray_diff(ray, rds);
    ray_diff.scale(2.0);
    std::cout << "Ray Differential Origin: " << ray_diff.main_ray().origin() << ", Direction: " << ray_diff.main_ray().direction()
              << std::endl;
    std::cout << "Ray Differential Origin: " << ray_diff.x().origin() << ", Direction: " << ray_diff.x().direction()
              << std::endl;
    std::cout << "Ray Differential Origin: " << ray_diff.y().origin() << ", Direction: " << ray_diff.y().direction()
              << std::endl;

    geometry::Bounds3 box(math::Pt3{0.0, 0.0, 0.0}, math::Pt3{1.0, 1.0, 1.0});

    math::Quat quaternion(1.0, math::Vec3{0.0, 1.0, 0.0});
    std::cout << "Quaternion: " << quaternion << std::endl;
    std::cout << "Quaternion Axis: " << quaternion.to_axis_angle().second << std::endl;
    std::cout << "Quaternion Angle: " << quaternion.to_axis_angle().first << std::endl;

    auto transform = geometry::Transform<float>::translate(math::Vec3{1.0, 2.0, 3.0});
    std::cout << "Transform Matrix: " << transform.matrix() << std::endl;

    math::Homo3 homo3 = math::Homo3::from_vector(math::Vec3{1.0, 2.0, 3.0});
    std::cout << "Homo3: " << homo3 << std::endl;

    math::Vec3 vec(1.0, 2.0, 3.0);

    auto vec_d = vec.type_cast<double>();
    constexpr math::Vec3 vec2(4.0, 5.0, 6.0);
    math::Vec3 vec3 = vec + vec2;
    std::cout << "Vector Addition: " << vec3 << std::endl;

    math::Normal3 normal(vec);
    std::cout << "Normal3 from Vector: " << normal << std::endl;
    math::Normal3 normal3 = math::Normal3::from_vector(vec);
    std::cout << "Normal3: " << normal3 << std::endl;

    std::cout << "Vector (float): " << vec.normalized() << std::endl;
    math::OctahedralVector<float> octahedral_vec(vec.normalized());
    std::cout << "Octahedral Vector: " << octahedral_vec.decode() << std::endl;

    std::cout << "Vector (double): " << vec_d.normalized().type_cast<double>() << std::endl;
    math::OctahedralVector<double> octahedral_vec_d(vec_d.normalized());
    std::cout << "Octahedral Vector (double): " << octahedral_vec_d.decode() << std::endl;

    geometry::DirectionalCone<math::Float> dc(
        math::Vector<math::Float, 3>(0, 0, 1), math::deg2rad(80.0)
    );

    std::cout << "Directional Cone Direction: " << dc.direction() << std::endl;
    std::cout << "Directional Cone Angle: " << math::rad2deg(dc.angle()) << std::endl;

    bool contains = dc.contains(math::Vector<math::Float, 3>(0.5, 0.5, 0.5));
    std::cout << "Contains Point? " << contains << std::endl;

    math::Mat4 mat4 = math::Mat4::identity();
    std::cout << "4x4 Identity Matrix:\n" << mat4 << std::endl;

    auto m = mat4 + math::Matrix<double, 4, 4>::identity();
    
    std::cout << (m.at(0, 1) = 5) << std::endl;

    auto row = m[0][1];
    std::cout << row << std::endl;

    auto subm = math::Mat4::MatView<2, 2>(m, 1, 1);
    subm.visit([](auto& val, int row, int col) {
        std::cout << "Submatrix Element [" << row << "][" << col << "]: " << val << std::endl;
    });

    std::cout << "4x4 Submatrix:\n" << subm.to_matrix() << std::endl;

    std::cout << "4x4 Matrix Addition:\n" << m << std::endl;

    auto ho = math::Homo3::from_vector(math::Vec3(1.0, 2.0, 3.0));
    auto ho2 = math::Homo3::from_point(math::Pt3(4.0, 5.0, 6.0));
    std::cout << "Homo3 from Vector: " << ho << std::endl;
    std::cout << "Homo3 from Point: " << ho2 << std::endl;

    auto ho3 = ho + ho2;
    std::cout << "Homo3 Sum: " << ho3 << std::endl;
    std::cout << "Homo3 Vector: " << ho3.to_point() << std::endl;

    auto pp = math::Homo3::from_point(math::Pt3(1.0, 2.0, 3.0));
    std::cout << "Homo3 from Point: " << pp << std::endl;

    geometry::Trans tr = geometry::Trans::translate(math::Vec3(1.0, 2.0, 3.0));
    std::cout << "Translation Transform: " << tr.matrix() << std::endl;

    geometry::Frame<double> frame(math::Vec3(0.0, 1.0, 0.0));
    std::cout << "Frame t: " << frame.t() << std::endl;
    std::cout << "Frame b: " << frame.b() << std::endl;
    std::cout << "Frame n: " << frame.n() << std::endl;

    math::Interval<double> interval(0.0, 1.0);
    std::cout << "Interval low: " << interval.m_low << ", high: " << interval.m_high << std::endl;

    using I = pbpt::math::Interval<float>;

    static_assert(I(0,1) <  I(2,3));
    static_assert(I(0,1) <= I(2,3));
    static_assert(!(I(0,2) <  I(1,3)));  // 重叠不成立
    static_assert(!(I(0,2) <= I(1,3)));

    static_assert(I(2,3) >  I(0,1));
    static_assert(I(2,3) >= I(0,2));

    static_assert(I(0,1) <  2.0f);
    static_assert(I(0,2) <= 2.0f);
    static_assert(!(I(1,3) <= 2.0f));

    static_assert(I(2,3) >  2.0f == false);
    static_assert(I(2,3) >= 2.0f);

    math::Pt3Interv p(
        math::Interval<float>::from_value_with_error(0.0f, 0.1f),
        math::Interval<float>::from_value_with_error(1.0f, 0.1f),
        math::Interval<float>::from_value_with_error(2.0f, 0.1f)
    );

    std::cout << "Point3Eps: " << p << std::endl;

    std::cout << "Point3Eps + vec1: " << p + math::Vector<float, 3>(1.0f, 1.0f, 1.0f) << std::endl;
    std::cout << "Point3Eps - vec1: " << p - math::Vector<float, 3>(1.0f, 1.0f, 1.0f) << std::endl;

    core::Sphere sphere;

    using namespace pbpt::integrator;

    UniformSampler sampler(1, {0.0, pi_v<Float> / 2});

    auto f = [](std::span<const double> x) {
        return std::cos(x[0]);
    };

    auto [result, var] = monte_carlo_integrate(f, sampler, 1000000);

    std::cout << "Integral ≈ " << result << ", Variance = " << var << "\n";
    std::cout << "Ground truth = 1.0\n";

    auto L = core::Radiance<Float>(1.0);
    auto irrad = L * core::SolidAngle<Float>(1.0);
    std::cout << "Irradiance: " << irrad << std::endl;

    math::RandomGenerator<double, 2> rng2d(42);
    core::UniformHemisphereDomain<double> hemisphere{};

    math::Normal3 norm(0.0f, 0.0f, 1.0f);
    int sample_count = 10000;
    auto n = norm.to_vector();
    auto res = core::integrate<double>(hemisphere, [&n](const math::Vector<double, 3>& wi) {
        return 1.0 * n.dot(wi);
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res << ", Expected: " << pi_v<double> << "\n";

    core::UniformDiskDomain<double> disk{};
    auto res_disk = core::integrate<double>(disk, [](const math::Point<double, 2>& p) {
        return 1.0f;
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res_disk << ", Expected: " << pi_v<double> << "\n";

    core::ProjectedHemisphereDomain<double> proj_hemi{};
    auto res_proj_hemi = core::integrate<double>(proj_hemi, [&n](const math::Vector<double, 3>& wi) {
        return 1.0f;
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res_proj_hemi << ", Expected: " << pi_v<double> << "\n";

    core::ParallelogramAreaDomain<double> para{
        math::Point<double, 3>(-1.0, 4.0, -1.0), 
        math::Vector<double, 3>(2.0, 0.0, 0.0), 
        math::Vector<double, 3>(0.0, 0.0, 2.0)
    };

    std::cout << "PDF: " << para.pdf() << ", Area: " << para.area() << "\n";

    auto shading_p = math::Point<double, 3>(0.0, 0.0, 0.0);
    auto shading_p_normal = math::Normal3(0.0, 1.0, 0.0);

    auto res_para = core::integrate<double>(para, [&shading_p, &shading_p_normal](const core::SurfaceInfo<double>& surface) {
        auto [p, normal] = surface;
        auto n = normal.to_vector();
        auto pn = shading_p_normal.to_vector();
        auto wi = (p - shading_p).normalized();
        auto cos_p = pn.dot(wi);
        auto cos_x = n.dot(-wi);
        auto r2 = (p - shading_p).length_squared();
        auto L = 1.0;
        return L * cos_p * cos_x / r2;
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res_para << ", Expected: 0.2308367977" << "\n";

    core::RGB<float> spectrum(1.0f, 0.5f, 0.25f);
    std::cout << "RGB Spectrum: " << spectrum << "\n";

    spectrum *= 2.0f;
    std::cout << "RGB Spectrum after multiplication: " << spectrum << "\n";

    core::SpectralRadiance<float> spec_rad(10.0f);
    std::cout << "Spectral Radiance: " << spec_rad << std::endl;
    core::WaveLength<float> delta_lambda(3.0f); 
    std::cout << "Delta Wavelength: " << delta_lambda << std::endl;

    auto ttt = spec_rad * delta_lambda;
    std::cout << "Spectral Radiance * Delta Wavelength: " << ttt << std::endl;

    core::BlackBodySpectrumDistribution<double> black_body_spectrum(3000.0);
    auto sampled_spectrum = black_body_spectrum.sample<4>(
        core::SampledSpectrum<double, 4>(
            math::Vector<double, 4>(400.0, 600.0, 800.0, 1000.0)
        ));
    std::cout << "Sampled Spectrum: " << sampled_spectrum / 1e12 << "\n";
    std::cout << "Max Value Wavelength: " << black_body_spectrum.max_wavelength() << "\n";
    return 0;
}