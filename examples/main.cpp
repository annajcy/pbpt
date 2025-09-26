#include <array>
#include <cmath>
#include <cstdlib>
#include <format>
#include <iostream>
#include <filesystem>

#include "core/interaction.hpp"
#include "core/radiometric_integrals.hpp"
#include "core/radiometry.hpp"
#include "core/shape.hpp"
#include "core/spectrum.hpp"
#include "core/color.hpp"
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
#include "geometry/ray.hpp"
#include "geometry/spherical.hpp"
#include "geometry/transform.hpp"
#include "math/format.hpp"
#include "math/vector.hpp"
#include "math/function.hpp"
#include "geometry/directional_cone.hpp"
#include "math/type_alias.hpp"
#include "utils/spectrum_loader.hpp"

#include "core/camera.hpp"

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

    core::BlackBodySpectrumDistribution<double> black_body_spectrum(2856.0);
    auto sampled_spectrum = black_body_spectrum.sample<4>(
        core::SampledSpectrum<double, 4>(
            math::Vector<double, 4>(400.0, 600.0, 800.0, 1000.0)
        ));
    std::cout << "Sampled Spectrum: " << sampled_spectrum / 1e12 << "\n";
    std::cout << "Max Value Wavelength: " << black_body_spectrum.max_wavelength() << "\n";


    auto cur_path = std::filesystem::current_path().parent_path().parent_path() / "asset" / "spectrum";

    std::cout << "Current Path: " << cur_path << std::endl;

    auto arr =
        pbpt::utils::make_spectra_from_csv<double, 3, core::XYZRange>((cur_path / "CIE_xyz_1931_2deg.csv").string());
    auto [Xbar, Ybar, Zbar] = arr;

    // std::cout << "std::array<T, lambda_max<T> - lambda_min<T> + 1>{\n";
    // for (int i = pbpt::utils::XYZRange::LMinValue; i <= pbpt::utils::XYZRange::LMaxValue; ++i) {
    //     std::cout << std::format("{:.12f}", Xbar.at(i)) << ", ";
    // }
    // std::cout << "\n}\n";

    // std::cout << "std::array<T, lambda_max<T> - lambda_min<T> + 1>{\n";
    // for (int i = pbpt::utils::XYZRange::LMinValue; i <= pbpt::utils::XYZRange::LMaxValue; ++i) {
    //     std::cout << std::format("{:.12f}", Ybar.at(i)) << ", ";
    // }
    // std::cout << "\n}\n";

    // std::cout << "std::array<T, lambda_max<T> - lambda_min<T> + 1>{\n";
    // for (int i = pbpt::utils::XYZRange::LMinValue; i <= pbpt::utils::XYZRange::LMaxValue; ++i) {
    //     std::cout << std::format("{:.12f}", Zbar.at(i)) << ", ";
    // }
    // std::cout << "\n}\n";

    Xbar = core::CIE_X<double>;
    Ybar = core::CIE_Y<double>;
    Zbar = core::CIE_Z<double>;

    auto pppp = Ybar.sample<5>(core::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 500, 600, 700, 800)));
    std::cout << pppp << std::endl;

    auto [D50] = pbpt::utils::make_spectra_from_csv<double, 1, core::luminantD50Range>((cur_path / "CIE_std_illum_D50.csv").string());
    auto [D65] = pbpt::utils::make_spectra_from_csv<double, 1, core::luminantD65Range>((cur_path / "CIE_std_illum_D65.csv").string());
    auto [A] = pbpt::utils::make_spectra_from_csv<double, 1, core::luminantARange>((cur_path / "CIE_std_illum_A.csv").string());

    // for (int i = core::D65Range::LMinValue; i <= core::D65Range::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", D65.at(i)) << " ";
    // }
    // std::cout << std::endl;

    D65 = core::CIE_D65_ilum<double>;

    // for (int i = core::D50Range::LMinValue; i <= core::D50Range::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", D50.at(i)) << " ";
    // }
    // std::cout << std::endl;

    D50 = core::CIE_D50_ilum<double>;

    // for (int i = core::ARange::LMinValue; i <= core::ARange::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", A.at(i)) << " ";
    // }
    // std::cout << std::endl;

    A = core::CIE_A_ilum<double>;

    auto d50_sampled = D50.sample<5>(core::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto d65_sampled = D65.sample<5>(core::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto a_sampled = A.sample<5>(core::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto black_body_sampled = black_body_spectrum.sample<5>(core::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));

    std::cout << "D50 Sampled: " << d50_sampled << std::endl;
    std::cout << "D65 Sampled: " << d65_sampled << std::endl;
    std::cout << "A Sampled: " << a_sampled << std::endl;
    std::cout << "Black Body Sampled: " << black_body_sampled / 1e11 << std::endl;
    std::cout << black_body_sampled * a_sampled.inv() << std::endl;

    core::XYZ<double> expected_xyz{};

    constexpr int sample_N = 10;
    constexpr int round_N = 1000;
    for (int i = 0; i < round_N; i ++) {
        math::RandomGenerator<double, sample_N> rng;
        auto wl_r = rng.generate_uniform(core::lambda_min<double>, core::lambda_max<double>);
        auto wl = core::SampledWavelength<double, sample_N>(math::Vector<double, sample_N>::from_array(wl_r));

        auto xyz = core::XYZ<double>::from_sampled_spectrum(
            D65.sample(wl),
            wl,
            core::SampledPdf<double, sample_N>(math::Vector<double, sample_N>::filled(1 / (core::lambda_max<double> - core::lambda_min<double>)))
        );

        //std::cout << "XYZ from Sampled Spectrum: " << xyz * 100 / xyz.y() << std::endl;
        expected_xyz += xyz;
    }

    expected_xyz = expected_xyz / round_N;

    std::cout << "Expected XYZ: " << expected_xyz * 100 / expected_xyz.y() << std::endl;

    auto xyz_d65 = core::XYZ<double>::from_standard_illuminant(D65);
    //std::cout << "XYZ from D65 Spectrum: " << xyz_d65 * 100 / xyz_d65.y() << std::endl;
    std::cout << "XYZ from D65 Spectrum: " << xyz_d65 << std::endl;
    std::cout << "XYZ from D65 Spectrum (normalized to Y=100): " << xyz_d65.normalized_to_y(100.0) << std::endl;
    math::Point<double, 2> rp{0.64, 0.33};
    math::Point<double, 2> gp{0.3, 0.6};
    math::Point<double, 2> bp{0.15, 0.06};

    core::XYZ<double> wp = core::XYZ<double>::from_standard_illuminant(D65);
    std::cout << "White Point from D65 Spectrum: " << wp << std::endl;
    core::RGBColorSpace<double> sRGB(rp, gp, bp, wp);

    std::cout << "sRGB to XYZ Matrix:\n" << sRGB.rgb_to_xyz_matrix() << std::endl;

    auto white_xyz = sRGB.to_xyz(core::RGB<double>(1.0, 1.0, 1.0));
    std::cout << "sRGB White Point to XYZ: " << white_xyz << std::endl;
    auto white_rgb = sRGB.to_rgb(white_xyz);
    std::cout << "XYZ White Point to sRGB: " << white_rgb << std::endl;
    std::cout << "sRGB White Point to LAB: " << core::LAB<double>::from_xyz(white_xyz, sRGB.white_point()) << std::endl;
    std::cout << std::endl;

    auto gray_xyz = sRGB.to_xyz(core::RGB<double>(0.214, 0.214, 0.214));
    std::cout << "sRGB Gray Point to XYZ: " << gray_xyz << std::endl;
    auto gray_rgb = sRGB.to_rgb(gray_xyz);
    std::cout << "XYZ Gray Point to sRGB: " << gray_rgb << std::endl;
    auto gray_lab = core::LAB<double>::from_xyz(gray_xyz, sRGB.white_point());
    std::cout << "XYZ Gray Point to LAB: " << gray_lab << std::endl;
    std::cout << std::endl;

    auto red_xyz = sRGB.to_xyz(core::RGB<double>(1.0, 0.0, 0.0));
    std::cout << "sRGB Red Point to XYZ: " << red_xyz << std::endl;
    auto red_rgb = sRGB.to_rgb(red_xyz);
    std::cout << "XYZ Red Point to sRGB: " << red_rgb << std::endl;
    auto red_lab = core::LAB<double>::from_xyz(red_xyz, sRGB.white_point());
    std::cout << "XYZ Red Point to LAB: " << red_lab << std::endl;
    std::cout << std::endl;

    auto green_xyz = sRGB.to_xyz(core::RGB<double>(0.0, 1.0, 0.0));
    std::cout << "sRGB Green Point to XYZ: " << green_xyz << std::endl;
    auto green_rgb = sRGB.to_rgb(green_xyz);
    std::cout << "XYZ Green Point to sRGB: " << green_rgb << std::endl;
    auto green_lab = core::LAB<double>::from_xyz(green_xyz, sRGB.white_point());
    std::cout << "XYZ Green Point to LAB: " << green_lab << std::endl;
    std::cout << std::endl;

    auto blue_xyz = sRGB.to_xyz(core::RGB<double>(0.0, 0.0, 1.0));
    std::cout << "sRGB Blue Point to XYZ: " << blue_xyz << std::endl;
    auto blue_rgb = sRGB.to_rgb(blue_xyz);
    std::cout << "XYZ Blue Point to sRGB: " << blue_rgb << std::endl;
    auto blue_lab = core::LAB<double>::from_xyz(blue_xyz, sRGB.white_point());
    std::cout << "XYZ Blue Point to LAB: " << blue_lab << std::endl;
    std::cout << std::endl;

    auto rgb_ = core::RGB<double>(0.133333, 1.0, 1.0);
    // core::RGBAlbedoSpectrumDistribution<double, core::RGBSigmoidPolynomial> albedo({
    //     -82.2253,   // C'  (constant)
    //     0.357441,  // B'  (linear)
    //     -0.000367656// A'  (quadratic)
    // });

    core::RGBAlbedoSpectrumDistribution<double, core::RGBSigmoidPolynomialNormalized> albedo({
        -1.14795, 43.0904, -80.4218
    });

    auto albedo_unnormalized = albedo.rsp().to_unnormalized();
    std::cout << "Albedo Coefficients (C0, C1, C2): " << albedo_unnormalized.c0 << ", " << albedo_unnormalized.c1 << ", " << albedo_unnormalized.c2 << std::endl;

    auto xyz_from_albedo = core::XYZ<double>::from_reflectance_under_illuminant(albedo, D65);
    std::cout << "XYZ from Albedo Spectrum: " << xyz_from_albedo << std::endl;
    auto rgb_from_albedo = core::sRGB<double>.to_rgb(xyz_from_albedo);
    std::cout << "sRGB from Albedo Spectrum: " << rgb_from_albedo << std::endl;

    auto xyz_d65_ = core::XYZ<double>::from_standard_illuminant(D65);
    std::cout << "XYZ from D65 Spectrum: " << xyz_d65_ << std::endl;
    std::cout << "XYZ from D65 Spectrum (normalized to Y=100): " << xyz_d65_.normalized_to_y(100.0) << std::endl;

    auto [error, coeffs] = core::optimize_albedo_rgb_sigmoid_polynomial(core::RGB<double>(0.139, 0.735, 0.989), core::sRGB<double>, D65);
    std::cout << "Optimization Error: " << error << std::endl;
    std::cout << "Optimized Albedo Coefficients (C0, C1, C2): " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << std::endl;

    auto optimized_sigmoid_polynomial_unnormalized = core::RGBSigmoidPolynomialNormalized<double>{coeffs[0], coeffs[1], coeffs[2]}.to_unnormalized();
    std::cout << "Optimized Albedo Coefficients Unnormalized (C0, C1, C2): " << optimized_sigmoid_polynomial_unnormalized.c0 << ", " << optimized_sigmoid_polynomial_unnormalized.c1 << ", " << optimized_sigmoid_polynomial_unnormalized.c2 << std::endl;
    
    core::RGBAlbedoSpectrumDistribution<double, core::RGBSigmoidPolynomial> optimized_albedo(optimized_sigmoid_polynomial_unnormalized);
    auto xyz_from_optimized_albedo = core::XYZ<double>::from_reflectance_under_illuminant(optimized_albedo, D65);
    std::cout << "XYZ from Optimized Albedo Spectrum: " << xyz_from_optimized_albedo << std::endl;
    auto rgb_from_optimized_albedo = core::sRGB<double>.to_rgb(xyz_from_optimized_albedo);
    std::cout << "sRGB from Optimized Albedo Spectrum: " << rgb_from_optimized_albedo << std::endl;

    std::cout << "Target RGB: " << rgb_from_optimized_albedo << std::endl;
    core::RGB<double> rgb_from_optimized_albedox2 = rgb_from_optimized_albedo * 2.0;
    std::cout << "sRGB from Optimized Albedo Spectrum x2: " << rgb_from_optimized_albedox2 << std::endl;
    auto [scaled_rgb, scale] = core::scale_unbounded_rgb(rgb_from_optimized_albedox2);
    std::cout << "Scaled RGB: " << scaled_rgb << ", Scale: " << scale << std::endl;
    auto opti = core::optimize_albedo_rgb_sigmoid_polynomial(scaled_rgb, core::sRGB<double>, D65);
    auto rspn = core::RGBSigmoidPolynomialNormalized<double>{opti.coeffs[0], opti.coeffs[1], opti.coeffs[2]};
    core::RGBUnboundedSpectrumDistribution<double, core::RGBSigmoidPolynomialNormalized> unbounded_rgb_spectrum(rspn, scale);
    auto xyz_from_unbounded_rgb = core::XYZ<double>::from_reflectance_under_illuminant(unbounded_rgb_spectrum, D65);
    std::cout << "XYZ from Unbounded RGB Spectrum: " << xyz_from_unbounded_rgb << std::endl;
    auto rgb_from_unbounded_rgb = core::sRGB<double>.to_rgb(xyz_from_unbounded_rgb);
    std::cout << "sRGB from Unbounded RGB Spectrum: " << rgb_from_unbounded_rgb << std::endl;
    std::cout << "Target RGB: " << rgb_from_optimized_albedox2 << std::endl;

    core::RGBIlluminantSpectrumDistribution<
        double, 
        core::RGBSigmoidPolynomialNormalized, 
        core::TabularSpectrumDistribution<
            double, 
            core::luminantD65Range::LMinValue, core::luminantD65Range::LMaxValue
        >
    > illuminant_spectrum(rspn, D65, scale);
    auto xyz_from_illuminant = core::XYZ<double>::from_emission_under_illuminant(illuminant_spectrum, D65);
    std::cout << "XYZ from Illuminant Spectrum: " << xyz_from_illuminant << std::endl;
    auto rgb_from_illuminant = core::sRGB<double>.to_rgb(xyz_from_illuminant);
    std::cout << "sRGB from Illuminant Spectrum: " << rgb_from_illuminant << std::endl;
    std::cout << "Target RGB: " << rgb_from_optimized_albedox2 << std::endl;
    return 0;
}