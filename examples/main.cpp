#include <array>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cmath>

#include "geometry/interaction.hpp"
#include "geometry/ray.hpp"
#include "math/normal.hpp"
#include "math/vector.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/swatch_reflectances_spectrum.hpp"
#include "integrator/domain.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "shape/shape.hpp"

#include "pbpt.h"
#include "shape/sphere.hpp"
#include "shape/triangle.hpp"
#include "scene/cornel_box_scene.hpp"

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

    math::Homo4 homo3 = math::Homo4::from_vector(math::Vec3{1.0, 2.0, 3.0});
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

    auto ho = math::Homo4::from_vector(math::Vec3(1.0, 2.0, 3.0));
    auto ho2 = math::Homo4::from_point(math::Pt3(4.0, 5.0, 6.0));
    std::cout << "Homo3 from Vector: " << ho << std::endl;
    std::cout << "Homo3 from Point: " << ho2 << std::endl;

    auto ho3 = ho + ho2;
    std::cout << "Homo3 Sum: " << ho3 << std::endl;
    std::cout << "Homo3 Vector: " << ho3.to_point() << std::endl;

    auto pp = math::Homo4::from_point(math::Pt3(1.0, 2.0, 3.0));
    std::cout << "Homo3 from Point: " << pp << std::endl;

    geometry::Trans tr = geometry::Trans::translate(math::Vec3(1.0, 2.0, 3.0));
    std::cout << "Translation Transform: " << tr.matrix() << std::endl;

    geometry::Frame<double> frame(math::Vec3(0.0, 1.0, 0.0));
    std::cout << "Frame t: " << frame.t() << std::endl;
    std::cout << "Frame b: " << frame.b() << std::endl;
    std::cout << "Frame n: " << frame.n() << std::endl;

    math::RandomGenerator<double, 2> rng2d(42);
    integrator::UniformHemisphereDomain<double> hemisphere{};

    math::Normal3 norm(0.0f, 0.0f, 1.0f);
    int sample_count = 10000;
    auto n = norm.to_vector();
    auto res = integrator::integrate<double>(hemisphere, [&n](const math::Vector<double, 3>& wi) {
        return 1.0 * n.dot(wi);
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res << ", Expected: " << pi_v<double> << "\n";

    integrator::UniformDiskDomain<double> disk{};
    auto res_disk = integrator::integrate<double>(disk, [](const math::Point<double, 2>& p) {
        return 1.0f;
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res_disk << ", Expected: " << pi_v<double> << "\n";

    integrator::CosineWeightedHemisphereDomain<double> proj_hemi{};
    auto res_proj_hemi = integrator::integrate<double>(proj_hemi, [&n](const math::Vector<double, 3>& wi) {
        return 1.0f;
    }, sample_count, rng2d);

    std::cout << "Estimated: " << res_proj_hemi << ", Expected: " << pi_v<double> << "\n";

    integrator::UniformParallelogramAreaDomain<double> para{
        math::Point<double, 3>(-1.0, 4.0, -1.0), 
        math::Vector<double, 3>(2.0, 0.0, 0.0), 
        math::Vector<double, 3>(0.0, 0.0, 2.0)
    };

    std::cout << "PDF: " << para.pdf(para.sample(rng2d)) << ", Area: " << para.area() << "\n";

    auto shading_p = math::Point<double, 3>(0.0, 0.0, 0.0);
    auto shading_p_normal = math::Normal3(0.0, 1.0, 0.0);

    auto res_para = integrator::integrate<double>(para, [&shading_p, &shading_p_normal](const integrator::SurfaceInfo<double>& surface) {
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

    radiometry::RGB<float> spectrum(1.0f, 0.5f, 0.25f);
    std::cout << "RGB Spectrum: " << spectrum << "\n";

    spectrum *= 2.0f;
    std::cout << "RGB Spectrum after multiplication: " << spectrum << "\n";

    radiometry::BlackBodySpectrumDistribution<double> black_body_spectrum(2856.0);
    auto sampled_spectrum = black_body_spectrum.sample<4>(
        radiometry::SampledSpectrum<double, 4>(
            math::Vector<double, 4>(400.0, 600.0, 800.0, 1000.0)
        ));
    std::cout << "Sampled Spectrum: " << sampled_spectrum / 1e12 << "\n";
    std::cout << "Max Value Wavelength: " << black_body_spectrum.max_wavelength() << "\n";


    auto cur_path = std::filesystem::current_path() / "asset" / "spectrum";

    std::cout << "Current Path: " << cur_path << std::endl;

    auto xyz_ =
        pbpt::radiometry::make_spectra_from_csv<double, radiometry::constant::XYZRange>((cur_path / "CIE_xyz_1931_2deg.csv").string(), 3);
    auto Xbar = xyz_[0];;
    auto Ybar = xyz_[1];;
    auto Zbar = xyz_[2];

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

    Xbar = radiometry::constant::CIE_X<double>;
    Ybar = radiometry::constant::CIE_Y<double>;
    Zbar = radiometry::constant::CIE_Z<double>;

    auto pppp = Ybar.sample<5>(radiometry::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 500, 600, 700, 800)));
    std::cout << pppp << std::endl;

    auto D50 = pbpt::radiometry::make_spectra_from_csv<double, radiometry::constant::luminantD50Range>((cur_path / "CIE_std_illum_D50.csv").string(), 1)[0];
    auto D65 = pbpt::radiometry::make_spectra_from_csv<double, radiometry::constant::luminantD65Range>((cur_path / "CIE_std_illum_D65.csv").string(), 1)[0];
    auto A = pbpt::radiometry::make_spectra_from_csv<double, radiometry::constant::luminantARange>((cur_path / "CIE_std_illum_A.csv").string(), 1)[0];

    // for (int i = radiometry::D65Range::LMinValue; i <= radiometry::D65Range::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", D65.at(i)) << " ";
    // }
    // std::cout << std::endl;

    D65 = radiometry::constant::CIE_D65_ilum<double>;

    // for (int i = radiometry::D50Range::LMinValue; i <= radiometry::D50Range::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", D50.at(i)) << " ";
    // }
    // std::cout << std::endl;

    D50 = radiometry::constant::CIE_D50_ilum<double>;

    // for (int i = radiometry::ARange::LMinValue; i <= radiometry::ARange::LMaxValue; ++i) {
    //     std::cout << std::format("{:.5f}, ", A.at(i)) << " ";
    // }
    // std::cout << std::endl;

    A = radiometry::constant::CIE_A_ilum<double>;

    auto d50_sampled = D50.sample<5>(radiometry::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto d65_sampled = D65.sample<5>(radiometry::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto a_sampled = A.sample<5>(radiometry::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));
    auto black_body_sampled = black_body_spectrum.sample<5>(radiometry::SampledWavelength<double, 5>(math::Vector<double, 5>(400, 560, 600, 700, 800)));

    std::cout << "D50 Sampled: " << d50_sampled << std::endl;
    std::cout << "D65 Sampled: " << d65_sampled << std::endl;
    std::cout << "A Sampled: " << a_sampled << std::endl;
    std::cout << "Black Body Sampled: " << black_body_sampled / 1e11 << std::endl;
    std::cout << black_body_sampled * a_sampled.inv() << std::endl;

    radiometry::XYZ<double> expected_xyz{};

    constexpr int sample_N = 10;
    constexpr int round_N = 1000;
    for (int i = 0; i < round_N; i ++) {
        math::RandomGenerator<double, sample_N> rng;
        auto wl_r = rng.generate_uniform(radiometry::constant::lambda_min<double>, radiometry::constant::lambda_max<double>);
        auto wl = radiometry::SampledWavelength<double, sample_N>(math::Vector<double, sample_N>::from_array(wl_r));

        auto xyz = radiometry::XYZ<double>::from_sampled_spectrum(
            D65.sample(wl),
            wl,
            radiometry::SampledPdf<double, sample_N>(math::Vector<double, sample_N>::filled(1 / (radiometry::constant::lambda_max<double> - radiometry::constant::lambda_min<double>)))
        );

        //std::cout << "XYZ from Sampled Spectrum: " << xyz * 100 / xyz.y() << std::endl;
        expected_xyz += xyz;
    }

    expected_xyz = expected_xyz / round_N;

    std::cout << "Expected XYZ: " << expected_xyz * 100 / expected_xyz.y() << std::endl;

    auto xyz_d65 = radiometry::XYZ<double>::from_illuminant(D65);
    //std::cout << "XYZ from D65 Spectrum: " << xyz_d65 * 100 / xyz_d65.y() << std::endl;
    std::cout << "XYZ from D65 Spectrum: " << xyz_d65 << std::endl;
    std::cout << "XYZ from D65 Spectrum (normalized to Y=100): " << xyz_d65.normalized_to_y(100.0) << std::endl;
    math::Point<double, 2> rp{0.64, 0.33};
    math::Point<double, 2> gp{0.3, 0.6};
    math::Point<double, 2> bp{0.15, 0.06};

    radiometry::XYZ<double> wp = radiometry::XYZ<double>::from_illuminant(D65);
    std::cout << "White Point from D65 Spectrum: " << wp << std::endl;
    radiometry::RGBColorSpace<double> sRGB(rp, gp, bp, wp);

    std::cout << "sRGB to XYZ Matrix:\n" << sRGB.rgb_to_xyz_matrix() << std::endl;

    auto white_xyz = sRGB.to_xyz(radiometry::RGB<double>(1.0, 1.0, 1.0));
    std::cout << "sRGB White Point to XYZ: " << white_xyz << std::endl;
    auto white_rgb = sRGB.to_rgb(white_xyz);
    std::cout << "XYZ White Point to sRGB: " << white_rgb << std::endl;
    std::cout << "sRGB White Point to LAB: " << radiometry::LAB<double>::from_xyz(white_xyz, sRGB.white_point()) << std::endl;
    std::cout << std::endl;

    auto gray_xyz = sRGB.to_xyz(radiometry::RGB<double>(0.214, 0.214, 0.214));
    std::cout << "sRGB Gray Point to XYZ: " << gray_xyz << std::endl;
    auto gray_rgb = sRGB.to_rgb(gray_xyz);
    std::cout << "XYZ Gray Point to sRGB: " << gray_rgb << std::endl;
    auto gray_lab = radiometry::LAB<double>::from_xyz(gray_xyz, sRGB.white_point());
    std::cout << "XYZ Gray Point to LAB: " << gray_lab << std::endl;
    std::cout << std::endl;

    auto red_xyz = sRGB.to_xyz(radiometry::RGB<double>(1.0, 0.0, 0.0));
    std::cout << "sRGB Red Point to XYZ: " << red_xyz << std::endl;
    auto red_rgb = sRGB.to_rgb(red_xyz);
    std::cout << "XYZ Red Point to sRGB: " << red_rgb << std::endl;
    auto red_lab = radiometry::LAB<double>::from_xyz(red_xyz, sRGB.white_point());
    std::cout << "XYZ Red Point to LAB: " << red_lab << std::endl;
    std::cout << std::endl;

    auto green_xyz = sRGB.to_xyz(radiometry::RGB<double>(0.0, 1.0, 0.0));
    std::cout << "sRGB Green Point to XYZ: " << green_xyz << std::endl;
    auto green_rgb = sRGB.to_rgb(green_xyz);
    std::cout << "XYZ Green Point to sRGB: " << green_rgb << std::endl;
    auto green_lab = radiometry::LAB<double>::from_xyz(green_xyz, sRGB.white_point());
    std::cout << "XYZ Green Point to LAB: " << green_lab << std::endl;
    std::cout << std::endl;

    auto blue_xyz = sRGB.to_xyz(radiometry::RGB<double>(0.0, 0.0, 1.0));
    std::cout << "sRGB Blue Point to XYZ: " << blue_xyz << std::endl;
    auto blue_rgb = sRGB.to_rgb(blue_xyz);
    std::cout << "XYZ Blue Point to sRGB: " << blue_rgb << std::endl;
    auto blue_lab = radiometry::LAB<double>::from_xyz(blue_xyz, sRGB.white_point());
    std::cout << "XYZ Blue Point to LAB: " << blue_lab << std::endl;
    std::cout << std::endl;

    auto rgb_ = radiometry::RGB<double>(0.133333, 1.0, 1.0);
    // radiometry::RGBAlbedoSpectrumDistribution<double, radiometry::RGBSigmoidPolynomial> albedo({
    //     -82.2253,   // C'  (constant)
    //     0.357441,  // B'  (linear)
    //     -0.000367656// A'  (quadratic)
    // });

    radiometry::RGBAlbedoSpectrumDistribution<double, radiometry::RGBSigmoidPolynomialNormalized> albedo({
        -1.14795, 43.0904, -80.4218
    });

    auto albedo_unnormalized = albedo.rsp().to_unnormalized();
    std::cout << "Albedo Coefficients (C0, C1, C2): " << albedo_unnormalized.c0 << ", " << albedo_unnormalized.c1 << ", " << albedo_unnormalized.c2 << std::endl;

    auto xyz_from_albedo = radiometry::XYZ<double>::from_reflectance(albedo, D65);
    std::cout << "XYZ from Albedo Spectrum: " << xyz_from_albedo << std::endl;
    auto rgb_from_albedo = radiometry::constant::sRGB<double>.to_rgb(xyz_from_albedo);
    std::cout << "sRGB from Albedo Spectrum: " << rgb_from_albedo << std::endl;

    auto xyz_d65_ = radiometry::XYZ<double>::from_illuminant(D65);
    std::cout << "XYZ from D65 Spectrum: " << xyz_d65_ << std::endl;
    std::cout << "XYZ from D65 Spectrum (normalized to Y=100): " << xyz_d65_.normalized_to_y(100.0) << std::endl;

    auto [error, coeffs] = radiometry::optimize_albedo_rgb_sigmoid_polynomial(radiometry::RGB<double>(0.139, 0.735, 0.989), radiometry::constant::sRGB<double>, D65);
    std::cout << "Optimization Error: " << error << std::endl;
    std::cout << "Optimized Albedo Coefficients (C0, C1, C2): " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << std::endl;

    auto optimized_sigmoid_polynomial_unnormalized = radiometry::RGBSigmoidPolynomialNormalized<double>{coeffs[0], coeffs[1], coeffs[2]}.to_unnormalized();
    std::cout << "Optimized Albedo Coefficients Unnormalized (C0, C1, C2): " << optimized_sigmoid_polynomial_unnormalized.c0 << ", " << optimized_sigmoid_polynomial_unnormalized.c1 << ", " << optimized_sigmoid_polynomial_unnormalized.c2 << std::endl;
    
    radiometry::RGBAlbedoSpectrumDistribution<double, radiometry::RGBSigmoidPolynomial> optimized_albedo(optimized_sigmoid_polynomial_unnormalized);
    auto xyz_from_optimized_albedo = radiometry::XYZ<double>::from_reflectance(optimized_albedo, D65);
    std::cout << "XYZ from Optimized Albedo Spectrum: " << xyz_from_optimized_albedo << std::endl;
    auto rgb_from_optimized_albedo = radiometry::constant::sRGB<double>.to_rgb(xyz_from_optimized_albedo);
    std::cout << "sRGB from Optimized Albedo Spectrum: " << rgb_from_optimized_albedo << std::endl;

    std::cout << "Target RGB: " << rgb_from_optimized_albedo << std::endl;
    radiometry::RGB<double> rgb_from_optimized_albedox2 = rgb_from_optimized_albedo * 2.0;
    std::cout << "sRGB from Optimized Albedo Spectrum x2: " << rgb_from_optimized_albedox2 << std::endl;
    auto [scaled_rgb, scale] = radiometry::scale_unbounded_rgb(rgb_from_optimized_albedox2);
    std::cout << "Scaled RGB: " << scaled_rgb << ", Scale: " << scale << std::endl;
    auto opti = radiometry::optimize_albedo_rgb_sigmoid_polynomial(scaled_rgb, radiometry::constant::sRGB<double>, D65);
    auto rspn = radiometry::RGBSigmoidPolynomialNormalized<double>{opti.normalized_coeffs[0], opti.normalized_coeffs[1], opti.normalized_coeffs[2]};
    radiometry::RGBUnboundedSpectrumDistribution<double, radiometry::RGBSigmoidPolynomialNormalized> unbounded_rgb_spectrum(rspn, scale);
    auto xyz_from_unbounded_rgb = radiometry::XYZ<double>::from_reflectance(unbounded_rgb_spectrum, D65);
    std::cout << "XYZ from Unbounded RGB Spectrum: " << xyz_from_unbounded_rgb << std::endl;
    auto rgb_from_unbounded_rgb = radiometry::constant::sRGB<double>.to_rgb(xyz_from_unbounded_rgb);
    std::cout << "sRGB from Unbounded RGB Spectrum: " << rgb_from_unbounded_rgb << std::endl;
    std::cout << "Target RGB: " << rgb_from_optimized_albedox2 << std::endl;

    radiometry::RGBIlluminantSpectrumDistribution<
        double, 
        radiometry::RGBSigmoidPolynomialNormalized, 
        radiometry::TabularSpectrumDistribution<
            double, 
            radiometry::constant::luminantD65Range::LMinValue, radiometry::constant::luminantD65Range::LMaxValue
        >
    > illuminant_spectrum(rspn, D65, scale);
    auto xyz_from_illuminant = radiometry::XYZ<double>::from_emission(illuminant_spectrum, D65);
    std::cout << "XYZ from Illuminant Spectrum: " << xyz_from_illuminant << std::endl;
    auto rgb_from_illuminant = radiometry::constant::sRGB<double>.to_rgb(xyz_from_illuminant);
    std::cout << "sRGB from Illuminant Spectrum: " << rgb_from_illuminant << std::endl;
    std::cout << "Target RGB: " << rgb_from_optimized_albedox2 << std::endl;


    camera::ThinLensPerspectiveCamera<double> camera(
        math::Vector<int, 2>(1000, 1000), 
        math::Vector<double, 2>(0.01, 0.01), 
        0.1, 100.0, 
        0.1, 1.0
    );

    for (int i = 0; i < 10; i ++) {
        for (int j = 0; j < 10; j ++) {
            auto r_ = camera.generate_ray(camera::CameraSample<double> {
                math::Point<double, 2>(i * 100 + 50, j * 100 + 50),
                math::Point<double, 2>(0.3, 0.3)
            });
            
            std::cout << "Center Camera Ray Origin: " << r_.origin() << ", Direction: " << r_.direction() << std::endl;
            
        }
    }

    radiometry::XYZ<double> d65_xyz = radiometry::XYZ<double>::from_illuminant(radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "D65 Illuminant XYZ: " << d65_xyz << std::endl;
    auto d65_rgb = radiometry::constant::sRGB<double>.to_rgb(d65_xyz);
    std::cout << "D65 Illuminant sRGB: " << d65_rgb << std::endl;
    auto d65_rgb_encoded = encode_to_nonlinear_srgb(d65_rgb);
    std::cout << "D65 Illuminant sRGB (gamma encoded): " << d65_rgb_encoded << std::endl;


    auto ref_red = radiometry::constant::get_swatch_reflectance<double>(radiometry::constant::SwatchReflectance::Red);
    radiometry::XYZ<double> xyz_red = radiometry::XYZ<double>::from_reflectance(ref_red, radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "Red Swatch Reflectance XYZ: " << xyz_red << std::endl;
    auto rgb_red = radiometry::constant::sRGB<double>.to_rgb(xyz_red);
    std::cout << "Red Swatch Reflectance sRGB: " << rgb_red << std::endl;
    auto rgb_red_encoded = encode_to_nonlinear_srgb(rgb_red);
    std::cout << "Red Swatch Reflectance sRGB (gamma encoded): " << rgb_red_encoded << std::endl;


    auto ref_blue = radiometry::constant::get_swatch_reflectance<double>(radiometry::constant::SwatchReflectance::Blue);
    radiometry::XYZ<double> xyz_blue = radiometry::XYZ<double>::from_reflectance(ref_blue, radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "Blue Swatch Reflectance XYZ: " << xyz_blue << std::endl;
    auto rgb_blue = radiometry::constant::sRGB<double>.to_rgb(xyz_blue);
    std::cout << "Blue Swatch Reflectance sRGB: " << rgb_blue << std::endl;
    auto rgb_blue_encoded = radiometry::encode_to_nonlinear_srgb(rgb_blue);
    std::cout << "Blue Swatch Reflectance sRGB (gamma encoded): " << rgb_blue_encoded << std::endl;


    auto ref_blue_ = radiometry::constant::get_swatch_reflectance<double>(radiometry::constant::SwatchReflectance::BlueSky);
    radiometry::XYZ<double> xyz_blue_ = radiometry::XYZ<double>::from_reflectance(ref_blue_, radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "Blue Sky Swatch Reflectance XYZ: " << xyz_blue_ << std::endl;
    auto rgb_blue_ = radiometry::constant::sRGB<double>.to_rgb(xyz_blue_);
    std::cout << "Blue Sky Swatch Reflectance sRGB: " << rgb_blue_ << std::endl;
    auto rgb_blue_sky_encoded = radiometry::encode_to_nonlinear_srgb(rgb_blue_);
    std::cout << "Blue Sky Swatch Reflectance sRGB (gamma encoded): " << rgb_blue_sky_encoded << std::endl;

    auto white_ref = radiometry::constant::get_swatch_reflectance<double>(radiometry::constant::SwatchReflectance::White);
    radiometry::XYZ<double> xyz_white = radiometry::XYZ<double>::from_reflectance(white_ref, radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "White Swatch Reflectance XYZ: " << xyz_white << std::endl;
    auto rgb_white = radiometry::constant::sRGB<double>.to_rgb(xyz_white);
    std::cout << "White Swatch Reflectance sRGB: " << rgb_white << std::endl;
    auto rgb_white_encoded = radiometry::encode_to_nonlinear_srgb(rgb_white);
    std::cout << "White Swatch Reflectance sRGB (gamma encoded): " << rgb_white_encoded << std::endl;

    auto black_ref = radiometry::constant::get_swatch_reflectance<double>(radiometry::constant::SwatchReflectance::Black);
    radiometry::XYZ<double> xyz_black = radiometry::XYZ<double>::from_reflectance(black_ref, radiometry::constant::CIE_D65_ilum<double>);
    std::cout << "Black Swatch Reflectance XYZ: " << xyz_black << std::endl;    
    auto rgb_black = radiometry::constant::sRGB<double>.to_rgb(xyz_black);
    std::cout << "Black Swatch Reflectance sRGB: " << rgb_black << std::endl;
    auto rgb_black_encoded = radiometry::encode_to_nonlinear_srgb(rgb_black);
    std::cout << "Black Swatch Reflectance sRGB (gamma encoded): " << rgb_black_encoded << std::endl;

    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.0, 2.0, 3.0})
    // );

    // std::cout << "Transformed Sphere Shape Bounding Box Min: " << sphere_shape.bounding_box().min() << ", Max: " << sphere_shape.bounding_box().max() << std::endl;
    // std::cout << "Transformed Sphere Shape Surface Area: " << sphere_shape.area() << std::endl;
    
    // geometry::Ray<double, 3> test_ray(math::Point<double, 3>{1.0, 2.0, 0.0}, math::Vector<double, 3>{0.0, 0.0, 1.0});
    // auto intersection = sphere_shape.intersect(test_ray);
    // if (intersection) {
    //     auto [intr, t] = intersection.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape1(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.2, 2.2, 3.0})
    // );

    // geometry::Ray<double, 3> test_ray1(math::Point<double, 3>{1.0, 2.0, 0.0}, math::Vector<double, 3>{0.0, 0.0, 1.0});
    // auto intersection1 = sphere_shape1.intersect(test_ray1);
    // if (intersection1) {
    //     auto [intr, t] = intersection1.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }


    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape_tangent(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.0, 2.0, 3.0})
    // );

    // geometry::Ray<double, 3> test_ray_tangent(math::Point<double, 3>{1.0, 1.0, 2.0}, math::Vector<double, 3>{0.0, 1.0, 0.0});
    // auto intersection_tangent = sphere_shape_tangent.intersect(test_ray_tangent);
    // if (intersection_tangent) {
    //     auto [intr, t] = intersection_tangent.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape_miss(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.0, 2.0, 3.0})
    // );

    // geometry::Ray<double, 3> test_ray_miss(math::Point<double, 3>{1.0, 1.0, 2.0}, math::Vector<double, 3>{1.0, 1.0, 0.0});
    // auto intersection_miss = sphere_shape_miss.intersect(test_ray_miss);
    // if (intersection_miss) {
    //     auto [intr, t] = intersection_miss.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape_in_sphere(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.0, 2.0, 3.0})
    // );

    // geometry::Ray<double, 3> test_ray_in_sphere(math::Point<double, 3>{1.0, 2.0, 3.0}, math::Vector<double, 3>{0.0, 1.0, 0.0});
    // auto intersection_in_sphere = sphere_shape_in_sphere.intersect(test_ray_in_sphere);
    // if (intersection_in_sphere) {
    //     auto [intr, t] = intersection_in_sphere.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // shape::TransformedShape<double, shape::Sphere<double>> sphere_shape_on_sphere(
    //     shape::Sphere(1.0),
    //     geometry::Transform<double>::translate(math::Vec3{1.0, 2.0, 3.0})
    // );

    // geometry::Ray<double, 3> test_ray_on_sphere(math::Point<double, 3>{1.0, 1.0, 3.0}, math::Vector<double, 3>{0.0, -1.0, 0.0});
    // std::cout << "Original Ray Origin: " << test_ray_on_sphere.origin() << std::endl;
    
    // auto intersection_on_sphere_before_offset = sphere_shape_on_sphere.intersect(test_ray_on_sphere);
    // if (intersection_on_sphere_before_offset) {
    //     auto [intr, t] = intersection_on_sphere_before_offset.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // auto origin = test_ray_on_sphere.origin();
    // math::Vector<double, 3> offset(1e-6, 1e-6, 1e-6);
    // auto offset_origin = geometry::offset_ray_origin(
    //     origin - offset, 
    //     origin + offset, 
    //     test_ray_on_sphere.direction(), 
    //     math::Normal<double, 3>(0.0, -1.0, 0.0)
    // );
    // test_ray_on_sphere.origin() = offset_origin;
    // std::cout << "Offset Ray Origin: " << test_ray_on_sphere.origin() << std::endl;
    // auto intersection_on_sphere = sphere_shape_on_sphere.intersect(test_ray_on_sphere);
    // if (intersection_on_sphere) {
    //     auto [intr, t] = intersection_on_sphere.value();
    //     std::cout << "Ray intersects Transformed Sphere Shape at t = " << t << ", Intersection Point: " << intr.point() << ", Normal: " << intr.n() << std::endl;
    // } else {
    //     std::cout << "Ray does not intersect Transformed Sphere Shape." << std::endl;
    // }

    // pbpt::light::AreaLight<double, pbpt::shape::Sphere<double>, radiometry::TabularSpectrumDistribution<double, pbpt::radiometry::constant::luminantD65Range::LMinValue, pbpt::radiometry::constant::luminantD65Range::LMaxValue>> area_light(
    //     sphere_shape,
    //     radiometry::constant::CIE_D65_ilum<double>
    // );



    return 0;
}
