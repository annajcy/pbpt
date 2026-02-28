#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "pbpt/material/plugin/bxdf/lambertian_bxdf.hpp"
#include "pbpt/material/plugin/bxdf/conductor_rough_bxdf.hpp"
#include "pbpt/material/plugin/bxdf/dielectric_rough_bxdf.hpp"
#include "pbpt/material/model.hpp"
#include "pbpt/material/optics.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/math/random_generator.hpp"
#include "pbpt/math/point.hpp"

using namespace pbpt;
using namespace pbpt::material;
using namespace pbpt::radiometry;
using namespace pbpt::math;

// Helper to generate random sames
template <typename T>
struct SampleData {
    std::vector<T> uc;
    std::vector<Point<T, 2>> u2d_i;
    std::vector<Point<T, 2>> u2d_o;
};

template <typename T>
SampleData<T> generate_samples(size_t count) {
    SampleData<T> data;
    data.uc.reserve(count);
    data.u2d_i.reserve(count);
    data.u2d_o.reserve(count);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<T> dist(0, 1);

    for (size_t i = 0; i < count; ++i) {
        data.uc.push_back(dist(rng));
        data.u2d_i.push_back(Point<T, 2>(dist(rng), dist(rng)));
        data.u2d_o.push_back(Point<T, 2>(dist(rng), dist(rng)));
    }
    return data;
}

TEST(BxDFTest, LambertianReflectance) {
    using Float = double;
    constexpr int N = 4;

    // 1. Setup Lambertian BxDF with albedo 0.5
    SampledSpectrum<Float, N> albedo = SampledSpectrum<Float, N>::filled(0.5);
    LambertianBxDF<Float, N> lambertian(albedo);

    // 2. Setup Wavelengths (doesn't matter for grey albedo)
    SampledWavelength<Float, N> swl;
    for (int i = 0; i < N; ++i)
        swl[i] = 500.0;

    // 3. Generate Samples
    const size_t sample_count = 100000;
    auto samples = generate_samples<Float>(sample_count);

    // 4. Test Hemispherical-Directional Reflectance (rou_hd)
    // For Lambertian, rho_hd should be equal to albedo regardless of wo (as long as wo is in hemisphere)
    Vector<Float, 3> wo(0, 0, 1);  // Normal incidence
    auto rho_hd = lambertian.rou_hd(swl, wo, samples.uc, samples.u2d_i);

    // Check if result is close to 0.5
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(rho_hd[i], 0.5, 1e-2) << "rho_hd should approach albedo for Lambertian";
    }

    // Test at grazing angle
    Vector<Float, 3> wo_grazing(0.99, 0, 0.1);
    wo_grazing = wo_grazing.normalized();
    auto rho_hd_grazing = lambertian.rou_hd(swl, wo_grazing, samples.uc, samples.u2d_i);
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(rho_hd_grazing[i], 0.5, 1e-2) << "rho_hd should be constant for Lambertian";
    }

    // 5. Test Hemispherical-Hemispherical Reflectance (rou_hh)
    auto rho_hh = lambertian.rou_hh(swl, samples.uc, samples.u2d_i, samples.u2d_o);

    // Check if result is close to 0.5
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(rho_hh[i], 0.5, 1e-2) << "rho_hh should approach albedo for Lambertian";
    }
}

TEST(BxDFTest, ConductorRoughWhiteFurnace) {
    using Float = double;
    constexpr int N = 4;

    // Setup:
    // 1. Perfect conductor approx (eta=1, k=infinity ideally, or just high reflectivity)
    // Conductor Fresnel: if k is very large, F -> 1.
    // Let's use eta=1, k=100.
    SampledSpectrum<Float, N> eta = SampledSpectrum<Float, N>::filled(1.0);
    SampledSpectrum<Float, N> k = SampledSpectrum<Float, N>::filled(100.0);

    // 2. High Roughness to exaggerate multiple scattering loss
    Float roughness = 0.5;

    MicrofacetModel<Float> microfacet(MicrofacetModel<Float>::roughness_to_alpha(roughness),
                                      MicrofacetModel<Float>::roughness_to_alpha(roughness),
                                      MicrofacetDistribution::GGX);

    ConductorRoughBxDF<Float, N> conductor(eta, k, microfacet);

    SampledWavelength<Float, N> swl;
    for (int i = 0; i < N; ++i)
        swl[i] = 500;

    auto samples = generate_samples<Float>(100000);

    // Test White Furnace (Hemispherical-Hemispherical Reflectance)
    // Energy conservation implies rho_hh <= 1.0.
    // Due to missing multiple scattering, we expect significant energy loss.

    auto rho_hh = conductor.rou_hh(swl, samples.uc, samples.u2d_i, samples.u2d_o);

    for (int i = 0; i < N; ++i) {
        EXPECT_LE(rho_hh[i], 1.0 + 1e-4) << "Energy conservation violated!";
        // We expect loss. For roughness 0.5, it should be noticeable.
        EXPECT_LT(rho_hh[i], 0.95) << "Rough conductor should exhibit energy loss due to single scattering assumption";
        std::cout << "[ConductorRough] Roughness " << roughness << " rho_hh: " << rho_hh[i] << std::endl;
    }

    // Lower roughness -> less energy loss
    constexpr Float smooth_roughness = 0.01;
    MicrofacetModel<Float> smooth_mf(MicrofacetModel<Float>::roughness_to_alpha(smooth_roughness),
                                     MicrofacetModel<Float>::roughness_to_alpha(smooth_roughness),
                                     MicrofacetDistribution::GGX);
    ConductorRoughBxDF<Float, N> smooth_conductor(eta, k, smooth_mf);

    auto rho_hh_smooth = smooth_conductor.rou_hh(swl, samples.uc, samples.u2d_i, samples.u2d_o);
    for (int i = 0; i < N; ++i) {
        EXPECT_GT(rho_hh_smooth[i], rho_hh[i]) << "Smoother surface should lose less energy";
        EXPECT_NEAR(rho_hh_smooth[i], 1.0, 0.05) << "Smooth surface should be close to 1.0";
        std::cout << "[ConductorRough] Roughness " << smooth_roughness << " rho_hh: " << rho_hh_smooth[i] << std::endl;
    }
}

TEST(BxDFTest, DielectricRoughWhiteFurnace) {
    using Float = double;
    constexpr int N = 4;

    // Setup:
    // 1. Dielectric with eta=1.5
    Float eta_val = 1.5;

    // 2. Roughness
    Float roughness = 0.5;
    MicrofacetModel<Float> microfacet(MicrofacetModel<Float>::roughness_to_alpha(roughness),
                                      MicrofacetModel<Float>::roughness_to_alpha(roughness),
                                      MicrofacetDistribution::GGX);

    // DielectricRoughBxDF(eta, microfacet, type=Refl|Trans)
    // If we test reflection + transmission, the sum should be < 1.0 due to MS loss.
    // Usually DielectricRoughBxDF handles both if sampled/evaluated correctly,
    // but often it's constructed as separate lobes or a combined lobe.
    // Let's check the constructor.
    // Based on filenames, it's `dielectric_rough_bxdf.hpp`.
    // Assuming standard construction: DielectricRoughBxDF(eta, distrib)

    DielectricRoughBxDF<Float, N> dielectric(eta_val, microfacet);

    SampledWavelength<Float, N> swl;
    for (int i = 0; i < N; ++i)
        swl[i] = 500;

    auto samples = generate_samples<Float>(100000);

    // Test White Furnace for Dielectric
    // Ideally, for a non-absorbing dielectric, R + T = 1.
    // With single scattering only, R + T < 1 for rough surfaces.

    auto rho_hh = dielectric.rou_hh(swl, samples.uc, samples.u2d_i, samples.u2d_o);

    for (int i = 0; i < N; ++i) {
        EXPECT_LE(rho_hh[i], 1.0 + 1e-4) << "Energy conservation violated!";
        EXPECT_LT(rho_hh[i], 0.95) << "Rough dielectric should exhibit energy loss due to single scattering assumption";
        std::cout << "[DielectricRough] Roughness " << roughness << " rho_hh: " << rho_hh[i] << std::endl;
    }
}

TEST(BxDFTest, MakeRayDifferentialOffsetNonSpecularReturnsNullopt) {
    using Float = double;

    geometry::SurfaceInteraction<Float> si(Point<Float, 3>(0, 0, 0), Vector<Float, 3>(0, 0, 1),
                                           Normal<Float, 3>(0, 0, 1), Point<Float, 2>(0, 0), Vector<Float, 3>(1, 0, 0),
                                           Vector<Float, 3>(0, 1, 0));
    geometry::ShadingInfo<Float> shading{Normal<Float, 3>(0, 0, 1), Normal<Float, 3>(0, 0, 0),
                                         Normal<Float, 3>(0, 0, 0)};
    geometry::SurfaceDifferentials<Float> surface_diffs;
    surface_diffs.dpdx = Vector<Float, 3>(0.01, 0.0, 0.0);
    surface_diffs.dpdy = Vector<Float, 3>(0.0, 0.01, 0.0);
    surface_diffs.dudx = 0.1;
    surface_diffs.dvdy = 0.1;

    geometry::Ray<Float, 3> main_ray(Point<Float, 3>(0, 0, 1), Vector<Float, 3>(0, 0, -1));
    std::array<geometry::Ray<Float, 3>, 2> diff_rays{
        geometry::Ray<Float, 3>(Point<Float, 3>(0.01, 0.0, 1), Vector<Float, 3>(0, 0, -1)),
        geometry::Ray<Float, 3>(Point<Float, 3>(0.0, 0.01, 1), Vector<Float, 3>(0, 0, -1))};
    geometry::RayDifferential<Float, 3> ray_diff(main_ray, diff_rays);

    auto offset = make_ray_differential_offset(BxDFFlags::DiffuseReflection, ray_diff, si, shading, surface_diffs,
                                               Vector<Float, 3>(0, 0, 1), Float(1.0));
    EXPECT_FALSE(offset.has_value());
}

TEST(BxDFTest, MakeRayDifferentialOffsetSpecularReflectionIsFinite) {
    using Float = double;

    geometry::SurfaceInteraction<Float> si(Point<Float, 3>(0, 0, 0), Vector<Float, 3>(0, 0, 1),
                                           Normal<Float, 3>(0, 0, 1), Point<Float, 2>(0, 0), Vector<Float, 3>(1, 0, 0),
                                           Vector<Float, 3>(0, 1, 0));
    geometry::ShadingInfo<Float> shading{Normal<Float, 3>(0, 0, 1), Normal<Float, 3>(0.01, 0.0, 0.0),
                                         Normal<Float, 3>(0.0, 0.01, 0.0)};
    geometry::SurfaceDifferentials<Float> surface_diffs;
    surface_diffs.dpdx = Vector<Float, 3>(0.02, 0.0, 0.0);
    surface_diffs.dpdy = Vector<Float, 3>(0.0, 0.02, 0.0);
    surface_diffs.dudx = 0.05;
    surface_diffs.dvdx = 0.01;
    surface_diffs.dudy = 0.02;
    surface_diffs.dvdy = 0.04;

    geometry::Ray<Float, 3> main_ray(Point<Float, 3>(0, 0, 1), Vector<Float, 3>(0, 0, -1));
    std::array<geometry::Ray<Float, 3>, 2> diff_rays{
        geometry::Ray<Float, 3>(Point<Float, 3>(0.01, 0.0, 1), Vector<Float, 3>(0.001, 0, -1)),
        geometry::Ray<Float, 3>(Point<Float, 3>(0.0, 0.01, 1), Vector<Float, 3>(0, 0.001, -1))};
    geometry::RayDifferential<Float, 3> ray_diff(main_ray, diff_rays);

    auto offset = make_ray_differential_offset(BxDFFlags::SpecularReflection, ray_diff, si, shading, surface_diffs,
                                               Vector<Float, 3>(0, 0, 1), Float(1.0));
    ASSERT_TRUE(offset.has_value());
    EXPECT_NEAR(offset->dpdx.x(), surface_diffs.dpdx.x(), 1e-12);
    EXPECT_NEAR(offset->dpdy.y(), surface_diffs.dpdy.y(), 1e-12);
    EXPECT_TRUE(std::isfinite(offset->dwdx.x()));
    EXPECT_TRUE(std::isfinite(offset->dwdx.y()));
    EXPECT_TRUE(std::isfinite(offset->dwdx.z()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.x()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.y()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.z()));
}

TEST(BxDFTest, MakeRayDifferentialOffsetSpecularTransmissionIsFinite) {
    using Float = double;

    geometry::SurfaceInteraction<Float> si(Point<Float, 3>(0, 0, 0), Vector<Float, 3>(0, 0, 1),
                                           Normal<Float, 3>(0, 0, 1), Point<Float, 2>(0, 0), Vector<Float, 3>(1, 0, 0),
                                           Vector<Float, 3>(0, 1, 0));
    geometry::ShadingInfo<Float> shading{Normal<Float, 3>(0, 0, 1), Normal<Float, 3>(0.01, 0.0, 0.0),
                                         Normal<Float, 3>(0.0, 0.01, 0.0)};
    geometry::SurfaceDifferentials<Float> surface_diffs;
    surface_diffs.dpdx = Vector<Float, 3>(0.02, 0.0, 0.0);
    surface_diffs.dpdy = Vector<Float, 3>(0.0, 0.02, 0.0);
    surface_diffs.dudx = 0.05;
    surface_diffs.dvdx = 0.01;
    surface_diffs.dudy = 0.02;
    surface_diffs.dvdy = 0.04;

    geometry::Ray<Float, 3> main_ray(Point<Float, 3>(0, 0, 1), Vector<Float, 3>(0, 0, -1));
    std::array<geometry::Ray<Float, 3>, 2> diff_rays{
        geometry::Ray<Float, 3>(Point<Float, 3>(0.01, 0.0, 1), Vector<Float, 3>(0.001, 0, -1)),
        geometry::Ray<Float, 3>(Point<Float, 3>(0.0, 0.01, 1), Vector<Float, 3>(0, 0.001, -1))};
    geometry::RayDifferential<Float, 3> ray_diff(main_ray, diff_rays);

    auto refract_result = refract(Vector<Float, 3>(0, 0, 1), Vector<Float, 3>(0, 0, 1), Float(1.5));
    ASSERT_TRUE(refract_result.has_value());

    auto offset = make_ray_differential_offset(BxDFFlags::SpecularTransmission, ray_diff, si, shading, surface_diffs,
                                               refract_result->wt, refract_result->etap);
    ASSERT_TRUE(offset.has_value());
    EXPECT_TRUE(std::isfinite(offset->dwdx.x()));
    EXPECT_TRUE(std::isfinite(offset->dwdx.y()));
    EXPECT_TRUE(std::isfinite(offset->dwdx.z()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.x()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.y()));
    EXPECT_TRUE(std::isfinite(offset->dwdy.z()));
}
