#include <gtest/gtest.h>

#include "geometry/interaction.hpp"
#include "light/light.hpp"
#include "math/function.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "shape/sphere.hpp"

namespace pbpt::light::testing {

namespace {

template <typename T>
geometry::NormalInteraction<T> make_ref_interaction(const math::Point<T, 3>& p) {
    return geometry::NormalInteraction<T>(
        p,
        math::Vector<T, 3>(0, 0, 1),
        math::Normal<T, 3>(0, 0, 1),
        math::Vector<T, 3>(0, 0, 0)
    );
}

struct NeverOccluded {
    template <typename Ray>
    bool is_intersected(const Ray&) const {
        return false;
    }
};

struct AlwaysOccluded {
    template <typename Ray>
    bool is_intersected(const Ray&) const {
        return true;
    }
};

template <typename T, int N>
void expect_constant_spectrum(const radiometry::SampledSpectrum<T, N>& s, T expected) {
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(s[i], expected, T(1e-6));
    }
}

}  // namespace

TEST(VisibilityTesterTest, UnoccludedDependsOnAggregateHit) {
    using T = float;

    auto ref = make_ref_interaction<T>(math::Point<T, 3>(0, 0, 0));
    math::Point<T, 3> dst(0, 0, 1);

    VisibilityTester<T, geometry::NormalInteraction<T>> tester(ref, dst);
    EXPECT_TRUE(tester.is_unoccluded(NeverOccluded{}));
    EXPECT_FALSE(tester.is_unoccluded(AlwaysOccluded{}));
    EXPECT_FLOAT_EQ(tester.dst_point().z(), 1.0f);
    EXPECT_FLOAT_EQ(tester.src_interaction().point().z(), 0.0f);
}

TEST(AreaLightTest, SampleLightReturnsNulloptWhenBackfacing) {
    using T = float;
    constexpr int N = 4;

    shape::Sphere<T> sphere(1);
    shape::TransformedShape<T, shape::Sphere<T>> transformed_sphere(
        sphere,
        geometry::Transform<T>::identity()
    );

    radiometry::ConstantSpectrumDistribution<T> power(T(3));
    AreaLight<T, shape::Sphere<T>, radiometry::ConstantSpectrumDistribution<T>> light(
        transformed_sphere,
        power
    );

    auto wavelengths = radiometry::sample_uniform_wavelengths_stratified<T, N>(T(0.25));
    auto ref = make_ref_interaction<T>(math::Point<T, 3>(0, 0, 3));

    // u.x() == 0 => z == -1 (south pole), which backfaces a reference point at +z.
    auto sample_opt = light.sample_light<N>(wavelengths, ref, math::Point<T, 2>(0, 0));
    EXPECT_FALSE(sample_opt.has_value());
}

TEST(AreaLightTest, SampleLightAndPdfMatchExpectedForNorthPoleSample) {
    using T = float;
    constexpr int N = 4;

    shape::Sphere<T> sphere(1);
    shape::TransformedShape<T, shape::Sphere<T>> transformed_sphere(
        sphere,
        geometry::Transform<T>::identity()
    );

    radiometry::ConstantSpectrumDistribution<T> power(T(5));
    AreaLight<T, shape::Sphere<T>, radiometry::ConstantSpectrumDistribution<T>> light(
        transformed_sphere,
        power
    );

    auto wavelengths = radiometry::sample_uniform_wavelengths_stratified<T, N>(T(0.25));
    auto ref = make_ref_interaction<T>(math::Point<T, 3>(0, 0, 3));

    // u.x() == 1 => z == +1 (north pole), fully facing the reference point at +z.
    auto sample_opt = light.sample_light<N>(wavelengths, ref, math::Point<T, 2>(1, 0));
    ASSERT_TRUE(sample_opt.has_value());
    const auto& sample = sample_opt.value();

    expect_constant_spectrum(sample.radiance, T(5));

    // light point: (0, 0, 1), ref point: (0, 0, 3) => wi = (0, 0, -1)
    EXPECT_NEAR(sample.wi.x(), T(0), T(1e-6));
    EXPECT_NEAR(sample.wi.y(), T(0), T(1e-6));
    EXPECT_NEAR(sample.wi.z(), T(-1), T(1e-6));

    // For this configuration:
    // - area = 4*pi, pdf_area = 1/(4*pi)
    // - distance^2 = 4
    // - cos_theta_light = 1
    // => pdf_solid_angle = (1/(4*pi)) * 4 = 1/pi
    T expected_pdf = T(1) / math::pi_v<T>;
    EXPECT_NEAR(sample.pdf, expected_pdf, T(1e-5));

    T pdf_eval = light.sample_light_pdf(ref, sample.wi);
    EXPECT_NEAR(pdf_eval, expected_pdf, T(1e-5));

    auto emission = light.emission_spectrum<N>(
        wavelengths,
        sample.visibility_tester.dst_point(),
        -sample.wi
    );
    expect_constant_spectrum(emission, T(5));
    EXPECT_FALSE(light.is_delta_light());
}

TEST(AreaLightTest, VisibilityTesterReferencesSampledPointAndRespectsOcclusion) {
    using T = float;
    constexpr int N = 4;

    shape::Sphere<T> sphere(1);
    shape::TransformedShape<T, shape::Sphere<T>> transformed_sphere(
        sphere,
        geometry::Transform<T>::identity()
    );

    radiometry::ConstantSpectrumDistribution<T> power(T(1));
    AreaLight<T, shape::Sphere<T>, radiometry::ConstantSpectrumDistribution<T>> light(
        transformed_sphere,
        power
    );

    auto wavelengths = radiometry::sample_uniform_wavelengths_stratified<T, N>(T(0.25));
    auto ref = make_ref_interaction<T>(math::Point<T, 3>(0, 0, 3));

    auto sample_opt = light.sample_light<N>(wavelengths, ref, math::Point<T, 2>(1, 0));
    ASSERT_TRUE(sample_opt.has_value());
    const auto& sample = sample_opt.value();

    const auto& tester = sample.visibility_tester;
    EXPECT_NEAR(tester.src_interaction().point().x(), ref.point().x(), T(1e-6));
    EXPECT_NEAR(tester.src_interaction().point().y(), ref.point().y(), T(1e-6));
    EXPECT_NEAR(tester.src_interaction().point().z(), ref.point().z(), T(1e-6));

    // For a unit sphere with u=(1,0), the sampled point is the north pole.
    EXPECT_NEAR(tester.dst_point().x(), T(0), T(1e-6));
    EXPECT_NEAR(tester.dst_point().y(), T(0), T(1e-6));
    EXPECT_NEAR(tester.dst_point().z(), T(1), T(1e-6));

    EXPECT_TRUE(tester.is_unoccluded(NeverOccluded{}));
    EXPECT_FALSE(tester.is_unoccluded(AlwaysOccluded{}));
}

}  // namespace pbpt::light::testing
