#include <gtest/gtest.h>
#include <cmath>

#include "camera/pixel_sensor.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"

namespace pbpt::camera::testing {

TEST(PixelSensorTest, SensorRgbConversionsRespectIdentityMatrix) {
    using T = float;
    using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorType =
        PixelSensor<T, Illuminant, Illuminant, radiometry::constant::XYZSpectrumType<T>>;

    SensorType sensor(
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::sRGB<T>,
        T(1)
    );

    auto matrix = sensor.sensor_rgb_to_xyz_matrix();
    EXPECT_NEAR(matrix[0][0], T(1), 1e-5f);
    EXPECT_NEAR(matrix[1][1], T(1), 1e-5f);
    EXPECT_NEAR(matrix[2][2], T(1), 1e-5f);
    EXPECT_NEAR(matrix[0][1], T(0), 1e-5f);
    EXPECT_NEAR(matrix[0][2], T(0), 1e-5f);
    EXPECT_NEAR(matrix[1][0], T(0), 1e-5f);
    EXPECT_NEAR(matrix[1][2], T(0), 1e-5f);
    EXPECT_NEAR(matrix[2][0], T(0), 1e-5f);
    EXPECT_NEAR(matrix[2][1], T(0), 1e-5f);

    radiometry::RGB<T> sensor_rgb(T(0.31f), T(0.42f), T(0.53f));

    auto xyz = sensor.sensor_rgb_to_xyz(sensor_rgb);
    EXPECT_NEAR(xyz.x(), sensor_rgb.r(), 1e-5f);
    EXPECT_NEAR(xyz.y(), sensor_rgb.g(), 1e-5f);
    EXPECT_NEAR(xyz.z(), sensor_rgb.b(), 1e-5f);

    auto color_rgb = sensor.sensor_rgb_to_color_space_rgb(sensor_rgb);
    auto expected_rgb =
        radiometry::constant::sRGB<T>.to_rgb(radiometry::XYZ<T>(sensor_rgb));

    EXPECT_NEAR(color_rgb.r(), expected_rgb.r(), 1e-5f);
    EXPECT_NEAR(color_rgb.g(), expected_rgb.g(), 1e-5f);
    EXPECT_NEAR(color_rgb.b(), expected_rgb.b(), 1e-5f);
}

TEST(PixelSensorTest, RadianceToSensorRgbAppliesResponseSpectrumAndImageRatio) {
    using T = float;
    using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorType =
        PixelSensor<T, Illuminant, Illuminant, radiometry::constant::XYZSpectrumType<T>>;

    T image_ratio = T(1.7f);
    SensorType sensor(
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::sRGB<T>,
        image_ratio
    );

    const T sigma = T(15.0f);
    radiometry::FunctionalSpectrumDistribution<T> radiance(
        [=](T lambda) -> T {
            auto gaussian = [&](T mean, T amplitude) -> double {
                const double diff = static_cast<double>(lambda - mean);
                const double sigma_val = static_cast<double>(sigma);
                const double exponent = -diff * diff / (2.0 * sigma_val * sigma_val);
                return static_cast<double>(amplitude) * std::exp(exponent);
            };

            const double value =
                gaussian(T(410.0f), T(1.0f)) +
                gaussian(T(510.0f), T(0.6f)) +
                gaussian(T(610.0f), T(0.35f)) +
                gaussian(T(710.0f), T(0.2f));
            return static_cast<T>(value);
        }
    );

    radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>> response(
        radiometry::constant::CIE_X<T>,
        radiometry::constant::CIE_Y<T>,
        radiometry::constant::CIE_Z<T>
    );

    auto projected_rgb =
        radiometry::project_emission<T, radiometry::RGB, decltype(radiance), Illuminant,
                                     radiometry::constant::XYZSpectrumType<T>>(
            radiance, radiometry::constant::CIE_D65_ilum<T>, response);
    auto expected_sensor_rgb = radiometry::RGB<T>(projected_rgb * image_ratio);

    auto sensor_rgb = sensor.radiance_to_sensor_rgb(radiance);

    EXPECT_NEAR(sensor_rgb.r(), expected_sensor_rgb.r(), 1e-4f);
    EXPECT_NEAR(sensor_rgb.g(), expected_sensor_rgb.g(), 1e-4f);
    EXPECT_NEAR(sensor_rgb.b(), expected_sensor_rgb.b(), 1e-4f);

    auto xyz = sensor.sensor_rgb_to_xyz(sensor_rgb);
    EXPECT_NEAR(xyz.x(), expected_sensor_rgb.r(), 1e-4f);
    EXPECT_NEAR(xyz.y(), expected_sensor_rgb.g(), 1e-4f);
    EXPECT_NEAR(xyz.z(), expected_sensor_rgb.b(), 1e-4f);

    auto color_space_rgb = sensor.sensor_rgb_to_color_space_rgb(sensor_rgb);
    auto expected_color_space_rgb =
        radiometry::constant::sRGB<T>.to_rgb(radiometry::XYZ<T>(expected_sensor_rgb));

    EXPECT_NEAR(color_space_rgb.r(), expected_color_space_rgb.r(), 1e-4f);
    EXPECT_NEAR(color_space_rgb.g(), expected_color_space_rgb.g(), 1e-4f);
    EXPECT_NEAR(color_space_rgb.b(), expected_color_space_rgb.b(), 1e-4f);
}

TEST(PixelSensorTest, WhiteBalanceMatrixMatchesExpectedMatrix) {
    using T = float;
    using SceneIlluminant = decltype(radiometry::constant::CIE_D50_ilum<T>);
    using StandardIlluminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorType =
        PixelSensor<T, SceneIlluminant, StandardIlluminant, radiometry::constant::XYZSpectrumType<T>>;

    SensorType sensor(
        radiometry::constant::CIE_D50_ilum<T>,
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::sRGB<T>,
        T(1)
    );

    auto src_xy = radiometry::XYZ<T>::from_illuminant(radiometry::constant::CIE_D50_ilum<T>).to_xy();
    auto dst_xy = radiometry::XYZ<T>::from_illuminant(radiometry::constant::CIE_D65_ilum<T>).to_xy();
    auto expected_matrix = radiometry::white_balance<T>(src_xy, dst_xy);
    auto sensor_matrix = sensor.sensor_rgb_to_xyz_matrix();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(sensor_matrix[i][j], expected_matrix[i][j], 1e-5f);
        }
    }
}

TEST(PixelSensorTest, ZeroRadianceProducesZeroSensorRgb) {
    using T = float;
    constexpr int N = 6;
    using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
    using SensorType =
        PixelSensor<T, Illuminant, Illuminant, radiometry::constant::XYZSpectrumType<T>>;

    SensorType sensor(
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::sRGB<T>,
        T(2)
    );

    auto zero_vec = math::Vector<T, N>::zeros();
    radiometry::SampledSpectrum<T, N> radiance(zero_vec);
    radiometry::SampledWavelength<T, N> wavelengths(math::Vector<T, N>::filled(T(500)));
    radiometry::SampledPdf<T, N> pdf(math::Vector<T, N>::filled(T(1)));

    auto sensor_rgb = sensor.template radiance_to_sensor_rgb<N>(radiance, wavelengths, pdf);

    EXPECT_NEAR(sensor_rgb.r(), T(0), 1e-6f);
    EXPECT_NEAR(sensor_rgb.g(), T(0), 1e-6f);
    EXPECT_NEAR(sensor_rgb.b(), T(0), 1e-6f);
}


}  // namespace pbpt::camera::testing
