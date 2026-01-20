#include <gtest/gtest.h>

#include "camera/film.hpp"
#include "camera/pixel_sensor.hpp"
#include "gtest/gtest.h"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/color.hpp"
#include "radiometry/color_spectrum_optimizer.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::camera::testing {
namespace {

using T = double;
using Illuminant = decltype(radiometry::constant::CIE_D65_ilum<T>);
using SensorResponse = radiometry::constant::XYZSpectrumType<T>;
using PixelSensorType = PixelSensor<T, Illuminant, Illuminant, SensorResponse>;
using FilmType = HDRFilm<T, PixelSensorType>;

PixelSensorType make_pixel_sensor(T ratio = T(1)) {
    return PixelSensorType(
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::CIE_D65_ilum<T>,
        radiometry::constant::sRGB<T>,
        radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
            radiometry::constant::CIE_X<T>,
            radiometry::constant::CIE_Y<T>,
            radiometry::constant::CIE_Z<T>
        ),
        ratio
    );
}

FilmType make_film(T ratio = T(1)) {
    math::Vector<int, 2> resolution(2, 2);
    math::Vector<T, 2> physical_size(1.0f, 1.0f);
    return FilmType(resolution, physical_size, make_pixel_sensor(ratio));
}

FilmType make_film(int width, int height, T ratio = T(1)) {
    math::Vector<int, 2> resolution(width, height);
    math::Vector<T, 2> physical_size(1.0f, 1.0f);
    return FilmType(resolution, physical_size, make_pixel_sensor(ratio));
}

}  // namespace

TEST(RGBFilmTest, AddColorSampleWeightedAverage) {
    auto film = make_film();

    math::Point<int, 2> p(0, 0);
    film.add_color_sample(p, radiometry::RGB<T>(1.0f, 2.0f, 3.0f), 1.0f);
    film.add_color_sample(p, radiometry::RGB<T>(3.0f, 1.0f, 0.0f), 3.0f);

    auto rgb = film.get_pixel_rgb(p);
    EXPECT_NEAR(rgb.r(), (1.0f * 1.0f + 3.0f * 3.0f) / 4.0f, 1e-5f);
    EXPECT_NEAR(rgb.g(), (1.0f * 2.0f + 3.0f * 1.0f) / 4.0f, 1e-5f);
    EXPECT_NEAR(rgb.b(), (1.0f * 3.0f + 3.0f * 0.0f) / 4.0f, 1e-5f);

    auto other = film.get_pixel_rgb(math::Point<int, 2>(1, 1));
    EXPECT_NEAR(other.r(), 0.0f, 1e-6f);
    EXPECT_NEAR(other.g(), 0.0f, 1e-6f);
    EXPECT_NEAR(other.b(), 0.0f, 1e-6f);
}

TEST(RGBFilmTest, AddSampleUsesPixelSensorConversion) {
    math::Vector<int, 2> resolution(1, 1);
    math::Vector<T, 2> physical_size(1.0f, 1.0f);
    FilmType film(resolution, physical_size, make_pixel_sensor(T(1.5f)));

    math::Point<int, 2> p(0, 0);
    math::Vector<T, 3> radiance_vec(0.8f, 0.4f, 0.2f);
    radiometry::SampledSpectrum<T, 3> radiance(radiance_vec);
    math::Vector<T, 3> wavelength_vec(410.0f, 520.0f, 650.0f);
    radiometry::SampledWavelength<T, 3> wavelengths(wavelength_vec);
    math::Vector<T, 3> pdf_vec(0.2f, 0.3f, 0.5f);
    radiometry::SampledPdf<T, 3> pdf(pdf_vec);

    const auto& sensor = film.pixel_sensor();
    auto expected_sensor_rgb = sensor.template radiance_to_sensor_rgb<3>(radiance, wavelengths, pdf);
    auto expected_display_rgb = sensor.sensor_rgb_to_color_space_rgb(expected_sensor_rgb);

    film.add_sample<3>(p, radiance, wavelengths, pdf, T(0.7f));

    auto pixel_rgb = film.get_pixel_rgb(p);
    EXPECT_NEAR(pixel_rgb.r(), expected_display_rgb.r(), 1e-4f);
    EXPECT_NEAR(pixel_rgb.g(), expected_display_rgb.g(), 1e-4f);
    EXPECT_NEAR(pixel_rgb.b(), expected_display_rgb.b(), 1e-4f);
}

TEST(RGBFilmTest, AddSampleAccumulatesWithWeights) {
    auto film = make_film();
    math::Point<int, 2> p(1, 1);
    const auto& sensor = film.pixel_sensor();

    math::Vector<T, 3> radiance1_vec(1.0f, 0.5f, 0.25f);
    math::Vector<T, 3> wavelengths1_vec(400.0f, 500.0f, 600.0f);
    math::Vector<T, 3> pdf1_vec(0.4f, 0.3f, 0.3f);
    radiometry::SampledSpectrum<T, 3> radiance1(radiance1_vec);
    radiometry::SampledWavelength<T, 3> wavelengths1(wavelengths1_vec);
    radiometry::SampledPdf<T, 3> pdf1(pdf1_vec);

    math::Vector<T, 3> radiance2_vec(0.3f, 0.6f, 0.9f);
    math::Vector<T, 3> wavelengths2_vec(420.0f, 540.0f, 660.0f);
    math::Vector<T, 3> pdf2_vec(0.2f, 0.4f, 0.4f);
    radiometry::SampledSpectrum<T, 3> radiance2(radiance2_vec);
    radiometry::SampledWavelength<T, 3> wavelengths2(wavelengths2_vec);
    radiometry::SampledPdf<T, 3> pdf2(pdf2_vec);

    T weight1 = 2.0f;
    T weight2 = 1.0f;

    auto expected_display_rgb1 = sensor.sensor_rgb_to_color_space_rgb(
        sensor.template radiance_to_sensor_rgb<3>(radiance1, wavelengths1, pdf1));
    auto expected_display_rgb2 = sensor.sensor_rgb_to_color_space_rgb(
        sensor.template radiance_to_sensor_rgb<3>(radiance2, wavelengths2, pdf2));

    film.add_sample<3>(p, radiance1, wavelengths1, pdf1, weight1);
    film.add_sample<3>(p, radiance2, wavelengths2, pdf2, weight2);

    T total_weight = weight1 + weight2;
    T expected_r = (expected_display_rgb1.r() * weight1 + expected_display_rgb2.r() * weight2) / total_weight;
    T expected_g = (expected_display_rgb1.g() * weight1 + expected_display_rgb2.g() * weight2) / total_weight;
    T expected_b = (expected_display_rgb1.b() * weight1 + expected_display_rgb2.b() * weight2) / total_weight;

    auto pixel_rgb = film.get_pixel_rgb(p);
    EXPECT_NEAR(pixel_rgb.r(), expected_r, 1e-4f);
    EXPECT_NEAR(pixel_rgb.g(), expected_g, 1e-4f);
    EXPECT_NEAR(pixel_rgb.b(), expected_b, 1e-4f);
}

TEST(RGBFilmTest, ClearResetsPixelAccumulation) {
    auto film = make_film();
    math::Point<int, 2> p(0, 0);

    film.add_color_sample(p, radiometry::RGB<T>(0.2f, 0.4f, 0.6f), 1.0f);
    film.clear();

    auto rgb = film.get_pixel_rgb(p);
    EXPECT_NEAR(rgb.r(), 0.0f, 1e-6f);
    EXPECT_NEAR(rgb.g(), 0.0f, 1e-6f);
    EXPECT_NEAR(rgb.b(), 0.0f, 1e-6f);
}

TEST(RGBFilmTest, DevelopGeneratesCorrectImage) {
    auto film = make_film(10, 5);
    int width = film.resolution().x();
    int height = film.resolution().y();

    // Fill film with some predictable data
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float r = static_cast<float>(x) / width;
            float g = static_cast<float>(y) / height;
            float b = 0.5f;
            film.add_color_sample(math::Point<int, 2>(x, y), radiometry::RGB<T>(r, g, b), 1.0f);
        }
    }

    // Develop to Image
    auto image = film.develop();

    // Check dimensions
    ASSERT_EQ(image.width(), width);
    ASSERT_EQ(image.height(), height);

    // Check content
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto rgb = film.get_pixel_rgb(math::Point<int, 2>(x, y));
            auto pixel = image.get_pixel(x, y);

            EXPECT_NEAR(pixel.x(), rgb.r(), 1e-4f);
            EXPECT_NEAR(pixel.y(), rgb.g(), 1e-4f);
            EXPECT_NEAR(pixel.z(), rgb.b(), 1e-4f);
        }
    }
}

TEST(RGBFilmTest, ZeroWeightSampleDoesNotAffectPixel) {
    auto film = make_film();
    math::Point<int, 2> p(0, 0);

    film.add_color_sample(p, radiometry::RGB<T>(1.0f, 1.0f, 1.0f), 0.0f);
    auto rgb = film.get_pixel_rgb(p);

    EXPECT_NEAR(rgb.r(), 0.0f, 1e-6f);
    EXPECT_NEAR(rgb.g(), 0.0f, 1e-6f);
    EXPECT_NEAR(rgb.b(), 0.0f, 1e-6f);
}

TEST(RGBFilmTest, OutOfBoundsAccessThrows) {
    auto film = make_film();

    EXPECT_THROW(
        film.add_color_sample(math::Point<int, 2>(-1, 0), radiometry::RGB<T>(0.1f, 0.2f, 0.3f), 1.0f),
        std::runtime_error
    );
    EXPECT_THROW(
        (void)film.get_pixel_rgb(math::Point<int, 2>(2, 0)),
        std::runtime_error
    );
}

TEST(RGBFilmTest, HalfBlackHalfWhiteRadianceProducesMidGray) {
    math::Vector<int, 2> resolution(1, 1);
    math::Vector<T, 2> physical_size(1.0f, 1.0f);
    FilmType film(resolution, physical_size, make_pixel_sensor());

    auto white_radiance = radiometry::constant::CIE_D65_ilum<T>;
    radiometry::ConstantSpectrumDistribution<T> black_radiance(T(0));

    const auto& sensor = film.pixel_sensor();
    auto white_display_rgb = sensor.sensor_rgb_to_color_space_rgb(sensor.radiance_to_sensor_rgb(white_radiance));
    std::cout << "White Display RGB: " << white_display_rgb << std::endl;
    auto black_display_rgb = sensor.sensor_rgb_to_color_space_rgb(sensor.radiance_to_sensor_rgb(black_radiance));
    std::cout << "Black Display RGB: " << black_display_rgb << std::endl;

    math::Point<int, 2> p(0, 0);
    film.add_sample(p, black_radiance, T(0.25f));
    film.add_sample(p, black_radiance, T(0.25f));
    film.add_sample(p, white_radiance, T(0.25f));
    film.add_sample(p, white_radiance, T(0.25f));

    auto rgb = film.get_pixel_rgb(p);
    std::cout << "Pixel RGB: " << rgb << std::endl;
    auto expected = radiometry::RGB<T>(white_display_rgb * T(0.5f) + black_display_rgb * T(0.5f));
    std::cout << "Expected RGB: " << expected << std::endl;
    EXPECT_NEAR(rgb.r(), expected.r(), 5e-3f);
    EXPECT_NEAR(rgb.g(), expected.g(), 5e-3f);
    EXPECT_NEAR(rgb.b(), expected.b(), 5e-3f);
}

TEST(RGBFilmTest, PrimaryReflectanceSamplesAverageToDarkGray) {
    math::Vector<int, 2> resolution(1, 1);
    math::Vector<T, 2> physical_size(1.0f, 1.0f);
    FilmType film(resolution, physical_size, make_pixel_sensor());
    auto illuminant = radiometry::constant::CIE_D65_ilum<T>;
    constexpr int max_attempts = 30;
    constexpr double lr = 1.0;
    constexpr double eps_ = 0.001;
    constexpr double delta_x = 1e-4;

    // auto red_reflectance = radiometry::constant::swatch_reflectances<T>[static_cast<int>(radiometry::constant::SwatchReflectance::Red)];
    // auto green_reflectance = radiometry::constant::swatch_reflectances<T>[static_cast<int>(radiometry::constant::SwatchReflectance::Green)];
    // auto blue_reflectance = radiometry::constant::swatch_reflectances<T>[static_cast<int>(radiometry::constant::SwatchReflectance::Blue)];
    
    auto red_rsp = radiometry::optimize_albedo_rgb_sigmoid_polynomial(
        radiometry::RGB<T>{1.0, eps_, eps_}, 
        radiometry::constant::sRGB<T>,
        radiometry::constant::CIE_D65_ilum<T>,
        max_attempts,
        lr,
        delta_x
    );

    auto red_reflectance = radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(
        radiometry::RGBSigmoidPolynomialNormalized<T>(
            red_rsp.normalized_coeffs
        )
    );

    auto red_xyz = radiometry::XYZ<T>::from_reflectance(red_reflectance, illuminant);
    std::cout << "Red Reflectance XYZ: " << red_xyz << std::endl;
    auto red_rgb = radiometry::constant::sRGB<T>.to_rgb(red_xyz);
    std::cout << "Red Reflectance RGB: " << red_rgb << std::endl;
    std::cout << std::endl;

    auto green_rsp = radiometry::optimize_albedo_rgb_sigmoid_polynomial(
        radiometry::RGB<T>{eps_, 1.0, eps_}, 
        radiometry::constant::sRGB<T>,
        radiometry::constant::CIE_D65_ilum<T>,
        max_attempts,
        lr,
        delta_x
    );

    auto green_reflectance = radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(
        radiometry::RGBSigmoidPolynomialNormalized<T>(
            green_rsp.normalized_coeffs
        )
    );

    auto green_xyz = radiometry::XYZ<T>::from_reflectance(green_reflectance, illuminant);
    std::cout << "Green Reflectance XYZ: " << green_xyz << std::endl;
    auto green_rgb = radiometry::constant::sRGB<T>.to_rgb(green_xyz);
    std::cout << "Green Reflectance RGB: " << green_rgb << std::endl;
    std::cout << std::endl;

    auto blue_rsp = radiometry::optimize_albedo_rgb_sigmoid_polynomial(
        radiometry::RGB<T>{eps_, eps_, 1.0}, 
        radiometry::constant::sRGB<T>,
        radiometry::constant::CIE_D65_ilum<T>,
        max_attempts,
        lr,
        delta_x
    );

    auto blue_reflectance = radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(
        radiometry::RGBSigmoidPolynomialNormalized<T>(
            blue_rsp.normalized_coeffs
        )
    );

    auto blue_xyz = radiometry::XYZ<T>::from_reflectance(blue_reflectance, illuminant);
    std::cout << "Blue Reflectance XYZ: " << blue_xyz << std::endl;
    auto blue_rgb = radiometry::constant::sRGB<T>.to_rgb(blue_xyz);
    std::cout << "Blue Reflectance RGB: " << blue_rgb << std::endl;
    std::cout << std::endl;

    auto red_radiance = red_reflectance * illuminant;
    auto green_radiance = green_reflectance * illuminant;
    auto blue_radiance = blue_reflectance * illuminant;

    auto red_sensor_rgb = film.pixel_sensor().radiance_to_sensor_rgb(red_radiance);
    std::cout << "Red Sensor RGB: " << red_sensor_rgb << std::endl;
    auto green_sensor_rgb = film.pixel_sensor().radiance_to_sensor_rgb(green_radiance);
    std::cout << "Green Sensor RGB: " << green_sensor_rgb << std::endl;
    auto blue_sensor_rgb = film.pixel_sensor().radiance_to_sensor_rgb(blue_radiance);
    std::cout << "Blue Sensor RGB: " << blue_sensor_rgb << std::endl;

    const auto& sensor = film.pixel_sensor();
    auto red_display = sensor.sensor_rgb_to_color_space_rgb(red_sensor_rgb);
    std::cout << "Red Display RGB: " << red_display << std::endl;
    auto green_display = sensor.sensor_rgb_to_color_space_rgb(green_sensor_rgb);
    std::cout << "Green Display RGB: " << green_display << std::endl;
    auto blue_display = sensor.sensor_rgb_to_color_space_rgb(blue_sensor_rgb);
    std::cout << "Blue Display RGB: " << blue_display << std::endl;

    math::Point<int, 2> p(0, 0);
    film.add_sample(p, red_radiance, T(1.0f / 3.0f));
    film.add_sample(p, green_radiance, T(1.0f / 3.0f));
    film.add_sample(p, blue_radiance, T(1.0f / 3.0f));

    auto rgb = film.get_pixel_rgb(p);
    std::cout << "Pixel RGB: " << rgb << std::endl;
    auto expected = radiometry::RGB<T>((red_display + green_display + blue_display) / T(3));
    std::cout << "Expected RGB: " << expected << std::endl;
    EXPECT_NEAR(rgb.r(), expected.r(), 5e-2f);
    EXPECT_NEAR(rgb.g(), expected.g(), 5e-2f);
    EXPECT_NEAR(rgb.b(), expected.b(), 5e-2f);
}

}  // namespace pbpt::camera::testing
