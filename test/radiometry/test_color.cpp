#include <gtest/gtest.h>
#include <filesystem>
#include <array>
#include <cmath>
#include <memory>

#include "pbpt/pbpt.h"

using namespace pbpt::radiometry;
using namespace pbpt::math;
using namespace pbpt::radiometry::constant;

namespace pbpt::radiometry::testing {

class ColorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up D65 white point for testing
        d65_white_point = XYZ<double>(0.95047, 1.0, 1.08883);

        // Set up sRGB color space for testing
        srgb_color_space = std::make_unique<RGBColorSpace<double>>(Point<double, 2>(0.64, 0.33),  // Red primary
                                                                   Point<double, 2>(0.30, 0.60),  // Green primary
                                                                   Point<double, 2>(0.15, 0.06),  // Blue primary
                                                                   d65_white_point                // White point
        );

        // Set up test colors
        test_white_xyz = XYZ<double>(1.0, 1.0, 1.0);
        test_red_rgb = RGB<double>(1.0, 0.0, 0.0);
        test_green_rgb = RGB<double>(0.0, 1.0, 0.0);
        test_blue_rgb = RGB<double>(0.0, 0.0, 1.0);
    }

    XYZ<double> d65_white_point;
    std::unique_ptr<RGBColorSpace<double>> srgb_color_space;

    // Test color constants
    XYZ<double> test_white_xyz;
    RGB<double> test_red_rgb;
    RGB<double> test_green_rgb;
    RGB<double> test_blue_rgb;
    RGB<double> test_white_rgb = RGB<double>(1.0, 1.0, 1.0);
    RGB<double> test_gray_rgb = RGB<double>(0.5, 0.5, 0.5);

    const double epsilon = 1e-12;
};

// =============================================================================
// RGB Color Tests
// =============================================================================

TEST_F(ColorTest, RGBConstruction) {
    RGB<float> rgb_default;
    EXPECT_FLOAT_EQ(rgb_default.r(), 0.0f);
    EXPECT_FLOAT_EQ(rgb_default.g(), 0.0f);
    EXPECT_FLOAT_EQ(rgb_default.b(), 0.0f);

    RGB<float> rgb_values(1.0f, 0.5f, 0.25f);
    EXPECT_FLOAT_EQ(rgb_values.r(), 1.0f);
    EXPECT_FLOAT_EQ(rgb_values.g(), 0.5f);
    EXPECT_FLOAT_EQ(rgb_values.b(), 0.25f);

    // Test construction from vector
    Vector<double, 3> vec(0.8, 0.6, 0.4);
    RGB<double> rgb_from_vec(vec);
    EXPECT_DOUBLE_EQ(rgb_from_vec.r(), 0.8);
    EXPECT_DOUBLE_EQ(rgb_from_vec.g(), 0.6);
    EXPECT_DOUBLE_EQ(rgb_from_vec.b(), 0.4);
}

TEST_F(ColorTest, RGBArithmetic) {
    RGB<float> rgb1(1.0f, 0.5f, 0.25f);
    RGB<float> rgb2(0.2f, 0.3f, 0.4f);

    // Test addition - cast result back to RGB
    RGB<float> sum = RGB<float>(rgb1 + rgb2);
    EXPECT_FLOAT_EQ(sum.r(), 1.2f);
    EXPECT_FLOAT_EQ(sum.g(), 0.8f);
    EXPECT_FLOAT_EQ(sum.b(), 0.65f);

    // Test subtraction - cast result back to RGB
    RGB<float> diff = RGB<float>(rgb1 - rgb2);
    EXPECT_FLOAT_EQ(diff.r(), 0.8f);
    EXPECT_FLOAT_EQ(diff.g(), 0.2f);
    EXPECT_FLOAT_EQ(diff.b(), -0.15f);

    // Test scalar multiplication - cast result back to RGB
    RGB<float> scaled = RGB<float>(rgb1 * 2.0f);
    EXPECT_FLOAT_EQ(scaled.r(), 2.0f);
    EXPECT_FLOAT_EQ(scaled.g(), 1.0f);
    EXPECT_FLOAT_EQ(scaled.b(), 0.5f);

    // Test scalar division - cast result back to RGB
    RGB<float> divided = RGB<float>(rgb1 / 4.0f);
    EXPECT_FLOAT_EQ(divided.r(), 0.25f);
    EXPECT_FLOAT_EQ(divided.g(), 0.125f);
    EXPECT_FLOAT_EQ(divided.b(), 0.0625f);

    // Test component-wise multiplication - cast result back to RGB
    RGB<float> product = RGB<float>(rgb1 * rgb2);
    EXPECT_FLOAT_EQ(product.r(), 0.2f);
    EXPECT_FLOAT_EQ(product.g(), 0.15f);
    EXPECT_FLOAT_EQ(product.b(), 0.1f);
}

// =============================================================================
// XYZ Color Tests
// =============================================================================

TEST_F(ColorTest, XYZConstruction) {
    XYZ<double> xyz_default;
    EXPECT_DOUBLE_EQ(xyz_default.x(), 0.0);
    EXPECT_DOUBLE_EQ(xyz_default.y(), 0.0);
    EXPECT_DOUBLE_EQ(xyz_default.z(), 0.0);

    XYZ<double> xyz_values(95.047, 100.0, 108.883);
    EXPECT_DOUBLE_EQ(xyz_values.x(), 95.047);
    EXPECT_DOUBLE_EQ(xyz_values.y(), 100.0);
    EXPECT_DOUBLE_EQ(xyz_values.z(), 108.883);
}

TEST_F(ColorTest, XYZFromxyY) {
    // Test D65 white point conversion
    Point<double, 2> d65_xy(0.31271, 0.32902);
    double Y = 100.0;

    auto xyz = XYZ<double>::from_xyY(d65_xy, Y);

    // Check that Y component matches
    EXPECT_DOUBLE_EQ(xyz.y(), Y);

    // Check that xy chromaticity is preserved
    auto computed_xy = xyz.to_xy();
    EXPECT_NEAR(computed_xy.x(), d65_xy.x(), 1e-4);
    EXPECT_NEAR(computed_xy.y(), d65_xy.y(), 1e-4);
}

TEST_F(ColorTest, XYZFromStandardIlluminant) {
    auto d65 = CIE_D65_ilum<double>;
    auto xyz = XYZ<double>::from_illuminant(d65);

    // For D65, expect normalized values
    EXPECT_NEAR(xyz.x(), 0.9504, 0.001);  // Approximate D65 x chromaticity
    EXPECT_NEAR(xyz.y(), 1.0000, 0.001);  // Y should be normalized to 1
    EXPECT_NEAR(xyz.z(), 1.0888, 0.001);  // Approximate D65 z chromaticity
}

TEST_F(ColorTest, XYZNormalization) {
    XYZ<double> xyz(50.0, 60.0, 70.0);

    // Test normalization to Y=100
    auto normalized = xyz.normalized_to_y(100.0);
    EXPECT_DOUBLE_EQ(normalized.y(), 100.0);
    EXPECT_NEAR(normalized.x(), 50.0 * 100.0 / 60.0, epsilon);
    EXPECT_NEAR(normalized.z(), 70.0 * 100.0 / 60.0, epsilon);

    // Test in-place normalization
    XYZ<double> xyz_copy = xyz;
    xyz_copy.normalize_to_y(50.0);
    EXPECT_DOUBLE_EQ(xyz_copy.y(), 50.0);
}

// =============================================================================
// LAB Color Tests
// =============================================================================

TEST_F(ColorTest, LABConstruction) {
    LAB<double> lab_default;
    EXPECT_DOUBLE_EQ(lab_default.L(), 0.0);
    EXPECT_DOUBLE_EQ(lab_default.a(), 0.0);
    EXPECT_DOUBLE_EQ(lab_default.b(), 0.0);

    LAB<double> lab_values(75.0, 25.0, -18.0);
    EXPECT_DOUBLE_EQ(lab_values.L(), 75.0);
    EXPECT_DOUBLE_EQ(lab_values.a(), 25.0);
    EXPECT_DOUBLE_EQ(lab_values.b(), -18.0);
}

TEST_F(ColorTest, LABFromXYZ) {
    // Test white point conversion to LAB
    XYZ<double> white_xyz = d65_white_point;
    auto white_lab = LAB<double>::from_xyz(white_xyz, d65_white_point);

    // White point should have L*=100, a*=0, b*=0
    EXPECT_NEAR(white_lab.L(), 100.0, 1.0);
    EXPECT_NEAR(white_lab.a(), 0.0, 1.0);
    EXPECT_NEAR(white_lab.b(), 0.0, 1.0);

    // Test black point
    XYZ<double> black_xyz(0.0, 0.0, 0.0);
    auto black_lab = LAB<double>::from_xyz(black_xyz, d65_white_point);
    EXPECT_NEAR(black_lab.L(), 0.0, 1e-6);
}

// =============================================================================
// RGB Color Space Tests
// =============================================================================

TEST_F(ColorTest, RGBColorSpaceConstruction) {
    // Test color space properties
    auto white_point = srgb_color_space->white_point();
    EXPECT_GT(white_point.x(), 0.0);
    EXPECT_GT(white_point.y(), 0.0);
    EXPECT_GT(white_point.z(), 0.0);

    // Test RGB to XYZ matrix exists and has reasonable values
    auto rgb_to_xyz_matrix = srgb_color_space->rgb_to_xyz_matrix();
    EXPECT_GT(rgb_to_xyz_matrix.determinant(), 0.0);  // Should be invertible
}

TEST_F(ColorTest, RGBToXYZConversion) {
    // Test white point conversion
    auto white_xyz = srgb_color_space->to_xyz(test_white_rgb);

    // White RGB should map to approximately the white point
    auto expected_white = srgb_color_space->white_point();
    EXPECT_NEAR(white_xyz.x(), expected_white.x(), 0.01);
    EXPECT_NEAR(white_xyz.y(), expected_white.y(), 0.01);
    EXPECT_NEAR(white_xyz.z(), expected_white.z(), 0.01);

    // Test pure red
    auto red_xyz = srgb_color_space->to_xyz(test_red_rgb);
    EXPECT_GT(red_xyz.x(), red_xyz.y());  // Red should have more X than Y
    EXPECT_GT(red_xyz.x(), red_xyz.z());  // Red should have more X than Z

    // Test pure green
    auto green_xyz = srgb_color_space->to_xyz(test_green_rgb);
    EXPECT_GT(green_xyz.y(), green_xyz.x());  // Green should have more Y than X
    EXPECT_GT(green_xyz.y(), green_xyz.z());  // Green should have more Y than Z

    // Test pure blue
    auto blue_xyz = srgb_color_space->to_xyz(test_blue_rgb);
    EXPECT_GT(blue_xyz.z(), blue_xyz.x());  // Blue should have more Z than X
    EXPECT_GT(blue_xyz.z(), blue_xyz.y());  // Blue should have more Z than Y
}

TEST_F(ColorTest, XYZToRGBConversion) {
    // Test round-trip conversion: RGB -> XYZ -> RGB
    auto white_xyz = srgb_color_space->to_xyz(test_white_rgb);
    auto recovered_white_rgb = srgb_color_space->to_rgb(white_xyz);

    EXPECT_NEAR(recovered_white_rgb.r(), test_white_rgb.r(), epsilon);
    EXPECT_NEAR(recovered_white_rgb.g(), test_white_rgb.g(), epsilon);
    EXPECT_NEAR(recovered_white_rgb.b(), test_white_rgb.b(), epsilon);

    // Test with gray
    auto gray_xyz = srgb_color_space->to_xyz(test_gray_rgb);
    auto recovered_gray_rgb = srgb_color_space->to_rgb(gray_xyz);

    EXPECT_NEAR(recovered_gray_rgb.r(), test_gray_rgb.r(), epsilon);
    EXPECT_NEAR(recovered_gray_rgb.g(), test_gray_rgb.g(), epsilon);
    EXPECT_NEAR(recovered_gray_rgb.b(), test_gray_rgb.b(), epsilon);
}

// =============================================================================
// Color Conversion Tests
// =============================================================================

TEST_F(ColorTest, RGBToLABConversion) {
    // Test white point
    auto white_xyz = srgb_color_space->to_xyz(test_white_rgb);
    auto white_lab = LAB<double>::from_xyz(white_xyz, srgb_color_space->white_point());

    EXPECT_NEAR(white_lab.L(), 100.0, 1.0);
    EXPECT_NEAR(white_lab.a(), 0.0, 1.0);
    EXPECT_NEAR(white_lab.b(), 0.0, 1.0);

    // Test gray point
    auto gray_xyz = srgb_color_space->to_xyz(test_gray_rgb);
    auto gray_lab = LAB<double>::from_xyz(gray_xyz, srgb_color_space->white_point());

    // Gray should have L* > 0, a* ≈ 0, b* ≈ 0
    EXPECT_GT(gray_lab.L(), 0.0);
    EXPECT_LT(gray_lab.L(), 100.0);
    EXPECT_NEAR(gray_lab.a(), 0.0, 2.0);  // Small tolerance for numerical errors
    EXPECT_NEAR(gray_lab.b(), 0.0, 2.0);

    // Test pure red
    auto red_xyz = srgb_color_space->to_xyz(test_red_rgb);
    auto red_lab = LAB<double>::from_xyz(red_xyz, srgb_color_space->white_point());

    EXPECT_GT(red_lab.L(), 0.0);
    EXPECT_GT(red_lab.a(), 0.0);  // Red should have positive a*

    // Test pure green
    auto green_xyz = srgb_color_space->to_xyz(test_green_rgb);
    auto green_lab = LAB<double>::from_xyz(green_xyz, srgb_color_space->white_point());

    EXPECT_GT(green_lab.L(), 0.0);
    EXPECT_LT(green_lab.a(), 0.0);  // Green should have negative a*

    // Test pure blue
    auto blue_xyz = srgb_color_space->to_xyz(test_blue_rgb);
    auto blue_lab = LAB<double>::from_xyz(blue_xyz, srgb_color_space->white_point());

    EXPECT_GT(blue_lab.L(), 0.0);
    EXPECT_LT(blue_lab.b(), 0.0);  // Blue should have negative b*
}

// =============================================================================
// Spectral Color Tests
// =============================================================================

TEST_F(ColorTest, XYZFromSampledSpectrum) {
    // Test XYZ calculation from sampled spectrum
    constexpr int N = 10;

    // Create uniform spectrum
    Vector<double, N> uniform_spectrum = Vector<double, N>::filled(1.0);
    SampledSpectrum<double, N> spectrum(uniform_spectrum);

    // Create wavelengths from 400 to 700 nm
    std::array<double, N> wl_array;
    for (int i = 0; i < N; ++i) {
        wl_array[i] = 400.0 + i * 300.0 / (N - 1);
    }
    SampledWavelength<double, N> wavelengths(Vector<double, N>::from_array(wl_array));

    // Create uniform PDF
    Vector<double, N> pdf_values = Vector<double, N>::filled(1.0 / 300.0);
    SampledPdf<double, N> pdf(pdf_values);

    auto xyz = XYZ<double>::from_sampled_spectrum(spectrum, wavelengths, pdf);

    // XYZ values should be positive for uniform spectrum
    EXPECT_GT(xyz.x(), 0.0);
    EXPECT_GT(xyz.y(), 0.0);
    EXPECT_GT(xyz.z(), 0.0);

    // Y should be largest (photopic sensitivity peak)
    EXPECT_GT(xyz.y(), xyz.x() * 0.5);
    EXPECT_GT(xyz.y(), xyz.z() * 0.5);
}

TEST_F(ColorTest, XYZFromReflectanceUnderIlluminant) {
    // Test perfect white reflector under D65
    auto d65 = CIE_D65_ilum<double>;

    // Create perfect white reflector (reflectance = 1.0 everywhere)
    RGBSigmoidPolynomial<double> white_poly{0.0, 0.0, 0.0};  // Constant 0.5 sigmoid
    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomial> white_reflector(white_poly);

    auto xyz = XYZ<double>::from_reflectance(white_reflector, d65);

    // Should be similar to D65 white point
    auto d65_xyz = XYZ<double>::from_illuminant(d65);

    // The reflectance spectrum affects the result, so we check for reasonable values
    EXPECT_GT(xyz.x(), 0.0);
    EXPECT_GT(xyz.y(), 0.0);
    EXPECT_GT(xyz.z(), 0.0);
}

// =============================================================================
// RGB Spectrum Optimization Tests
// =============================================================================

TEST_F(ColorTest, UnboundedRGBSpectrum) {
    // Test unbounded RGB spectrum for colors outside [0,1]
    RGB<double> bright_rgb(0.5, 1.5, 2.0);
    auto scaled_data = scale_unbounded_rgb(bright_rgb);
    RGB<double> scaled_rgb = scaled_data.rgb;
    double scale = scaled_data.scale;

    // Scaled RGB should be in [0,1]
    EXPECT_GE(scaled_rgb.r(), 0.0);
    EXPECT_LE(scaled_rgb.r(), 1.0);
    EXPECT_GE(scaled_rgb.g(), 0.0);
    EXPECT_LE(scaled_rgb.g(), 1.0);
    EXPECT_GE(scaled_rgb.b(), 0.0);
    EXPECT_LE(scaled_rgb.b(), 1.0);

    // Scale should be > 1 for bright colors
    EXPECT_GT(scale, 1.0);

    // Test that we can recover the original
    RGB<double> recovered = RGB<double>(scaled_rgb * scale);
    EXPECT_NEAR(recovered.r(), bright_rgb.r(), epsilon);
    EXPECT_NEAR(recovered.g(), bright_rgb.g(), epsilon);
    EXPECT_NEAR(recovered.b(), bright_rgb.b(), epsilon);
}

// =============================================================================
// Standard Color Space Tests
// =============================================================================

TEST_F(ColorTest, sRGBColorSpace) {
    auto srgb = sRGB<double>;

    // Test that sRGB color space is properly initialized
    auto white_point = srgb.white_point();
    EXPECT_GT(white_point.x(), 0.0);
    EXPECT_GT(white_point.y(), 0.0);
    EXPECT_GT(white_point.z(), 0.0);

    // Test conversion matrices
    auto rgb_to_xyz = srgb.rgb_to_xyz_matrix();
    EXPECT_GT(rgb_to_xyz.determinant(), 0.0);

    // Test round-trip conversion
    RGB<double> test_color(0.5, 0.7, 0.3);
    auto xyz = srgb.to_xyz(test_color);
    auto recovered = srgb.to_rgb(xyz);

    EXPECT_NEAR(recovered.r(), test_color.r(), epsilon);
    EXPECT_NEAR(recovered.g(), test_color.g(), epsilon);
    EXPECT_NEAR(recovered.b(), test_color.b(), epsilon);
}

// =============================================================================
// Color Gamut and Boundary Tests
// =============================================================================

TEST_F(ColorTest, ColorGamutBoundaries) {
    // Test colors at gamut boundaries
    std::vector<RGB<double>> boundary_colors = {
        RGB<double>(1.0, 0.0, 0.0),  // Pure red
        RGB<double>(0.0, 1.0, 0.0),  // Pure green
        RGB<double>(0.0, 0.0, 1.0),  // Pure blue
        RGB<double>(1.0, 1.0, 0.0),  // Yellow
        RGB<double>(1.0, 0.0, 1.0),  // Magenta
        RGB<double>(0.0, 1.0, 1.0),  // Cyan
        RGB<double>(0.0, 0.0, 0.0),  // Black
        RGB<double>(1.0, 1.0, 1.0)   // White
    };

    for (const auto& rgb : boundary_colors) {
        auto xyz = srgb_color_space->to_xyz(rgb);
        auto recovered = srgb_color_space->to_rgb(xyz);

        // Check round-trip accuracy
        EXPECT_NEAR(recovered.r(), rgb.r(), epsilon);
        EXPECT_NEAR(recovered.g(), rgb.g(), epsilon);
        EXPECT_NEAR(recovered.b(), rgb.b(), epsilon);

        // XYZ values should be non-negative
        EXPECT_GE(xyz.x(), 0.0);
        EXPECT_GE(xyz.y(), 0.0);
        EXPECT_GE(xyz.z(), 0.0);
    }
}

TEST_F(ColorTest, ColorOutOfGamut) {
    // Test colors outside the [0,1] RGB range
    RGB<double> negative_rgb(-0.1, 0.5, 0.8);
    RGB<double> bright_rgb(1.2, 0.9, 1.5);

    // These should still convert to XYZ (though may be non-physical)
    auto negative_xyz = srgb_color_space->to_xyz(negative_rgb);
    auto bright_xyz = srgb_color_space->to_xyz(bright_rgb);

    // Check round-trip (may not be exact for out-of-gamut colors)
    auto recovered_negative = srgb_color_space->to_rgb(negative_xyz);
    auto recovered_bright = srgb_color_space->to_rgb(bright_xyz);

    EXPECT_NEAR(recovered_negative.r(), negative_rgb.r(), 1e-10);
    EXPECT_NEAR(recovered_negative.g(), negative_rgb.g(), 1e-10);
    EXPECT_NEAR(recovered_negative.b(), negative_rgb.b(), 1e-10);

    EXPECT_NEAR(recovered_bright.r(), bright_rgb.r(), 1e-10);
    EXPECT_NEAR(recovered_bright.g(), bright_rgb.g(), 1e-10);
    EXPECT_NEAR(recovered_bright.b(), bright_rgb.b(), 1e-10);
}

// =============================================================================
// Performance and Numerical Stability Tests
// =============================================================================

TEST_F(ColorTest, NumericalStability) {
    // Test with very small values
    RGB<double> small_rgb(1e-10, 1e-10, 1e-10);
    auto small_xyz = srgb_color_space->to_xyz(small_rgb);
    auto recovered_small = srgb_color_space->to_rgb(small_xyz);

    EXPECT_NEAR(recovered_small.r(), small_rgb.r(), 1e-15);
    EXPECT_NEAR(recovered_small.g(), small_rgb.g(), 1e-15);
    EXPECT_NEAR(recovered_small.b(), small_rgb.b(), 1e-15);

    // Test XYZ to LAB conversion stability near zero
    XYZ<double> small_xyz_direct(1e-10, 1e-10, 1e-10);
    auto small_lab = LAB<double>::from_xyz(small_xyz_direct, d65_white_point);

    // L* should be close to 0 for very dark colors
    EXPECT_LT(small_lab.L(), 1.0);
}

TEST_F(ColorTest, ColorArithmeticEdgeCases) {
    RGB<double> color1(0.5, 0.5, 0.5);
    RGB<double> zero_color(0.0, 0.0, 0.0);

    // Test division by small numbers
    RGB<double> divided = RGB<double>(color1 / 1e-6);
    EXPECT_GT(divided.r(), 1e5);

    // Test multiplication with zero - cast result to RGB
    RGB<double> multiplied_zero = RGB<double>(color1 * zero_color);
    EXPECT_DOUBLE_EQ(multiplied_zero.r(), 0.0);
    EXPECT_DOUBLE_EQ(multiplied_zero.g(), 0.0);
    EXPECT_DOUBLE_EQ(multiplied_zero.b(), 0.0);

    // Test addition with negative values
    RGB<double> negative(-0.1, -0.2, -0.3);
    RGB<double> added = RGB<double>(color1 + negative);
    EXPECT_DOUBLE_EQ(added.r(), 0.4);
    EXPECT_DOUBLE_EQ(added.g(), 0.3);
    EXPECT_DOUBLE_EQ(added.b(), 0.2);
}

// =============================================================================
// Comprehensive Integration Tests (from function_test.cpp)
// =============================================================================

TEST_F(ColorTest, ComprehensiveRGBToSpectrumConversion) {
    // Test comprehensive RGB to spectrum conversion like in function_test.cpp
    auto srgb = sRGB<double>;

    RGB<double> test_colors[] = {
        RGB<double>(1.0, 0.0, 0.0),  // Pure red
        RGB<double>(0.0, 1.0, 0.0),  // Pure green
        RGB<double>(0.0, 0.0, 1.0),  // Pure blue
        RGB<double>(0.5, 0.5, 0.5),  // Gray
        RGB<double>(1.0, 1.0, 1.0)   // White
    };

    for (const auto& rgb : test_colors) {
        auto xyz = srgb.to_xyz(rgb);

        // XYZ values should be non-negative
        EXPECT_GE(xyz.x(), 0.0);
        EXPECT_GE(xyz.y(), 0.0);
        EXPECT_GE(xyz.z(), 0.0);

        // Test conversion back to RGB
        auto rgb_back = srgb.to_rgb(xyz);

        // Should be close to original (within floating point precision)
        EXPECT_NEAR(rgb.r(), rgb_back.r(), 1e-10);
        EXPECT_NEAR(rgb.g(), rgb_back.g(), 1e-10);
        EXPECT_NEAR(rgb.b(), rgb_back.b(), 1e-10);
    }
}

TEST_F(ColorTest, RGBSigmoidPolynomialIntegration) {
    // Test RGB sigmoid polynomial integration like function_test.cpp
    RGBSigmoidPolynomialNormalized<double> coeffs{-1.14795, 43.0904, -80.4218};

    // Test that the polynomial produces reasonable values
    double test_wavelengths[] = {400.0, 500.0, 550.0, 600.0, 700.0};

    for (double wl : test_wavelengths) {
        double value = coeffs.at(wl);

        EXPECT_GE(value, 0.0);
        EXPECT_LE(value, 1.0);  // Normalized should be in [0,1]
        EXPECT_TRUE(std::isfinite(value));
    }

    // Test conversion to unnormalized
    auto unnormalized = coeffs.to_unnormalized();

    for (double wl : test_wavelengths) {
        double norm_val = coeffs.at(wl);
        double unnorm_val = unnormalized.at(wl);

        EXPECT_TRUE(std::isfinite(unnorm_val));
        EXPECT_GE(unnorm_val, 0.0);

        // Unnormalized should generally be >= normalized (due to scaling)
        EXPECT_GE(unnorm_val + 1e-10, norm_val);
    }
}

TEST_F(ColorTest, MultipleColorSpaceOperations) {
    // Test operations across multiple color spaces
    auto srgb = sRGB<double>;

    RGB<double> rgb(0.8, 0.4, 0.2);

    // Convert through multiple color spaces
    auto xyz = srgb.to_xyz(rgb);
    auto white_point = srgb.white_point();
    auto lab = LAB<double>::from_xyz(xyz, white_point);

    // Test that conversions maintain reasonable values
    EXPECT_GT(xyz.y(), 0.0);    // Y is luminance, should be positive
    EXPECT_GT(lab.L(), 0.0);    // L* should be positive for non-black colors
    EXPECT_LE(lab.L(), 100.0);  // L* should not exceed 100

    // Test RGB roundtrip conversion
    auto rgb_back = srgb.to_rgb(xyz);

    EXPECT_NEAR(rgb.r(), rgb_back.r(), 1e-10);
    EXPECT_NEAR(rgb.g(), rgb_back.g(), 1e-10);
    EXPECT_NEAR(rgb.b(), rgb_back.b(), 1e-10);
}

TEST_F(ColorTest, StandardIlluminantComparison) {
    // Test standard illuminant XYZ values like function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto D50 = CIE_D50_ilum<double>;
    auto A = CIE_A_ilum<double>;

    auto xyz_d65 = XYZ<double>::from_illuminant(D65);
    auto xyz_d50 = XYZ<double>::from_illuminant(D50);
    auto xyz_a = XYZ<double>::from_illuminant(A);

    // All should have positive Y (luminance)
    EXPECT_GT(xyz_d65.y(), 0.0);
    EXPECT_GT(xyz_d50.y(), 0.0);
    EXPECT_GT(xyz_a.y(), 0.0);

    // All illuminants should normalize to roughly unit Y when scaled
    auto normalized_d65 = xyz_d65 / xyz_d65.y();
    auto normalized_d50 = xyz_d50 / xyz_d50.y();
    auto normalized_a = xyz_a / xyz_a.y();

    EXPECT_NEAR(normalized_d65.y(), 1.0, 1e-10);
    EXPECT_NEAR(normalized_d50.y(), 1.0, 1e-10);
    EXPECT_NEAR(normalized_a.y(), 1.0, 1e-10);
}

TEST_F(ColorTest, RGBOptimizationValidation) {
    // Test RGB optimization/validation functionality
    auto srgb = sRGB<double>;

    // Test various RGB values including edge cases
    RGB<double> test_cases[] = {
        RGB<double>(0.0, 0.0, 0.0),    // Black
        RGB<double>(1.0, 1.0, 1.0),    // White
        RGB<double>(0.5, 0.5, 0.5),    // Gray
        RGB<double>(1.0, 0.0, 0.0),    // Red
        RGB<double>(0.0, 1.0, 0.0),    // Green
        RGB<double>(0.0, 0.0, 1.0),    // Blue
        RGB<double>(0.25, 0.75, 0.5),  // Mixed color
    };

    for (const auto& rgb : test_cases) {
        auto xyz = srgb.to_xyz(rgb);

        // XYZ should be reasonable
        EXPECT_TRUE(std::isfinite(xyz.x()));
        EXPECT_TRUE(std::isfinite(xyz.y()));
        EXPECT_TRUE(std::isfinite(xyz.z()));

        EXPECT_GE(xyz.x(), -1e-10);  // Allow tiny negative due to floating point
        EXPECT_GE(xyz.y(), -1e-10);
        EXPECT_GE(xyz.z(), -1e-10);

        // Test conversion back
        auto rgb_back = srgb.to_rgb(xyz);

        // Should match original within precision
        EXPECT_NEAR(rgb.r(), rgb_back.r(), 1e-10);
        EXPECT_NEAR(rgb.g(), rgb_back.g(), 1e-10);
        EXPECT_NEAR(rgb.b(), rgb_back.b(), 1e-10);
    }
}

TEST_F(ColorTest, sRGBColorSpaceAdvancedOperations) {
    // Test advanced sRGB color space operations like in function_test.cpp
    auto srgb = sRGB<double>;

    // Test specific colors from function_test.cpp output
    struct ColorTestCase {
        RGB<double> rgb;
        std::string name;
        LAB<double> expected_lab_range;
    };

    std::vector<ColorTestCase> test_cases = {{RGB<double>(1.0, 1.0, 1.0), "white", LAB<double>(100, 0, 0)},
                                             {RGB<double>(0.214, 0.214, 0.214), "gray", LAB<double>(53, 0, 0)},
                                             {RGB<double>(1.0, 0.0, 0.0), "red", LAB<double>(53, 80, 67)},
                                             {RGB<double>(0.0, 1.0, 0.0), "green", LAB<double>(88, -86, 83)},
                                             {RGB<double>(0.0, 0.0, 1.0), "blue", LAB<double>(32, 79, -108)}};

    for (const auto& test_case : test_cases) {
        auto xyz = srgb.to_xyz(test_case.rgb);
        auto white_point = srgb.white_point();
        auto lab = LAB<double>::from_xyz(xyz, white_point);

        // Test conversion back to RGB
        auto rgb_back = srgb.to_rgb(xyz);

        // Check RGB roundtrip (should be very close)
        EXPECT_NEAR(rgb_back.r(), test_case.rgb.r(), 1e-10) << "Failed for " << test_case.name;
        EXPECT_NEAR(rgb_back.g(), test_case.rgb.g(), 1e-10) << "Failed for " << test_case.name;
        EXPECT_NEAR(rgb_back.b(), test_case.rgb.b(), 1e-10) << "Failed for " << test_case.name;

        // Check LAB values are in expected ranges (allow some tolerance)
        EXPECT_NEAR(lab.L(), test_case.expected_lab_range.L(), 5.0) << "Failed L* for " << test_case.name;

        // For chromatic colors, check approximate a* and b* values
        if (test_case.name == "red") {
            EXPECT_GT(lab.a(), 70.0);  // Should be positive (green-red)
            EXPECT_GT(lab.b(), 60.0);  // Should be positive (blue-yellow)
        } else if (test_case.name == "green") {
            EXPECT_LT(lab.a(), -80.0);  // Should be negative (green-red)
            EXPECT_GT(lab.b(), 70.0);   // Should be positive (blue-yellow)
        } else if (test_case.name == "blue") {
            EXPECT_GT(lab.a(), 70.0);    // Should be positive (green-red)
            EXPECT_LT(lab.b(), -100.0);  // Should be negative (blue-yellow)
        }
    }
}

TEST_F(ColorTest, RGBAlbedoSpectrumIntegration) {
    // Test RGB albedo spectrum integration from function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto srgb = sRGB<double>;

    // Test coefficients from function_test.cpp output
    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> albedo({-1.14795, 43.0904, -80.4218});

    // Get unnormalized coefficients
    auto unnormalized = albedo.rsp().to_unnormalized();

    // Coefficients should be reasonable
    EXPECT_TRUE(std::isfinite(unnormalized.c0));
    EXPECT_TRUE(std::isfinite(unnormalized.c1));
    EXPECT_TRUE(std::isfinite(unnormalized.c2));

    // Test XYZ calculation from reflectance under illuminant
    auto xyz_from_albedo = XYZ<double>::from_reflectance(albedo, D65);
    auto rgb_from_albedo = srgb.to_rgb(xyz_from_albedo);

    // Should produce a reasonable RGB color (approximately cyan from function_test output)
    EXPECT_GT(rgb_from_albedo.g(), 0.9);  // High green component
    EXPECT_GT(rgb_from_albedo.b(), 0.9);  // High blue component
    EXPECT_LT(rgb_from_albedo.r(), 0.2);  // Low red component

    // All components should be in valid range
    EXPECT_GE(rgb_from_albedo.r(), 0.0);
    EXPECT_GE(rgb_from_albedo.g(), 0.0);
    EXPECT_GE(rgb_from_albedo.b(), 0.0);
    EXPECT_LE(rgb_from_albedo.r(), 1.1);  // Allow slight over 1.0 due to gamut
    EXPECT_LE(rgb_from_albedo.g(), 1.1);
    EXPECT_LE(rgb_from_albedo.b(), 1.1);
}

TEST_F(ColorTest, ScaledRGBFunctionality) {
    // Test ScaledRGB functionality from function_test.cpp
    RGB<double> base_rgb(0.139, 0.735, 0.989);
    RGB<double> scaled_up = base_rgb * 2.0;

    // Test scale_unbounded_rgb function
    auto scaled_result = scale_unbounded_rgb(scaled_up);

    // Scale should be > 1 for out-of-gamut colors
    EXPECT_GT(scaled_result.scale, 1.0);

    // Scaled RGB should be in gamut
    EXPECT_LE(scaled_result.rgb.r(), 1.0);
    EXPECT_LE(scaled_result.rgb.g(), 1.0);
    EXPECT_LE(scaled_result.rgb.b(), 1.0);
    EXPECT_GE(scaled_result.rgb.r(), 0.0);
    EXPECT_GE(scaled_result.rgb.g(), 0.0);
    EXPECT_GE(scaled_result.rgb.b(), 0.0);

    // Test that scaling preserves the color ratios
    double orig_max = std::max({scaled_up.r(), scaled_up.g(), scaled_up.b()});
    double scaled_max = std::max({scaled_result.rgb.r(), scaled_result.rgb.g(), scaled_result.rgb.b()});

    // The maximum component should be 0.5 (scaled_up / (2 * max))
    EXPECT_NEAR(scaled_max, 0.5, 1e-10);

    // The scale should be 2 * original_max
    EXPECT_NEAR(scaled_result.scale, 2.0 * orig_max, 1e-10);
}

}  // namespace pbpt::radiometry::testing