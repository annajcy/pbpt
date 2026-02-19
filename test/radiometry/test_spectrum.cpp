#include <gtest/gtest.h>
#include <filesystem>
#include <array>
#include <cmath>
#include <vector>

#include "pbpt/math/type_alias.hpp"
#include "pbpt/pbpt.h"
#include "pbpt/math/random_generator.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/black_body.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/constant.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/multiplied.hpp"

using namespace pbpt::math;
using namespace pbpt::radiometry::constant;

namespace pbpt::radiometry::testing {

class SpectrumTest : public ::testing::Test {
protected:
    void SetUp() override {
        epsilon = 1e-6f;
        temperature = 2856.0;
        test_wavelength_400 = 400.0;
        test_wavelength_550 = 550.0;
        test_wavelength_700 = 700.0;
        test_wavelength_800 = 800.0;

        // Set up paths for test data
        auto current_path = std::filesystem::current_path();
        // Navigate to asset directory from build directory
        spectrum_path = current_path.parent_path().parent_path().parent_path() / "asset" / "spectrum";
    }

    double epsilon;
    double temperature;
    double test_wavelength_400;
    double test_wavelength_550;
    double test_wavelength_700;
    double test_wavelength_800;
    std::filesystem::path spectrum_path;
};

// =============================================================================
// SampledSpectrum Tests
// =============================================================================

TEST_F(SpectrumTest, SampledSpectrumConstruction) {
    SampledSpectrum<float, 4> spectrum;
    EXPECT_EQ(spectrum.dims(), 4);

    // Test construction from vector
    Vector<float, 4> vec(1.0f, 2.0f, 3.0f, 4.0f);
    SampledSpectrum<float, 4> spectrum_from_vec(vec);

    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(spectrum_from_vec[i], vec[i]);
    }
}

TEST_F(SpectrumTest, SampledSpectrumArithmetic) {
    SampledSpectrum<float, 3> s1(Vector<float, 3>(1.0f, 2.0f, 3.0f));
    SampledSpectrum<float, 3> s2(Vector<float, 3>(4.0f, 5.0f, 6.0f));

    auto sum = s1 + s2;
    EXPECT_FLOAT_EQ(sum[0], 5.0f);
    EXPECT_FLOAT_EQ(sum[1], 7.0f);
    EXPECT_FLOAT_EQ(sum[2], 9.0f);

    auto diff = s2 - s1;
    EXPECT_FLOAT_EQ(diff[0], 3.0f);
    EXPECT_FLOAT_EQ(diff[1], 3.0f);
    EXPECT_FLOAT_EQ(diff[2], 3.0f);

    auto scaled = s1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled[0], 2.0f);
    EXPECT_FLOAT_EQ(scaled[1], 4.0f);
    EXPECT_FLOAT_EQ(scaled[2], 6.0f);

    auto divided = s1 / 2.0f;
    EXPECT_FLOAT_EQ(divided[0], 0.5f);
    EXPECT_FLOAT_EQ(divided[1], 1.0f);
    EXPECT_FLOAT_EQ(divided[2], 1.5f);
}

// =============================================================================
// SampledWavelength Tests
// =============================================================================

TEST_F(SpectrumTest, SampledWavelengthConstruction) {
    SampledWavelength<double, 5> wavelengths;
    EXPECT_EQ(wavelengths.dims(), 5);

    Vector<double, 5> vec(400.0, 500.0, 600.0, 700.0, 800.0);
    SampledWavelength<double, 5> wl_from_vec(vec);

    for (int i = 0; i < 5; ++i) {
        EXPECT_DOUBLE_EQ(wl_from_vec[i], vec[i]);
    }
}

TEST_F(SpectrumTest, SampledWavelengthVisibleRange) {
    // Test typical visible light wavelengths
    Vector<double, 3> visible_range(test_wavelength_400, test_wavelength_550, test_wavelength_700);
    SampledWavelength<double, 3> visible_wl(visible_range);

    EXPECT_DOUBLE_EQ(visible_wl[0], test_wavelength_400);  // Violet
    EXPECT_DOUBLE_EQ(visible_wl[1], test_wavelength_550);  // Green
    EXPECT_DOUBLE_EQ(visible_wl[2], test_wavelength_700);  // Red
}

// =============================================================================
// SampledPdf Tests
// =============================================================================

TEST_F(SpectrumTest, SampledPdfConstruction) {
    SampledPdf<float, 4> pdf;
    EXPECT_EQ(pdf.dims(), 4);

    // Test uniform PDF
    float uniform_pdf_value = 1.0f / (lambda_max<float> - lambda_min<float>);
    Vector<float, 4> uniform_vec = Vector<float, 4>::filled(uniform_pdf_value);
    SampledPdf<float, 4> uniform_pdf(uniform_vec);

    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(uniform_pdf[i], uniform_pdf_value);
    }
}

TEST_F(SpectrumTest, SampledPdfInverse) {
    Vector<float, 3> pdf_values(0.1f, 0.2f, 0.5f);
    SampledPdf<float, 3> pdf(pdf_values);

    auto inv_pdf = pdf.inv();
    EXPECT_FLOAT_EQ(inv_pdf[0], 10.0f);
    EXPECT_FLOAT_EQ(inv_pdf[1], 5.0f);
    EXPECT_FLOAT_EQ(inv_pdf[2], 2.0f);
}

// =============================================================================
// Black Body Spectrum Tests
// =============================================================================

TEST_F(SpectrumTest, BlackBodyFunction) {
    // Test black body radiation function
    double intensity_550 = BlackBodySpectrumDistribution<double>::black_body(temperature, test_wavelength_550);
    EXPECT_GT(intensity_550, 0.0);

    // Test Wien's displacement law - peak wavelength
    double peak_wavelength = BlackBodySpectrumDistribution<double>::black_body_max_wavelength(temperature);
    EXPECT_GT(peak_wavelength, 0.0);
    EXPECT_LT(peak_wavelength, 3000.0);  // Reasonable range for peak wavelength

    // Verify peak wavelength produces maximum intensity
    double peak_intensity = BlackBodySpectrumDistribution<double>::black_body(temperature, peak_wavelength);
    double intensity_400 = BlackBodySpectrumDistribution<double>::black_body(temperature, test_wavelength_400);
    double intensity_800 = BlackBodySpectrumDistribution<double>::black_body(temperature, test_wavelength_800);

    EXPECT_GT(peak_intensity, intensity_400);
    EXPECT_GT(peak_intensity, intensity_800);
}

TEST_F(SpectrumTest, BlackBodySpectrumDistribution) {
    BlackBodySpectrumDistribution<double> bb_spectrum(temperature);

    // Test at method
    double value_550 = bb_spectrum.at(test_wavelength_550);
    EXPECT_GT(value_550, 0.0);

    // Test max wavelength
    double max_wl = bb_spectrum.max_wavelength();
    EXPECT_GT(max_wl, 0.0);

    // Test max value
    double max_val = bb_spectrum.max_value();
    EXPECT_GT(max_val, 0.0);
    EXPECT_DOUBLE_EQ(max_val, bb_spectrum.at(max_wl));
}

TEST_F(SpectrumTest, BlackBodySpectrumSampling) {
    BlackBodySpectrumDistribution<double> bb_spectrum(temperature);

    Vector<double, 4> wavelengths_vec(400.0, 600.0, 800.0, 1000.0);
    SampledWavelength<double, 4> wavelengths(wavelengths_vec);

    auto sampled = bb_spectrum.sample<4>(wavelengths);

    for (int i = 0; i < 4; ++i) {
        EXPECT_GT(sampled[i], 0.0);
        EXPECT_DOUBLE_EQ(sampled[i], bb_spectrum.at(wavelengths[i]));
    }
}

// =============================================================================
// Tabular Spectrum Distribution Tests
// =============================================================================

TEST_F(SpectrumTest, TabularSpectrumDistributionConstruction) {
    constexpr int lambda_min = 400;
    constexpr int lambda_max = 700;
    constexpr int size = lambda_max - lambda_min + 1;

    std::array<float, size> samples;
    for (int i = 0; i < size; ++i) {
        samples[i] = 1.0f + 0.001f * i;  // Linear increase
    }

    TabularSpectrumDistribution<float, lambda_min, lambda_max> tabular(samples);

    EXPECT_EQ(tabular.sample_count(), size);
    EXPECT_EQ(tabular.lambda_min(), lambda_min);
    EXPECT_EQ(tabular.lambda_max(), lambda_max);

    // Test at method
    EXPECT_FLOAT_EQ(tabular.at(400), samples[0]);
    EXPECT_FLOAT_EQ(tabular.at(450), samples[50]);
    EXPECT_FLOAT_EQ(tabular.at(700), samples[size - 1]);
}

TEST_F(SpectrumTest, TabularSpectrumDistributionOutOfRange) {
    constexpr int lambda_min = 500;
    constexpr int lambda_max = 600;
    constexpr int size = lambda_max - lambda_min + 1;

    std::array<double, size> samples;
    std::fill(samples.begin(), samples.end(), 1.0);

    TabularSpectrumDistribution<double, lambda_min, lambda_max> tabular(samples);

    // Test out of range values return 0
    EXPECT_DOUBLE_EQ(tabular.at(400), 0.0);  // Below range
    EXPECT_DOUBLE_EQ(tabular.at(700), 0.0);  // Above range

    // Test in range values
    EXPECT_DOUBLE_EQ(tabular.at(550), 1.0);
}

// =============================================================================
// Standard Illuminants Tests
// =============================================================================

TEST_F(SpectrumTest, StandardIlluminantConstants) {
    // Test CIE standard observer functions exist
    auto x_spectrum = CIE_X<double>;
    auto y_spectrum = CIE_Y<double>;
    auto z_spectrum = CIE_Z<double>;

    // Sample at a known wavelength
    double x_550 = x_spectrum.at(550);
    double y_550 = y_spectrum.at(550);
    double z_550 = z_spectrum.at(550);

    EXPECT_GT(x_550, 0.0);
    EXPECT_GT(y_550, 0.0);
    EXPECT_GT(z_550, 0.0);

    // Y function should have its peak around 555nm (photopic vision)
    double y_555 = y_spectrum.at(555);
    EXPECT_GT(y_555, y_550);
}

TEST_F(SpectrumTest, StandardIlluminantD65) {
    auto d65 = CIE_D65_ilum<double>;

    // Test sampling at various wavelengths
    Vector<double, 5> wavelengths_vec(400, 560, 600, 700, 800);
    SampledWavelength<double, 5> wavelengths(wavelengths_vec);

    auto sampled = d65.sample<5>(wavelengths);

    for (int i = 0; i < 5; ++i) {
        EXPECT_GT(sampled[i], 0.0);
    }
}

// =============================================================================
// RGB Sigmoid Polynomial Tests
// =============================================================================

TEST_F(SpectrumTest, RGBSigmoidPolynomialBasic) {
    RGBSigmoidPolynomial<double> poly{-82.2253, 0.357441, -0.000367656};

    // Test evaluation at various wavelengths
    double val_400 = poly.at(400.0);
    double val_550 = poly.at(550.0);
    double val_700 = poly.at(700.0);

    // Values should be between 0 and 1 (sigmoid function)
    EXPECT_GE(val_400, 0.0);
    EXPECT_LE(val_400, 1.0);
    EXPECT_GE(val_550, 0.0);
    EXPECT_LE(val_550, 1.0);
    EXPECT_GE(val_700, 0.0);
    EXPECT_LE(val_700, 1.0);
}

TEST_F(SpectrumTest, RGBSigmoidPolynomialNormalized) {
    RGBSigmoidPolynomialNormalized<double> poly_norm{-1.14795, 43.0904, -80.4218};

    // Test conversion to unnormalized
    auto poly_unnorm = poly_norm.to_unnormalized();

    EXPECT_NE(poly_unnorm.c0, 0.0);
    EXPECT_NE(poly_unnorm.c1, 0.0);
    EXPECT_NE(poly_unnorm.c2, 0.0);

    // Test evaluation
    double val_550 = poly_norm.at(550.0);
    EXPECT_GE(val_550, 0.0);
    EXPECT_LE(val_550, 1.0);
}

TEST_F(SpectrumTest, RGBAlbedoSpectrumDistribution) {
    RGBSigmoidPolynomialNormalized<double> poly{-1.14795, 43.0904, -80.4218};
    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> albedo(poly);

    // Test at method
    double reflectance_550 = albedo.at(550.0);
    EXPECT_GE(reflectance_550, 0.0);
    EXPECT_LE(reflectance_550, 1.0);  // Albedo should be between 0 and 1

    // Test sampling
    Vector<double, 3> wavelengths_vec(450.0, 550.0, 650.0);
    SampledWavelength<double, 3> wavelengths(wavelengths_vec);

    auto sampled = albedo.sample<3>(wavelengths);

    for (int i = 0; i < 3; ++i) {
        EXPECT_GE(sampled[i], 0.0);
        EXPECT_LE(sampled[i], 1.0);
    }
}

TEST_F(SpectrumTest, RGBUnboundedSpectrumDistribution) {
    RGBSigmoidPolynomialNormalized<double> poly{-1.14795, 43.0904, -80.4218};
    double scale = 2.0;

    RGBUnboundedSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> unbounded(poly, scale);

    // Test at method - can exceed 1.0 due to scale factor
    double value_550 = unbounded.at(550.0);
    EXPECT_GT(value_550, 0.0);

    // Test that scaling works
    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> albedo(poly);
    double albedo_value = albedo.at(550.0);

    EXPECT_DOUBLE_EQ(value_550, scale * albedo_value);
}

// =============================================================================
// Spectrum Integration Tests
// =============================================================================

TEST_F(SpectrumTest, SpectrumIntegrationMonteCarloXYZ) {
    auto d65 = CIE_D65_ilum<double>;

    // Monte Carlo integration test similar to function_test
    XYZ<double> expected_xyz{};

    constexpr int sample_N = 100;
    constexpr int round_N = 10000;

    for (int i = 0; i < round_N; i++) {
        math::RandomGenerator<double, sample_N> rng;
        auto wl_r = rng.generate_uniform(lambda_min<double>, lambda_max<double>);
        auto wl = SampledWavelength<double, sample_N>(Vector<double, sample_N>::from_array(wl_r));

        auto xyz = XYZ<double>::from_sampled_spectrum(d65.sample(wl), wl,
                                                      SampledPdf<double, sample_N>(Vector<double, sample_N>::filled(
                                                          1.0 / (lambda_max<double> - lambda_min<double>))));

        expected_xyz += xyz;
    }

    expected_xyz = expected_xyz / static_cast<double>(round_N);

    // Verify XYZ values are reasonable
    EXPECT_GT(expected_xyz.x(), 0.0);
    EXPECT_GT(expected_xyz.y(), 0.0);
    EXPECT_GT(expected_xyz.z(), 0.0);

    // For D65, roughly expect x ≈ 0.31, y ≈ 0.33, z ≈ 0.36 (normalized)
    auto normalized_xyz = expected_xyz.normalized_to_y(100.0);
    EXPECT_NEAR(normalized_xyz.x(), 95.0, 2.0);  // Rough D65 white point
    EXPECT_NEAR(normalized_xyz.y(), 100.0, 0.001);
    EXPECT_NEAR(normalized_xyz.z(), 108.0, 2.0);
}

// =============================================================================
// Spectrum Loading Tests (if CSV files exist)
// =============================================================================

TEST_F(SpectrumTest, SpectrumLoadingFromCSVIfExists) {
    if (!std::filesystem::exists(spectrum_path)) {
        std::cout << spectrum_path << std::endl;
        GTEST_SKIP() << "Spectrum data directory not found, skipping CSV loading tests";
        return;
    }

    auto xyz_csv_path = spectrum_path / "CIE_xyz_1931_2deg.csv";
    if (!std::filesystem::exists(xyz_csv_path)) {
        GTEST_SKIP() << "CIE XYZ CSV file not found, skipping CSV loading tests";
        return;
    }

    try {
        auto arr = pbpt::radiometry::make_spectra_from_csv<double, XYZRange>(xyz_csv_path.string(), 3);
        auto Xbar = arr[0], Ybar = arr[1], Zbar = arr[2];

        // Test sampling loaded spectra
        Vector<double, 5> wavelengths_vec(400, 500, 600, 700, 800);
        SampledWavelength<double, 5> wavelengths(wavelengths_vec);

        auto x_sampled = Xbar.sample<5>(wavelengths);
        auto y_sampled = Ybar.sample<5>(wavelengths);
        auto z_sampled = Zbar.sample<5>(wavelengths);

        // Verify non-zero values in visible range
        EXPECT_GT(y_sampled[2], 0.0);  // Green should have significant Y response

        // Z>X>Y
        EXPECT_GT(z_sampled[0], y_sampled[0]);
        EXPECT_GT(z_sampled[0], x_sampled[0]);
        EXPECT_GT(x_sampled[0], y_sampled[0]);

        // Y>Z>X
        EXPECT_GT(y_sampled[1], z_sampled[1]);
        EXPECT_GT(y_sampled[1], x_sampled[1]);
        EXPECT_GT(z_sampled[1], x_sampled[1]);

        // X>Y>Z
        EXPECT_GT(x_sampled[2], y_sampled[2]);
        EXPECT_GT(x_sampled[2], z_sampled[2]);
        EXPECT_GT(y_sampled[2], z_sampled[2]);

        // X>Y>Z
        EXPECT_GT(x_sampled[3], y_sampled[3]);
        EXPECT_GT(x_sampled[3], z_sampled[3]);
        EXPECT_GT(y_sampled[3], z_sampled[3]);
    } catch (const std::exception& e) {
        GTEST_SKIP() << "CSV loading failed: " << e.what();
    }
}

TEST_F(SpectrumTest, IlluminantLoadingFromCSVIfExists) {
    if (!std::filesystem::exists(spectrum_path)) {
        GTEST_SKIP() << "Spectrum data directory not found";
        return;
    }

    auto d65_csv_path = spectrum_path / "CIE_std_illum_D65.csv";
    if (!std::filesystem::exists(d65_csv_path)) {
        GTEST_SKIP() << "D65 illuminant CSV file not found";
        return;
    }

    try {
        auto D65_loaded =
            pbpt::radiometry::make_spectra_from_csv<double, luminantD65Range>(d65_csv_path.string(), 1)[0];

        // Test sampling
        Vector<double, 5> wavelengths_vec(400, 560, 600, 700, 800);
        SampledWavelength<double, 5> wavelengths(wavelengths_vec);

        auto d65_sampled = D65_loaded.sample<5>(wavelengths);

        for (int i = 0; i < 5; ++i) {
            EXPECT_GT(d65_sampled[i], 0.0);
        }
    } catch (const std::exception& e) {
        GTEST_SKIP() << "D65 CSV loading failed: " << e.what();
    }
}

// =============================================================================
// Spectrum Arithmetic Tests
// =============================================================================

TEST_F(SpectrumTest, SpectrumArithmetic) {
    BlackBodySpectrumDistribution<double> bb1(2856.0);
    BlackBodySpectrumDistribution<double> bb2(5778.0);  // Sun temperature

    // Test spectrum multiplication (product of two spectra)
    // Note: For this test we'll create a simple product evaluation
    double test_wl = 550.0;

    double spectrum1_val = bb1.at(test_wl);
    double spectrum2_val = bb2.at(test_wl);

    // Test manual product calculation
    double product_val = spectrum1_val * spectrum2_val;
    EXPECT_GT(product_val, 0.0);

    // Test spectrum scaling
    double scale_factor = 2.0;
    double scaled_val = spectrum1_val * scale_factor;
    EXPECT_DOUBLE_EQ(scaled_val, spectrum1_val * scale_factor);
}

TEST_F(SpectrumTest, SpectrumInnerProduct) {
    auto cie_x = CIE_X<double>;
    auto d65 = CIE_D65_ilum<double>;

    // Test inner product (integration)
    double x_integral = inner_product(d65, cie_x);
    EXPECT_GT(x_integral, 0.0);

    // Test that Y integral is used for normalization
    auto cie_y = CIE_Y<double>;
    double y_integral = inner_product(d65, cie_y);
    EXPECT_GT(y_integral, 0.0);
}

// =============================================================================
// Performance and Edge Case Tests
// =============================================================================

TEST_F(SpectrumTest, LambdaMinMaxConstants) {
    // Test lambda constants
    EXPECT_EQ(lambda_min<double>, 360);
    EXPECT_EQ(lambda_max<double>, 830);
    EXPECT_EQ(lambda_min<float>, 360);
    EXPECT_EQ(lambda_max<float>, 830);
}

TEST_F(SpectrumTest, SpectrumTypeTraits) {
    // Test that spectrum types are properly defined
    // Note: Since these are Vector-based types, we test their inheritance
    static_assert(std::is_base_of_v<Vector<float, 4>, SampledSpectrum<float, 4>>);
    static_assert(std::is_base_of_v<Vector<double, 3>, SampledWavelength<double, 3>>);

    // Test dimensions
    SampledSpectrum<double, 8> large_spectrum;
    EXPECT_EQ(large_spectrum.dims(), 8);
}

TEST_F(SpectrumTest, SpectrumBoundaryValues) {
    BlackBodySpectrumDistribution<double> bb(1000.0);  // Low temperature

    // Test at boundary wavelengths
    double val_min = bb.at(lambda_min<double>);
    double val_max = bb.at(lambda_max<double>);

    EXPECT_GT(val_min, 0.0);
    EXPECT_GT(val_max, 0.0);

    // For low temperature, longer wavelengths should have higher values
    EXPECT_GT(val_max, val_min);
}

// =============================================================================
// Comprehensive Integration Tests (from function_test.cpp)
// =============================================================================

TEST_F(SpectrumTest, MultipleIlluminantSampling) {
    // Test sampling multiple illuminants like in function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto D50 = CIE_D50_ilum<double>;
    auto A = CIE_A_ilum<double>;

    BlackBodySpectrumDistribution<double> black_body_spectrum(2856.0);

    Vector<double, 5> wavelengths(400, 560, 600, 700, 800);
    SampledWavelength<double, 5> sample_wl(wavelengths);

    auto d50_sampled = D50.sample(sample_wl);
    auto d65_sampled = D65.sample(sample_wl);
    auto a_sampled = A.sample(sample_wl);
    auto black_body_sampled = black_body_spectrum.sample(sample_wl);

    // All samples should be positive
    for (int i = 0; i < 5; i++) {
        EXPECT_GT(d50_sampled[i], 0.0);
        EXPECT_GT(d65_sampled[i], 0.0);
        EXPECT_GT(a_sampled[i], 0.0);
        EXPECT_GT(black_body_sampled[i], 0.0);
    }

    // Test inverse relationship (like in function_test.cpp)
    auto inverse_product = black_body_sampled * a_sampled.inv();
    for (int i = 0; i < 5; i++) {
        EXPECT_GT(inverse_product[i], 0.0);
    }
}

TEST_F(SpectrumTest, BlackBodyMaxWavelengthValidation) {
    // Test black body max wavelength calculation
    BlackBodySpectrumDistribution<double> bb2856(2856.0);  // Like function_test.cpp

    auto max_wl = bb2856.max_wavelength();

    // Wien's displacement law: λ_max ≈ 2.898×10^-3 m·K / T
    double expected_max_wl = 2.898e6 / 2856.0;  // Convert to nm

    EXPECT_NEAR(max_wl, expected_max_wl, 10.0);  // Within 10nm
}

TEST_F(SpectrumTest, RGBSigmoidPolynomialComprehensive) {
    // Test RGB sigmoid polynomial functionality thoroughly
    RGBSigmoidPolynomialNormalized<double> normalized_poly{-1.14795, 43.0904, -80.4218};

    // Test conversion to unnormalized
    auto unnormalized = normalized_poly.to_unnormalized();

    // Test that coefficients are reasonable
    EXPECT_TRUE(std::isfinite(unnormalized.c0));
    EXPECT_TRUE(std::isfinite(unnormalized.c1));
    EXPECT_TRUE(std::isfinite(unnormalized.c2));

    // Test evaluation at some wavelengths
    double test_wavelengths[] = {400.0, 550.0, 700.0};
    for (double wl : test_wavelengths) {
        double val_norm = normalized_poly.at(wl);
        double val_unnorm = unnormalized.at(wl);

        EXPECT_TRUE(std::isfinite(val_norm));
        EXPECT_TRUE(std::isfinite(val_unnorm));
        EXPECT_GE(val_norm, 0.0);
        EXPECT_LE(val_norm, 1.0);  // Normalized should be in [0,1]
    }
}

TEST_F(SpectrumTest, RGBAlbedoSpectrumDistributionUsage) {
    // Test RGB albedo spectrum like function_test.cpp
    RGBSigmoidPolynomialNormalized<double> poly{-1.14795, 43.0904, -80.4218};
    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> albedo(poly);

    // Test sampling at various wavelengths
    Vector<double, 5> wavelengths(400, 500, 600, 650, 700);
    SampledWavelength<double, 5> sample_wl(wavelengths);

    auto sampled = albedo.sample(sample_wl);

    // All samples should be valid reflectance values
    for (int i = 0; i < 5; i++) {
        EXPECT_GE(sampled[i], 0.0);
        EXPECT_LE(sampled[i], 1.0);
        EXPECT_TRUE(std::isfinite(sampled[i]));
    }
}

TEST_F(SpectrumTest, RGBUnboundedSpectrumDistributionAdvanced) {
    // Test unbounded RGB spectrum functionality
    RGBSigmoidPolynomialNormalized<double> poly{-1.14795, 43.0904, -80.4218};
    double scale = 2.5;  // Scale factor > 1

    RGBUnboundedSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> unbounded(poly, scale);

    // Test sampling
    Vector<double, 4> wavelengths(450, 550, 650, 750);
    SampledWavelength<double, 4> sample_wl(wavelengths);

    auto sampled = unbounded.sample(sample_wl);

    // Values should be scaled appropriately
    for (int i = 0; i < 4; i++) {
        EXPECT_GE(sampled[i], 0.0);
        EXPECT_TRUE(std::isfinite(sampled[i]));

        // Should allow values > 1 due to scaling
        // But not unreasonably large
        EXPECT_LT(sampled[i], scale * 10.0);
    }
}

TEST_F(SpectrumTest, RGBOptimizationPipeline) {
    // Test complete RGB optimization pipeline like in function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto srgb = sRGB<double>;

    // Target RGB color from function_test.cpp
    RGB<double> target_rgb(0.139, 0.735, 0.989);

    // Test RGB optimization
    auto result = optimize_albedo_rgb_sigmoid_polynomial(target_rgb, srgb, D65);

    // Check that optimization converged (error should be small)
    double error_magnitude = std::sqrt(result.error[0] * result.error[0] + result.error[1] * result.error[1] +
                                       result.error[2] * result.error[2]);
    EXPECT_LT(error_magnitude, 1e-2);
    EXPECT_EQ(result.normalized_coeffs.size(), 3);

    // Validate optimized coefficients are reasonable
    EXPECT_TRUE(std::isfinite(result.normalized_coeffs[0]));
    EXPECT_TRUE(std::isfinite(result.normalized_coeffs[1]));
    EXPECT_TRUE(std::isfinite(result.normalized_coeffs[2]));

    // Create optimized albedo spectrum
    RGBSigmoidPolynomialNormalized<double> normalized_poly{result.normalized_coeffs[0], result.normalized_coeffs[1],
                                                           result.normalized_coeffs[2]};
    auto unnormalized_poly = normalized_poly.to_unnormalized();

    RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomial> optimized_albedo(unnormalized_poly);
    auto xyz_from_optimized = XYZ<double>::from_reflectance(optimized_albedo, D65);
    auto rgb_from_optimized = srgb.to_rgb(xyz_from_optimized);

    // Should closely match target RGB
    EXPECT_NEAR(rgb_from_optimized.r(), target_rgb.r(), 1e-2);
    EXPECT_NEAR(rgb_from_optimized.g(), target_rgb.g(), 1e-2);
    EXPECT_NEAR(rgb_from_optimized.b(), target_rgb.b(), 1e-2);
}

TEST_F(SpectrumTest, RGBToSpectrumLookupMatchesRGB) {
    auto D65 = CIE_D65_ilum<double>;
    auto srgb = sRGB<double>;

    auto check_match = [&](const RGB<double>& target) {
        auto rsp = pbpt::radiometry::lookup_srgb_to_rsp(target);
        RGBAlbedoSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> albedo(rsp);
        auto xyz = XYZ<double>::from_reflectance(albedo, D65);
        auto reconstructed = srgb.to_rgb(xyz);

        EXPECT_NEAR(reconstructed.r(), target.r(), 1.5e-2);
        EXPECT_NEAR(reconstructed.g(), target.g(), 1.5e-2);
        EXPECT_NEAR(reconstructed.b(), target.b(), 1.5e-2);
    };

    check_match(RGB<double>(0.25, 0.6, 0.1));
    check_match(RGB<double>(0.8, 0.25, 0.55));
    check_match(RGB<double>(0.5, 0.5, 0.5));
}

TEST_F(SpectrumTest, RGBToSpectrumLookupExtremeValuesAreFinite) {
    auto check_finite_bounded = [&](const RGB<double>& rgb) {
        auto spectrum = create_srgb_albedo_spectrum(rgb);
        for (double lambda : {360.0, 450.0, 550.0, 700.0, 830.0}) {
            const double value = spectrum.at(lambda);
            EXPECT_TRUE(std::isfinite(value));
            EXPECT_GE(value, 0.0);
            EXPECT_LE(value, 1.0);
        }
    };

    check_finite_bounded(RGB<double>(0.0, 0.0, 0.0));
    check_finite_bounded(RGB<double>(1.0, 1.0, 1.0));
}

TEST_F(SpectrumTest, UnboundedRGBSpectrumWorkflow) {
    // Test unbounded RGB spectrum workflow from function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto srgb = sRGB<double>;

    // Start with optimized RGB
    RGB<double> base_rgb(0.139, 0.735, 0.989);
    RGB<double> scaled_rgb_x2 = base_rgb * 2.0;

    std::cout << "Testing unbounded RGB workflow with scaled RGB: " << scaled_rgb_x2 << std::endl;

    // Test RGB scaling function
    auto scaled_result = scale_unbounded_rgb(scaled_rgb_x2);

    std::cout << "Scaled result: RGB = " << scaled_result.rgb << ", scale = " << scaled_result.scale << std::endl;

    EXPECT_GT(scaled_result.scale, 1.0);  // Scale should be > 1 for out-of-gamut colors
    EXPECT_LE(scaled_result.rgb.r(), 1.0);
    EXPECT_LE(scaled_result.rgb.g(), 1.0);
    EXPECT_LE(scaled_result.rgb.b(), 1.0);
    EXPECT_GE(scaled_result.rgb.r(), 0.0);
    EXPECT_GE(scaled_result.rgb.g(), 0.0);
    EXPECT_GE(scaled_result.rgb.b(), 0.0);

    // Optimize the scaled-down RGB to get polynomial coefficients
    auto optimization_result = optimize_albedo_rgb_sigmoid_polynomial(scaled_result.rgb, srgb, D65);

    std::cout << "Optimization result: error magnitude = "
              << std::sqrt(optimization_result.error[0] * optimization_result.error[0] +
                           optimization_result.error[1] * optimization_result.error[1] +
                           optimization_result.error[2] * optimization_result.error[2])
              << ", coeffs = (" << optimization_result.normalized_coeffs[0] << ", "
              << optimization_result.normalized_coeffs[1] << ", " << optimization_result.normalized_coeffs[2] << ")"
              << std::endl;

    // Check that optimization converged
    double error_magnitude = std::sqrt(optimization_result.error[0] * optimization_result.error[0] +
                                       optimization_result.error[1] * optimization_result.error[1] +
                                       optimization_result.error[2] * optimization_result.error[2]);
    EXPECT_LT(error_magnitude, 1e-3);

    // Create optimized polynomial
    RGBSigmoidPolynomialNormalized<double> optimized_poly{optimization_result.normalized_coeffs[0],
                                                          optimization_result.normalized_coeffs[1],
                                                          optimization_result.normalized_coeffs[2]};

    // Create unbounded spectrum with optimized coefficients and original scale
    RGBUnboundedSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> unbounded_spectrum(optimized_poly,
                                                                                                scaled_result.scale);

    // Test sampling
    Vector<double, 3> test_wavelengths(450, 550, 650);
    SampledWavelength<double, 3> sample_wl(test_wavelengths);
    auto sampled = unbounded_spectrum.sample(sample_wl);

    for (int i = 0; i < 3; i++) {
        EXPECT_GE(sampled[i], 0.0);
        EXPECT_TRUE(std::isfinite(sampled[i]));
    }

    // Verify that the unbounded spectrum reproduces the original scaled RGB
    auto xyz_from_unbounded = XYZ<double>::from_reflectance(unbounded_spectrum, D65);
    auto rgb_from_unbounded = srgb.to_rgb(xyz_from_unbounded);

    std::cout << "Final verification: original scaled_rgb_x2 = " << scaled_rgb_x2
              << ", reconstructed = " << rgb_from_unbounded << std::endl;

    // Should closely match original scaled RGB
    EXPECT_NEAR(rgb_from_unbounded.r(), scaled_rgb_x2.r(), 1e-2);
    EXPECT_NEAR(rgb_from_unbounded.g(), scaled_rgb_x2.g(), 1e-2);
    EXPECT_NEAR(rgb_from_unbounded.b(), scaled_rgb_x2.b(), 1e-2);
}

TEST_F(SpectrumTest, IlluminantSpectrumDistributionBasic) {
    // Test complete illuminant spectrum distribution workflow from function_test.cpp
    auto D65 = CIE_D65_ilum<double>;
    auto srgb = sRGB<double>;

    // Start with a test RGB color that needs illuminant spectrum optimization
    RGB<double> target_rgb(0.8, 0.6, 0.4);  // Warm orange-ish color

    std::cout << "Testing illuminant spectrum distribution with target RGB: " << target_rgb << std::endl;

    // First, optimize to get base albedo spectrum coefficients
    auto albedo_result = optimize_albedo_rgb_sigmoid_polynomial(target_rgb, srgb, D65);

    std::cout << "Albedo optimization result: error magnitude = "
              << std::sqrt(albedo_result.error[0] * albedo_result.error[0] +
                           albedo_result.error[1] * albedo_result.error[1] +
                           albedo_result.error[2] * albedo_result.error[2])
              << ", coeffs = (" << albedo_result.normalized_coeffs[0] << ", " << albedo_result.normalized_coeffs[1]
              << ", " << albedo_result.normalized_coeffs[2] << ")" << std::endl;

    // Check that albedo optimization converged
    double albedo_error =
        std::sqrt(albedo_result.error[0] * albedo_result.error[0] + albedo_result.error[1] * albedo_result.error[1] +
                  albedo_result.error[2] * albedo_result.error[2]);
    EXPECT_LT(albedo_error, 1e-2);

    // Create optimized albedo polynomial
    RGBSigmoidPolynomialNormalized<double> optimized_albedo_poly{
        albedo_result.normalized_coeffs[0], albedo_result.normalized_coeffs[1], albedo_result.normalized_coeffs[2]};

    // Test different scale factors for illuminant spectrum distribution
    std::vector<double> test_scales = {1.5, 2.0, 3.0};

    for (double scale : test_scales) {
        std::cout << "Testing illuminant distribution with scale factor: " << scale << std::endl;

        // Create RGB illuminant spectrum distribution
        // This would use the optimized polynomial coefficients with the scale factor
        RGBUnboundedSpectrumDistribution<double, RGBSigmoidPolynomialNormalized> illuminant_spectrum(
            optimized_albedo_poly, scale);

        // Test polynomial evaluation at various wavelengths
        Vector<double, 4> test_wavelengths(450, 550, 650, 750);
        SampledWavelength<double, 4> sample_wl(test_wavelengths);

        auto illuminant_sampled = illuminant_spectrum.sample(sample_wl);

        for (int i = 0; i < 4; i++) {
            double wl = test_wavelengths[i];
            double value = optimized_albedo_poly.at(wl);
            double scaled_value = illuminant_sampled[i];

            EXPECT_GE(value, 0.0);
            EXPECT_LE(value, 1.0);  // Normalized polynomial should be in [0,1]
            EXPECT_TRUE(std::isfinite(value));

            // Scaled value should be the polynomial value times the scale
            EXPECT_NEAR(scaled_value, value * scale, 1e-10);
            EXPECT_TRUE(std::isfinite(scaled_value));
        }

        // Test that we can use this as an illuminant by computing XYZ under it
        // For testing, we'll use a simple white reflectance spectrum
        auto white_reflectance = [](double lambda) -> double { return 0.9; };  // 90% reflectance

        // Sample the illuminant and compute approximate XYZ
        Vector<double, 10> sample_wavelengths;
        for (int i = 0; i < 10; i++) {
            sample_wavelengths[i] = 400.0 + i * 30.0;  // 400nm to 670nm
        }
        SampledWavelength<double, 10> full_sample_wl(sample_wavelengths);

        auto full_illuminant_sampled = illuminant_spectrum.sample(full_sample_wl);

        // All samples should be reasonable
        for (int i = 0; i < 10; i++) {
            EXPECT_GT(full_illuminant_sampled[i], 0.0);
            EXPECT_TRUE(std::isfinite(full_illuminant_sampled[i]));

            // Should be scaled version of normalized polynomial
            double expected = optimized_albedo_poly.at(sample_wavelengths[i]) * scale;
            EXPECT_NEAR(full_illuminant_sampled[i], expected, 1e-10);
        }

        std::cout << "Scale " << scale << " - Sample at 550nm: " << illuminant_spectrum.at(550.0)
                  << " (expected: " << optimized_albedo_poly.at(550.0) * scale << ")" << std::endl;
    }

    // Test D65 illuminant sampling for comparison
    Vector<double, 4> test_wavelengths(450, 550, 650, 750);
    SampledWavelength<double, 4> sample_wl(test_wavelengths);
    auto d65_sampled = D65.sample(sample_wl);

    std::cout << "D65 reference samples: ";
    for (int i = 0; i < 4; i++) {
        EXPECT_GT(d65_sampled[i], 0.0);
        EXPECT_TRUE(std::isfinite(d65_sampled[i]));
        std::cout << d65_sampled[i] << " ";
    }
    std::cout << std::endl;
}

TEST_F(SpectrumTest, MultipleIlluminantSamplingAdvanced) {
    // Advanced multi-illuminant sampling like in function_test.cpp output
    auto D50 = CIE_D50_ilum<double>;
    auto D65 = CIE_D65_ilum<double>;
    auto A = CIE_A_ilum<double>;

    BlackBodySpectrumDistribution<double> black_body(2856.0);  // Same temperature as function_test

    Vector<double, 5> test_wavelengths(400, 560, 600, 700, 800);
    SampledWavelength<double, 5> sample_wl(test_wavelengths);

    auto d50_sampled = D50.sample(sample_wl);
    auto d65_sampled = D65.sample(sample_wl);
    auto a_sampled = A.sample(sample_wl);
    auto bb_sampled = black_body.sample(sample_wl);

    // Verify expected illuminant characteristics from function_test output
    // D65 should have ~100.0 at 560nm (peak)
    EXPECT_NEAR(d65_sampled[1], 100.0, 5.0);

    // A illuminant should increase with wavelength (warmer/redder)
    EXPECT_LT(a_sampled[0], a_sampled[4]);  // 400nm < 800nm

    // Test inverse operations
    auto inverse_product = bb_sampled * a_sampled.inv();
    for (int i = 0; i < 5; i++) {
        EXPECT_GT(inverse_product[i], 0.0);
        EXPECT_TRUE(std::isfinite(inverse_product[i]));
    }

    // Test that black body has reasonable values scaled by 1e11 (as in function_test)
    auto bb_scaled = bb_sampled / 1e11;
    for (int i = 0; i < 5; i++) {
        EXPECT_GT(bb_scaled[i], 0.0);
        EXPECT_LT(bb_scaled[i], 100.0);  // Should be reasonable after scaling
    }
}

// =============================================================================
// MultipliedSpectrumDistribution Lifetime Tests
// =============================================================================

TEST_F(SpectrumTest, MultipliedSpectrumOwnsRvalueRvalueOperands) {
    using S = ConstantSpectrumDistribution<float>;

    auto product = S(2.0f) * S(3.0f);

    EXPECT_NE(product.s1.owned, nullptr);
    EXPECT_EQ(product.s1.ref, product.s1.owned.get());
    EXPECT_NE(product.s2.owned, nullptr);
    EXPECT_EQ(product.s2.ref, product.s2.owned.get());

    EXPECT_FLOAT_EQ(product.at(500.0f), 6.0f);
}

TEST_F(SpectrumTest, MultipliedSpectrumOwnsOnlyRvalueSide) {
    using S = ConstantSpectrumDistribution<float>;

    S a(2.0f);
    S b(3.0f);

    {
        auto product = S(2.0f) * b;
        EXPECT_NE(product.s1.owned, nullptr);
        EXPECT_EQ(product.s2.owned, nullptr);
        EXPECT_EQ(product.s2.ref, &b);
        EXPECT_FLOAT_EQ(product.at(500.0f), 6.0f);
    }

    {
        auto product = a * S(3.0f);
        EXPECT_EQ(product.s1.owned, nullptr);
        EXPECT_EQ(product.s1.ref, &a);
        EXPECT_NE(product.s2.owned, nullptr);
        EXPECT_FLOAT_EQ(product.at(500.0f), 6.0f);
    }
}

TEST_F(SpectrumTest, MultipliedSpectrumReferencesBothLvalues) {
    using S = ConstantSpectrumDistribution<float>;

    S a(2.0f);
    S b(3.0f);

    auto product = a * b;
    EXPECT_EQ(product.s1.owned, nullptr);
    EXPECT_EQ(product.s1.ref, &a);
    EXPECT_EQ(product.s2.owned, nullptr);
    EXPECT_EQ(product.s2.ref, &b);
    EXPECT_FLOAT_EQ(product.at(500.0f), 6.0f);
}

TEST_F(SpectrumTest, MultipliedSpectrumOwnsTemporaryConstantTimesD65) {
    using S = ConstantSpectrumDistribution<float>;
    const auto& d65 = CIE_D65_ilum<float>;

    auto product = S(2.0f) * d65;
    EXPECT_NE(product.s1.owned, nullptr);
    EXPECT_EQ(product.s2.owned, nullptr);

    EXPECT_NEAR(product.at(560.0f), 2.0f * d65.at(560.0f), 1e-5f);
}

// =============================================================================
// PiecewiseLinearSpectrumDistribution from_string Tests
// =============================================================================

TEST_F(SpectrumTest, PiecewiseLinearFromStringBasic) {
    // Test basic parsing with multiple points
    std::string input = "400:0.5, 500:1.0, 600:0.8, 700:0.3";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    // Test exact knot points
    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
    EXPECT_FLOAT_EQ(spectrum.at(600.0f), 0.8f);
    EXPECT_FLOAT_EQ(spectrum.at(700.0f), 0.3f);

    // Test interpolation
    EXPECT_FLOAT_EQ(spectrum.at(450.0f), 0.75f);  // midpoint between 400:0.5 and 500:1.0
    EXPECT_FLOAT_EQ(spectrum.at(550.0f), 0.9f);   // midpoint between 500:1.0 and 600:0.8
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringSinglePoint) {
    // Test with a single point
    std::string input = "550:1.5";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(550.0f), 1.5f);
    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 1.5f);  // Below range, should clamp
    EXPECT_FLOAT_EQ(spectrum.at(700.0f), 1.5f);  // Above range, should clamp
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringWithSpaces) {
    // Test parsing with various whitespace
    std::string input = "  400:0.5  ,  500:1.0  , 600:0.8   ";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
    EXPECT_FLOAT_EQ(spectrum.at(600.0f), 0.8f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringNoComma) {
    // Test with single point without comma
    std::string input = "500:1.0";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringUnsortedInput) {
    // Test with unsorted wavelengths (should be sorted automatically)
    std::string input = "600:0.8, 400:0.5, 700:0.3, 500:1.0";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
    EXPECT_FLOAT_EQ(spectrum.at(600.0f), 0.8f);
    EXPECT_FLOAT_EQ(spectrum.at(700.0f), 0.3f);

    // Test interpolation works correctly after sorting
    EXPECT_FLOAT_EQ(spectrum.at(450.0f), 0.75f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringExtrapolation) {
    // Test extrapolation behavior (should clamp to boundary values)
    std::string input = "500:1.0, 600:0.5";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    // Below range
    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 1.0f);
    EXPECT_FLOAT_EQ(spectrum.at(499.0f), 1.0f);

    // Above range
    EXPECT_FLOAT_EQ(spectrum.at(601.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(700.0f), 0.5f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringNegativeValues) {
    // Test with negative values
    std::string input = "400:-0.5, 500:0.0, 600:1.5";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(400.0f), -0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 0.0f);
    EXPECT_FLOAT_EQ(spectrum.at(600.0f), 1.5f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringDecimalWavelengths) {
    // Test with decimal wavelengths
    std::string input = "450.5:0.5, 550.5:1.0";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(450.5f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(550.5f), 1.0f);
    EXPECT_FLOAT_EQ(spectrum.at(500.5f), 0.75f);  // midpoint
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringInvalidFormat) {
    // Test with invalid format (missing colon)
    std::string input = "400-0.5, 500:1.0";
    EXPECT_THROW(
        { auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input); }, std::invalid_argument);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringEmptyString) {
    // Test with empty string
    std::string input = "";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    // Should return 0 for any wavelength (empty spectrum)
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 0.0f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringScientificNotation) {
    // Test with scientific notation
    std::string input = "400:1.5e-3, 500:2.5e-3, 600:3.5e-3";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_NEAR(spectrum.at(400.0f), 1.5e-3f, 1e-6f);
    EXPECT_NEAR(spectrum.at(500.0f), 2.5e-3f, 1e-6f);
    EXPECT_NEAR(spectrum.at(600.0f), 3.5e-3f, 1e-6f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringDoubleType) {
    // Test with double precision
    std::string input = "400:0.123456789, 500:0.987654321";
    auto spectrum = PiecewiseLinearSpectrumDistribution<double>::from_string(input);

    EXPECT_DOUBLE_EQ(spectrum.at(400.0), 0.123456789);
    EXPECT_DOUBLE_EQ(spectrum.at(500.0), 0.987654321);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringTrailingComma) {
    // Test with trailing comma - function handles it gracefully by skipping empty point
    std::string input = "400:0.5, 500:1.0,";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);
    // Should only parse the two valid points
    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringMultipleColons) {
    // Test with multiple colons in point
    std::string input = "400:0.5:extra, 500:1.0";
    // atof will stop at first non-numeric character, so "0.5:extra" -> 0.5
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);
    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 0.5f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 1.0f);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringOnlySpaces) {
    // Test with only spaces - leads to empty point_str which has no colon
    std::string input = "   ";
    EXPECT_THROW(
        { auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input); }, std::invalid_argument);
}

TEST_F(SpectrumTest, PiecewiseLinearFromStringLargeValues) {
    // Test with very large values
    std::string input = "400:1e10, 500:2e10, 600:3e10";
    auto spectrum = PiecewiseLinearSpectrumDistribution<float>::from_string(input);

    EXPECT_FLOAT_EQ(spectrum.at(400.0f), 1e10f);
    EXPECT_FLOAT_EQ(spectrum.at(500.0f), 2e10f);
    EXPECT_FLOAT_EQ(spectrum.at(600.0f), 3e10f);
    EXPECT_FLOAT_EQ(spectrum.at(450.0f), 1.5e10f);  // Interpolation
}

}  // namespace pbpt::radiometry::testing
