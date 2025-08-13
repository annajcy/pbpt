/**
 * @file test_octahedral.cpp
 * @brief Unit tests for the OctahedralVector template class
 * 
 * This file contains tests for the OctahedralVector class template,
 * focusing on basic functionality, storage compression, and reasonable precision
 * for the octahedral mapping compression technique.
 */

#include <gtest/gtest.h>

#include <type_traits>
#include <random>

#include "math/geometry/octahedral.hpp"

namespace pbpt::math::testing {

// Test fixture for OctahedralVector tests
class OctahedralVectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup standard test vectors
        unit_x = Vector<Float, 3>{1.0, 0.0, 0.0};
        unit_y = Vector<Float, 3>{0.0, 1.0, 0.0};
        unit_z = Vector<Float, 3>{0.0, 0.0, 1.0};
        unit_neg_x = Vector<Float, 3>{-1.0, 0.0, 0.0};
        unit_neg_y = Vector<Float, 3>{0.0, -1.0, 0.0};
        unit_neg_z = Vector<Float, 3>{0.0, 0.0, -1.0};
        
        // Normalized diagonal vectors
        diagonal_positive = Vector<Float, 3>{1.0, 1.0, 1.0}.normalized();
        diagonal_negative = Vector<Float, 3>{-1.0, -1.0, -1.0}.normalized();
    }
    
    // Standard unit vectors
    Vector<Float, 3> unit_x, unit_y, unit_z;
    Vector<Float, 3> unit_neg_x, unit_neg_y, unit_neg_z;
    
    // Diagonal test vectors
    Vector<Float, 3> diagonal_positive, diagonal_negative;
    
    // Helper function to generate random unit vector
    Vector<Float, 3> random_unit_vector(std::mt19937& gen) {
        std::uniform_real_distribution<Float> dist(-1.0, 1.0);
        Vector<Float, 3> v;
        do {
            v = Vector<Float, 3>{dist(gen), dist(gen), dist(gen)};
        } while (v.is_all_zero() || v.length_squared() > 1.0);
        return v.normalized();
    }
};


// Test basic encoding/decoding works
TEST_F(OctahedralVectorTest, BasicEncodeDecode) {
    std::vector<Vector<Float, 3>> test_vectors = {
        unit_x, unit_y, unit_z,
        unit_neg_x, unit_neg_y, unit_neg_z,
        diagonal_positive, diagonal_negative
    };
    
    for (const auto& original : test_vectors) {
        OctahedralVector<Float> ov(original);
        Vector<Float, 3> reconstructed = ov.decode();
        
        // Check that the reconstructed vector is normalized
        EXPECT_TRUE(reconstructed.is_normalized()) 
            << "Reconstructed vector should be normalized for input: " 
            << original;

        std::cout << "Original: " << original << ", Reconstructed: " << reconstructed << std::endl;

        EXPECT_NEAR(reconstructed.x(), original.x(), 1e-4) << "Reconstructed x should match original x for input: " << original;
        EXPECT_NEAR(reconstructed.y(), original.y(), 1e-4) << "Reconstructed y should match original y for input: " << original;
        EXPECT_NEAR(reconstructed.z(), original.z(), 1e-4) << "Reconstructed z should match original z for input: " << original;
    }
}

// Test that different vectors produce different encoded values
TEST_F(OctahedralVectorTest, DifferentInputsProduceDifferentEncodings) {
    OctahedralVector<Float> ov_x(unit_x);
    OctahedralVector<Float> ov_y(unit_y);
    OctahedralVector<Float> ov_z(unit_z);
    
    Vector<Float, 3> decoded_x = ov_x.decode();
    Vector<Float, 3> decoded_y = ov_y.decode();
    Vector<Float, 3> decoded_z = ov_z.decode();
    
    // These should not be the same (basic sanity check)
    EXPECT_FALSE((decoded_x - decoded_y).is_all_zero());
    EXPECT_FALSE((decoded_x - decoded_z).is_all_zero());
    EXPECT_FALSE((decoded_y - decoded_z).is_all_zero());
}

// Test with non-unit vectors (should be normalized internally)
TEST_F(OctahedralVectorTest, NonUnitVectorHandling) {
    Vector<Float, 3> long_vector{10.0, 20.0, 30.0};
    Vector<Float, 3> expected_direction = long_vector.normalized();
    
    OctahedralVector<Float> ov(long_vector);
    Vector<Float, 3> reconstructed = ov.decode();
    
    EXPECT_TRUE(reconstructed.is_normalized());
    
    EXPECT_NEAR(reconstructed.x(), expected_direction.x(), 1e-4);
    EXPECT_NEAR(reconstructed.y(), expected_direction.y(), 1e-4);
    EXPECT_NEAR(reconstructed.z(), expected_direction.z(), 1e-4);
}

// Test storage size - this is a key feature of octahedral vectors
TEST_F(OctahedralVectorTest, StorageSize) {
    OctahedralVector<Float> ov(unit_x);
    
    // Should be exactly 4 bytes (2 * uint16_t)
    EXPECT_EQ(sizeof(ov), 4);
    
    // Compare to uncompressed storage
    Vector<Float, 3> uncompressed_vector(unit_x);
    EXPECT_GT(sizeof(uncompressed_vector), sizeof(ov));
}

// Test that encoding/decoding preserves some basic properties
TEST_F(OctahedralVectorTest, BasicProperties) {
    std::vector<Vector<Float, 3>> test_vectors = {
        unit_x, unit_y, unit_z, diagonal_positive
    };
    
    for (const auto& original : test_vectors) {
        OctahedralVector<Float> ov(original);
        Vector<Float, 3> reconstructed = ov.decode();
        
        // Should always be normalized
        EXPECT_TRUE(reconstructed.is_normalized());
        EXPECT_NEAR(reconstructed.x(), original.x(), 1e-4);
        EXPECT_NEAR(reconstructed.y(), original.y(), 1e-4);
        EXPECT_NEAR(reconstructed.z(), original.z(), 1e-4);
    }
}

// Test with some random vectors (basic sanity check)
TEST_F(OctahedralVectorTest, RandomVectorSanityCheck) {
    std::mt19937 gen(42);  // Fixed seed for reproducible tests
    const int num_tests = 100;  // Reduced number for basic sanity
    
    for (int i = 0; i < num_tests; ++i) {
        Vector<Float, 3> original = random_unit_vector(gen);
        
        OctahedralVector<Float> ov(original);
        Vector<Float, 3> reconstructed = ov.decode();
        
        // Check that the reconstructed vector is normalized
        EXPECT_TRUE(reconstructed.is_normalized());
        EXPECT_NEAR(reconstructed.x(), original.x(), 1e-4);
        EXPECT_NEAR(reconstructed.y(), original.y(), 1e-4);
        EXPECT_NEAR(reconstructed.z(), original.z(), 1e-4);
    }
}

// Test different floating point types
TEST(OctahedralVectorTypesTest, FloatTypes) {
    Vector<float, 3> v_float{1.0f, 0.0f, 0.0f};
    Vector<double, 3> v_double{0.0, 1.0, 0.0};
    
    // Test with float
    OctahedralVector<float> ov_float(v_float);
    auto decoded_float = ov_float.decode();
    EXPECT_TRUE(decoded_float.is_normalized());
    
    // Test with double
    OctahedralVector<double> ov_double(v_double);
    auto decoded_double = ov_double.decode();
    EXPECT_TRUE(decoded_double.is_normalized());
    
    // Verify template instantiation works
    static_assert(std::is_same_v<decltype(decoded_float), Vector<float, 3>>);
    static_assert(std::is_same_v<decltype(decoded_double), Vector<double, 3>>);
}

// Test consistency - same input should produce same output
TEST_F(OctahedralVectorTest, Consistency) {
    Vector<Float, 3> test_vector = diagonal_positive;
    
    OctahedralVector<Float> ov1(test_vector);
    OctahedralVector<Float> ov2(test_vector);
    
    Vector<Float, 3> decoded1 = ov1.decode();
    Vector<Float, 3> decoded2 = ov2.decode();
    
    // Same input should produce same output
    Float diff = (decoded1 - decoded2).length();
    EXPECT_LT(diff, 1e-6f) << "Same input should produce same output";
}

// Test that the implementation can handle edge cases without crashing
TEST_F(OctahedralVectorTest, EdgeCasesNoCrash) {
    std::vector<Vector<Float, 3>> edge_cases = {
        Vector<Float, 3>{1e-6f, 0.0f, 0.0f}.normalized(),  // Very small components
        Vector<Float, 3>{0.0f, 0.0f, 1.0f},                // Pure z-axis
        Vector<Float, 3>{0.0f, 0.0f, -1.0f},               // Pure negative z-axis
        Vector<Float, 3>{0.707f, 0.707f, 0.0f}.normalized() // In xy-plane
    };
    
    for (size_t i = 0; i < edge_cases.size(); ++i) {
        const auto& test_vec = edge_cases[i];
        
        // Should not crash and should produce valid output
        OctahedralVector<Float> ov(test_vec);
        Vector<Float, 3> decoded = ov.decode();
        EXPECT_TRUE(decoded.is_normalized()) << "Edge case " << i;

        EXPECT_NEAR(decoded.x(), test_vec.x(), 1e-4) << "Edge case " << i;
        EXPECT_NEAR(decoded.y(), test_vec.y(), 1e-4) << "Edge case " << i;
        EXPECT_NEAR(decoded.z(), test_vec.z(), 1e-4) << "Edge case " << i;
    }
}

}  // namespace pbpt::math::testing
