#pragma once

#include <gtest/gtest.h>
#include <random>
#include <chrono>
#include "matrix.hpp"
#include "vector.hpp"
#include "homogeneous.hpp"

namespace pbpt::math::test {

class MatrixTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up random number generator with fixed seed for reproducible tests
        rng.seed(42);
    }

    std::mt19937 rng;
    
    template<typename T>
    T random_value(T min = T(-10), T max = T(10)) {
        if constexpr (std::is_integral_v<T>) {
            std::uniform_int_distribution<T> dist(min, max);
            return dist(rng);
        } else {
            std::uniform_real_distribution<T> dist(min, max);
            return dist(rng);
        }
    }
    
    template<typename T, int R, int C>
    Matrix<T, R, C> create_random_matrix() {
        Matrix<T, R, C> result;
        result.visit([this](T& val, int, int) {
            val = random_value<T>();
        });
        return result;
    }
    
    template<typename T, int N>
    Vector<T, N> create_random_vector() {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i) {
            result[i] = random_value<T>();
        }
        return result;
    }
    
    template<typename T, int N>
    Homogeneous<T, N> create_random_homogeneous() {
        Vector<T, N> v = create_random_vector<T, N>();
        T w = random_value<T>(T(0.1), T(2)); // Avoid zero weight
        auto result = Homogeneous<T, N>::from_vector(v);
        result.w() = w;
        return result;
    }
};

// Test 1x1 matrices
TEST_F(MatrixTest, Matrix1x1BasicOperations) {
    using Mat1x1 = Matrix<float, 1, 1>;
    
    // Construction
    Mat1x1 m1(5.0f);
    Mat1x1 m2(3.0f);
    Mat1x1 zero = Mat1x1::zeros();
    Mat1x1 one = Mat1x1::ones();
    Mat1x1 identity = Mat1x1::identity();
    
    EXPECT_FLOAT_EQ(m1.at(0, 0), 5.0f);
    EXPECT_FLOAT_EQ(m2.at(0, 0), 3.0f);
    EXPECT_FLOAT_EQ(zero.at(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(one.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(identity.at(0, 0), 1.0f);
    
    // Arithmetic operations
    Mat1x1 sum = m1 + m2;
    EXPECT_FLOAT_EQ(sum.at(0, 0), 8.0f);
    
    Mat1x1 diff = m1 - m2;
    EXPECT_FLOAT_EQ(diff.at(0, 0), 2.0f);
    
    Mat1x1 scaled = m1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled.at(0, 0), 10.0f);
    
    Mat1x1 divided = m1 / 2.0f;
    EXPECT_FLOAT_EQ(divided.at(0, 0), 2.5f);
    
    // Matrix multiplication
    Mat1x1 product = m1 * m2;
    EXPECT_FLOAT_EQ(product.at(0, 0), 15.0f);
    
    // Determinant and inverse
    EXPECT_FLOAT_EQ(m1.determinant(), 5.0f);
    Mat1x1 inv = m1.inversed();
    EXPECT_FLOAT_EQ(inv.at(0, 0), 0.2f);
    
    // Transpose
    Mat1x1 transposed = m1.transposed();
    EXPECT_FLOAT_EQ(transposed.at(0, 0), 5.0f);
}

// Test 1xM matrices (row vectors)
TEST_F(MatrixTest, Matrix1xMOperations) {
    using Mat1x3 = Matrix<float, 1, 3>;
    using Mat1x4 = Matrix<float, 1, 4>;
    using Mat1x5 = Matrix<float, 1, 5>;
    
    // Construction
    Mat1x3 m1(1.0f, 2.0f, 3.0f);
    Mat1x4 m2(4.0f, 5.0f, 6.0f, 7.0f);
    Mat1x5 zero = Mat1x5::zeros();
    Mat1x5 ones = Mat1x5::ones();
    
    // Basic access
    EXPECT_FLOAT_EQ(m1.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m1.at(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(m1.at(0, 2), 3.0f);
    
    // Row access
    auto row = m1.row(0);
    EXPECT_FLOAT_EQ(row[0], 1.0f);
    EXPECT_FLOAT_EQ(row[1], 2.0f);
    EXPECT_FLOAT_EQ(row[2], 3.0f);
    
    // Arithmetic operations
    Mat1x3 m1_copy(1.0f, 2.0f, 3.0f);
    Mat1x3 sum = m1 + m1_copy;
    EXPECT_FLOAT_EQ(sum.at(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(sum.at(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(sum.at(0, 2), 6.0f);
    
    // Scalar multiplication
    Mat1x3 scaled = m1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled.at(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(scaled.at(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(scaled.at(0, 2), 6.0f);
    
    // Transpose to column vector
    auto transposed = m1.transposed();
    static_assert(std::is_same_v<decltype(transposed), Matrix<float, 3, 1>>);
    EXPECT_FLOAT_EQ(transposed.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(transposed.at(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(transposed.at(2, 0), 3.0f);
}

// Test Nx1 matrices (column vectors)
TEST_F(MatrixTest, MatrixNx1Operations) {
    using Mat3x1 = Matrix<float, 3, 1>;
    using Mat4x1 = Matrix<float, 4, 1>;
    using Mat5x1 = Matrix<float, 5, 1>;
    
    // Construction
    Mat3x1 m1(1.0f, 2.0f, 3.0f);
    Mat4x1 m2(4.0f, 5.0f, 6.0f, 7.0f);
    Mat5x1 zero = Mat5x1::zeros();
    Mat5x1 ones = Mat5x1::ones();
    
    // Basic access
    EXPECT_FLOAT_EQ(m1.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m1.at(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(m1.at(2, 0), 3.0f);
    
    // Column access
    auto col = m1.col(0);
    EXPECT_FLOAT_EQ(col[0], 1.0f);
    EXPECT_FLOAT_EQ(col[1], 2.0f);
    EXPECT_FLOAT_EQ(col[2], 3.0f);
    
    // Arithmetic operations
    Mat3x1 m1_copy(1.0f, 2.0f, 3.0f);
    Mat3x1 sum = m1 + m1_copy;
    EXPECT_FLOAT_EQ(sum.at(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(sum.at(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(sum.at(2, 0), 6.0f);
    
    // Scalar multiplication
    Mat3x1 scaled = m1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled.at(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(scaled.at(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(scaled.at(2, 0), 6.0f);
    
    // Transpose to row vector
    auto transposed = m1.transposed();
    static_assert(std::is_same_v<decltype(transposed), Matrix<float, 1, 3>>);
    EXPECT_FLOAT_EQ(transposed.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(transposed.at(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(transposed.at(0, 2), 3.0f);
}

// Test matrix-vector multiplication with 1xM and Nx1 matrices
TEST_F(MatrixTest, Matrix1xMAndNx1Multiplication) {
    using Mat1x3 = Matrix<float, 1, 3>;
    using Mat3x1 = Matrix<float, 3, 1>;
    using Mat3x3 = Matrix<float, 3, 3>;
    using Vec3 = Vector<float, 3>;
    
    Mat1x3 row_mat(1.0f, 2.0f, 3.0f);
    Mat3x1 col_mat(4.0f, 5.0f, 6.0f);
    Vec3 vec(4.0f, 5.0f, 6.0f);
    
    // Row matrix * column matrix = 1x1 matrix (scalar)
    auto scalar_result = row_mat * col_mat;
    static_assert(std::is_same_v<decltype(scalar_result), Matrix<float, 1, 1>>);
    EXPECT_FLOAT_EQ(scalar_result.at(0, 0), 32.0f); // 1*4 + 2*5 + 3*6 = 32
    
    // Row matrix * vector = 1D vector (scalar)
    auto vec_result = row_mat * vec;
    static_assert(std::is_same_v<decltype(vec_result), Vector<float, 1>>);
    EXPECT_FLOAT_EQ(vec_result[0], 32.0f);
    
    // Column matrix * row matrix = 3x3 matrix (outer product)
    auto outer_product = col_mat * row_mat;
    static_assert(std::is_same_v<decltype(outer_product), Matrix<float, 3, 3>>);
    EXPECT_FLOAT_EQ(outer_product.at(0, 0), 4.0f);  // 4*1
    EXPECT_FLOAT_EQ(outer_product.at(0, 1), 8.0f);  // 4*2
    EXPECT_FLOAT_EQ(outer_product.at(0, 2), 12.0f); // 4*3
    EXPECT_FLOAT_EQ(outer_product.at(1, 0), 5.0f);  // 5*1
    EXPECT_FLOAT_EQ(outer_product.at(2, 2), 18.0f); // 6*3
}

// Random matrix tests
TEST_F(MatrixTest, RandomMatrixOperations) {
    const int num_tests = 100;
    
    for (int test = 0; test < num_tests; ++test) {
        // Test random 3x3 matrices
        auto m1 = create_random_matrix<float, 3, 3>();
        auto m2 = create_random_matrix<float, 3, 3>();
        
        // Test commutativity of addition
        auto sum1 = m1 + m2;
        auto sum2 = m2 + m1;
        EXPECT_TRUE(sum1 == sum2);
        
        // Test associativity of addition
        auto m3 = create_random_matrix<float, 3, 3>();
        auto assoc1 = (m1 + m2) + m3;
        auto assoc2 = m1 + (m2 + m3);
        EXPECT_TRUE(assoc1 == assoc2);
        
        // Test distributivity of scalar multiplication
        float scalar = random_value<float>();
        auto dist1 = scalar * (m1 + m2);
        auto dist2 = scalar * m1 + scalar * m2;
        EXPECT_TRUE(dist1 == dist2);
        
        // Test matrix multiplication associativity (when determinant is non-zero)
        if (std::abs(m1.determinant()) > 1e-6f) {
            auto assoc_mult1 = (m1 * m2) * m3;
            auto assoc_mult2 = m1 * (m2 * m3);
            EXPECT_TRUE(assoc_mult1 == assoc_mult2);
        }
    }
}

// Random vector tests
TEST_F(MatrixTest, RandomVectorOperations) {
    const int num_tests = 100;
    
    for (int test = 0; test < num_tests; ++test) {
        auto v1 = create_random_vector<float, 3>();
        auto v2 = create_random_vector<float, 3>();
        auto v3 = create_random_vector<float, 3>();
        
        // Test vector addition commutativity
        auto sum1 = v1 + v2;
        auto sum2 = v2 + v1;
        EXPECT_TRUE(sum1 == sum2);
        
        // Test vector addition associativity
        auto assoc1 = (v1 + v2) + v3;
        auto assoc2 = v1 + (v2 + v3);
        EXPECT_TRUE(assoc1 == assoc2);
        
        // Test scalar multiplication distributivity
        float scalar = random_value<float>();
        auto dist1 = scalar * (v1 + v2);
        auto dist2 = scalar * v1 + scalar * v2;
        EXPECT_TRUE(dist1 == dist2);
        
        // Test dot product commutativity
        float dot1 = v1.dot(v2);
        float dot2 = v2.dot(v1);
        EXPECT_FLOAT_EQ(dot1, dot2);
        
        // Test dot product bilinearity
        float a = random_value<float>();
        float b = random_value<float>();
        auto linear_combo = a * v1 + b * v2;
        float dot_linear = linear_combo.dot(v3);
        float dot_bilinear = a * v1.dot(v3) + b * v2.dot(v3);
        EXPECT_NEAR(dot_linear, dot_bilinear, 1e-5f);
    }
}

// Random homogeneous point tests
TEST_F(MatrixTest, RandomHomogeneousOperations) {
    const int num_tests = 50;
    
    for (int test = 0; test < num_tests; ++test) {
        auto h1 = create_random_homogeneous<float, 3>();
        auto h2 = create_random_homogeneous<float, 3>();
        
        // Test conversion to vector and back
        auto v1 = h1.to_vector();
        auto h1_reconstructed = Homogeneous<float, 3>::from_vector(v1);
        h1_reconstructed.w() = h1.w();
        EXPECT_TRUE(h1 == h1_reconstructed);
        
        // Test homogeneous addition
        auto sum = h1 + h2;
        auto v_sum = h1.to_vector() + h2.to_vector();
        EXPECT_TRUE(sum.to_vector() == v_sum);
        
        // Test scalar multiplication
        float scalar = random_value<float>(0.1f, 5.0f);
        auto scaled = scalar * h1;
        auto v_scaled = scalar * h1.to_vector();
        EXPECT_TRUE(scaled.to_vector() == v_scaled);
        
        // Test normalization
        if (std::abs(h1.w()) > 1e-6f) {
            auto standardized = h1.standardized();
            EXPECT_FLOAT_EQ(standardized.w(), 1.0f);
            
            // Check that the 3D point remains the same after standardization
            auto original_point = h1.to_point();
            auto standardized_point = standardized.to_point();
            EXPECT_TRUE(original_point == standardized_point);
        }
    }
}

// Random matrix-vector multiplication tests
TEST_F(MatrixTest, RandomMatrixVectorMultiplication) {
    const int num_tests = 100;
    
    for (int test = 0; test < num_tests; ++test) {
        auto m = create_random_matrix<float, 4, 4>();
        auto v = create_random_vector<float, 4>();
        auto h = create_random_homogeneous<float, 3>();
        
        // Test matrix-vector multiplication
        auto result_v = m * v;
        static_assert(std::is_same_v<decltype(result_v), Vector<float, 4>>);
        
        // Manually compute and verify
        for (int i = 0; i < 4; ++i) {
            float expected = 0.0f;
            for (int j = 0; j < 4; ++j) {
                expected += m.at(i, j) * v[j];
            }
            EXPECT_NEAR(result_v[i], expected, 1e-5f);
        }
        
        // Test matrix-homogeneous multiplication
        auto result_h = m * h;
        static_assert(std::is_same_v<decltype(result_h), Homogeneous<float, 3>>);
        
        // Verify by converting homogeneous to vector and multiplying
        auto h_as_vector = h.to_vector_raw();
        auto expected_vector = m * h_as_vector;
        EXPECT_TRUE(result_h.to_vector_raw() == expected_vector);
    }
}

// Performance test for random matrices
TEST_F(MatrixTest, RandomMatrixPerformance) {
    const int num_operations = 1000;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_operations; ++i) {
        auto m1 = create_random_matrix<float, 4, 4>();
        auto m2 = create_random_matrix<float, 4, 4>();
        
        // Perform various operations
        auto sum = m1 + m2;
        auto product = m1 * m2;
        auto transposed = m1.transposed();
        
        if (std::abs(m1.determinant()) > 1e-6f) {
            auto inverse = m1.inversed();
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Just ensure it completes in reasonable time (< 5 seconds)
    EXPECT_LT(duration.count(), 5000);
}

// Test random matrix properties
TEST_F(MatrixTest, RandomMatrixProperties) {
    const int num_tests = 50;
    
    for (int test = 0; test < num_tests; ++test) {
        auto m = create_random_matrix<double, 3, 3>();
        
        // Test that transpose of transpose is original
        auto double_transposed = m.transposed().transposed();
        EXPECT_TRUE(m == double_transposed);
        
        // Test determinant of transpose equals determinant of original
        double det_original = m.determinant();
        double det_transposed = m.transposed().determinant();
        EXPECT_NEAR(det_original, det_transposed, 1e-10);
        
        // Test inverse properties (if matrix is invertible)
        if (std::abs(det_original) > 1e-10) {
            auto inverse = m.inversed();
            auto identity_check = m * inverse;
            
            // Check if it's close to identity
            EXPECT_TRUE(identity_check.is_identity() || 
                       (identity_check - Matrix<double, 3, 3>::identity()).is_all_zero());
        }
    }
}

// Test edge cases with random values
TEST_F(MatrixTest, RandomMatrixEdgeCases) {
    // Test with very small values
    for (int test = 0; test < 10; ++test) {
        auto m = create_random_matrix<float, 2, 2>();
        m *= 1e-7f; // Make values very small
        
        auto sum = m + m;
        auto scaled = 2.0f * m;
        EXPECT_TRUE(sum == scaled);
    }
    
    // Test with very large values
    for (int test = 0; test < 10; ++test) {
        auto m = create_random_matrix<float, 2, 2>();
        m *= 1e6f; // Make values very large
        
        auto diff = m - m;
        EXPECT_TRUE(diff.is_all_zero());
    }
    
    // Test with mixed positive and negative values
    for (int test = 0; test < 10; ++test) {
        auto m = create_random_matrix<float, 3, 3>();
        auto negated = -m;
        auto sum = m + negated;
        EXPECT_TRUE(sum.is_all_zero());
    }
}

} // namespace pbpt::math::test
