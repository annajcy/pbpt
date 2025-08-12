#include "gtest/gtest.h"
#include "math/geometry/matrix.hpp"
#include "math/geometry/vector.hpp"
#include <chrono>

namespace pbpt::math::testing {

class MatrixPerformanceTest : public ::testing::Test {
protected:
    template<typename T, int N>
    void BenchmarkInverse(const Matrix<T, N, N>& matrix, const std::string& name) {
        const int iterations = 10000;
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations; ++i) {
            volatile auto temp = matrix.inversed();  // volatile to prevent optimization
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double avg_time = static_cast<double>(duration.count()) / iterations;
        
        std::cout << name << ": " << avg_time << " μs per inversion" << std::endl;
        
        // Performance should be reasonable for hardcoded methods
        if constexpr (N == 2) {
            EXPECT_LT(avg_time, 0.2);  // 2x2 should be very fast
        } else if constexpr (N == 3) {
            EXPECT_LT(avg_time, 0.5);  // 3x3 should be fast
        } else if constexpr (N == 4) {
            EXPECT_LT(avg_time, 2.1);  // 4x4 should be reasonable
        }
    }
};

TEST_F(MatrixPerformanceTest, HardcodedInversePerformance) {
    std::cout << "\n=== Matrix Inverse Performance Benchmark ===" << std::endl;
    
    // 2x2 Matrix
    Matrix<double, 2, 2> m2(
        Vector<double, 2>(4, 3), 
        Vector<double, 2>(2, 1)
    );
    BenchmarkInverse(m2, "2×2 Matrix");
    
    // 3x3 Matrix  
    Matrix<double, 3, 3> m3(
        Vector<double, 3>(2, 1, 0), 
        Vector<double, 3>(1, 2, 1), 
        Vector<double, 3>(0, 1, 2)
    );
    BenchmarkInverse(m3, "3×3 Matrix");
    
    // 4x4 Matrix
    Matrix<double, 4, 4> m4(
        Vector<double, 4>(2, 0, 0, 1), 
        Vector<double, 4>(0, 2, 0, 2), 
        Vector<double, 4>(0, 0, 2, 3),
        Vector<double, 4>(0, 0, 0, 1)
    );
    BenchmarkInverse(m4, "4×4 Matrix");
    
    std::cout << "=============================================" << std::endl;
}

TEST_F(MatrixPerformanceTest, ComparisonOperatorPerformance) {
    const int iterations = 100000;
    Matrix<float, 3, 3> m1 = Matrix<float, 3, 3>::identity();
    Matrix<float, 3, 3> m2 = Matrix<float, 3, 3>::identity();
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
        volatile bool result = (m1 == m2);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double avg_time = static_cast<double>(duration.count()) / iterations;
    
    std::cout << "\nComparison operator: " << avg_time << " μs per comparison" << std::endl;
    
    // Should be fast with std::ranges::equal
    EXPECT_LT(avg_time, 0.2);
}

}  // namespace pbpt::math::testing
