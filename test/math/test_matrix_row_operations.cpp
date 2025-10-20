#include "gtest/gtest.h"
#include "math/matrix.hpp"
#include "math/format.hpp"

namespace pbpt::math::testing {

class MatrixRowOperationsTest : public ::testing::Test {
protected:
    template <typename T, int R, int C>
    void ExpectMatricesNear(const Matrix<T, R, C>& m1, const Matrix<T, R, C>& m2, T tolerance = T(1e-10)) {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                EXPECT_NEAR(m1.at(r, c), m2.at(r, c), tolerance)
                    << "Mismatch at (" << r << ", " << c << ")";
            }
        }
    }
};

// ===== Elementary Row Operations Tests =====

TEST_F(MatrixRowOperationsTest, SwapRows) {
    Matrix<double, 3, 3> m(
        1, 2, 3, 
        4, 5, 6, 
        7, 8, 9
    );
    
    m.swap_rows(0, 2);
    
    EXPECT_DOUBLE_EQ(m.at(0, 0), 7);
    EXPECT_DOUBLE_EQ(m.at(0, 1), 8);
    EXPECT_DOUBLE_EQ(m.at(0, 2), 9);
    EXPECT_DOUBLE_EQ(m.at(2, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(2, 1), 2);
    EXPECT_DOUBLE_EQ(m.at(2, 2), 3);
    
    // Test out of range
    EXPECT_THROW(m.swap_rows(-1, 0), std::out_of_range);
    EXPECT_THROW(m.swap_rows(0, 5), std::out_of_range);
}

TEST_F(MatrixRowOperationsTest, ScaleRow) {
    Matrix<double, 3, 3> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    m.scale_row(1, 2.0);
    
    EXPECT_DOUBLE_EQ(m.at(1, 0), 8);
    EXPECT_DOUBLE_EQ(m.at(1, 1), 10);
    EXPECT_DOUBLE_EQ(m.at(1, 2), 12);
    
    // Other rows unchanged
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(2, 0), 7);
    
    // Test out of range
    EXPECT_THROW(m.scale_row(-1, 2.0), std::out_of_range);
    EXPECT_THROW(m.scale_row(5, 2.0), std::out_of_range);
}

TEST_F(MatrixRowOperationsTest, AddScaledRow) {
    Matrix<double, 3, 3> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    // Add 2 * row[0] to row[1]
    m.add_scaled_row(1, 0, 2.0);
    
    EXPECT_DOUBLE_EQ(m.at(1, 0), 4 + 2*1);  // 6
    EXPECT_DOUBLE_EQ(m.at(1, 1), 5 + 2*2);  // 9
    EXPECT_DOUBLE_EQ(m.at(1, 2), 6 + 2*3);  // 12
    
    // Other rows unchanged
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(2, 0), 7);
    
    // Test out of range
    EXPECT_THROW(m.add_scaled_row(-1, 0, 1.0), std::out_of_range);
    EXPECT_THROW(m.add_scaled_row(0, 5, 1.0), std::out_of_range);
}

// ===== Elementary Column Operations Tests =====

TEST_F(MatrixRowOperationsTest, SwapCols) {
    Matrix<double, 3, 3> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    m.swap_cols(0, 2);
    
    EXPECT_DOUBLE_EQ(m.at(0, 0), 3);
    EXPECT_DOUBLE_EQ(m.at(1, 0), 6);
    EXPECT_DOUBLE_EQ(m.at(2, 0), 9);
    EXPECT_DOUBLE_EQ(m.at(0, 2), 1);
    EXPECT_DOUBLE_EQ(m.at(1, 2), 4);
    EXPECT_DOUBLE_EQ(m.at(2, 2), 7);
    
    // Test out of range
    EXPECT_THROW(m.swap_cols(-1, 0), std::out_of_range);
    EXPECT_THROW(m.swap_cols(0, 5), std::out_of_range);
}

TEST_F(MatrixRowOperationsTest, ScaleCol) {
    Matrix<double, 3, 3> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    m.scale_col(1, 3.0);
    
    EXPECT_DOUBLE_EQ(m.at(0, 1), 6);
    EXPECT_DOUBLE_EQ(m.at(1, 1), 15);
    EXPECT_DOUBLE_EQ(m.at(2, 1), 24);
    
    // Other columns unchanged
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(m.at(0, 2), 3);
    
    // Test out of range
    EXPECT_THROW(m.scale_col(-1, 2.0), std::out_of_range);
    EXPECT_THROW(m.scale_col(5, 2.0), std::out_of_range);
}

TEST_F(MatrixRowOperationsTest, AddScaledCol) {
    Matrix<double, 3, 3> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    // Add -1 * col[2] to col[0]
    m.add_scaled_col(0, 2, -1.0);
    
    EXPECT_DOUBLE_EQ(m.at(0, 0), 1 - 3);   // -2
    EXPECT_DOUBLE_EQ(m.at(1, 0), 4 - 6);   // -2
    EXPECT_DOUBLE_EQ(m.at(2, 0), 7 - 9);   // -2
    
    // Other columns unchanged
    EXPECT_DOUBLE_EQ(m.at(0, 1), 2);
    EXPECT_DOUBLE_EQ(m.at(0, 2), 3);
    
    // Test out of range
    EXPECT_THROW(m.add_scaled_col(-1, 0, 1.0), std::out_of_range);
    EXPECT_THROW(m.add_scaled_col(0, 5, 1.0), std::out_of_range);
}

// ===== Helper Functions Tests =====

TEST_F(MatrixRowOperationsTest, ArgmaxAbsInCol) {
    Matrix<double, 4, 3> m(
        1,  2,  3,
        -5, 4,  6,
        2,  -8, 1,
        3,  1,  -9
    );
    
    // Find max abs in column 0
    auto [idx0, val0] = m.argmax_abs_in_col(0);
    EXPECT_EQ(idx0, 1);  // Row 1 has -5
    EXPECT_DOUBLE_EQ(val0, -5.0);
    
    // Find max abs in column 1
    auto [idx1, val1] = m.argmax_abs_in_col(1);
    EXPECT_EQ(idx1, 2);  // Row 2 has -8
    EXPECT_DOUBLE_EQ(val1, -8.0);
    
    // Find max abs in column 2 starting from row 1
    auto [idx2, val2] = m.argmax_abs_in_col(2, 1);
    EXPECT_EQ(idx2, 3);  // Row 3 has -9
    EXPECT_DOUBLE_EQ(val2, -9.0);
    
    // Test out of range
    EXPECT_THROW(m.argmax_abs_in_col(-1), std::out_of_range);
    EXPECT_THROW(m.argmax_abs_in_col(5), std::out_of_range);
}

TEST_F(MatrixRowOperationsTest, ArgmaxAbsInRow) {
    Matrix<double, 3, 4> m(
        1,  -5,  2,  3,
        2,  4,   -8, 1,
        3,  6,   1,  -9
    );
    
    // Find max abs in row 0
    auto [idx0, val0] = m.argmax_abs_in_row(0);
    EXPECT_EQ(idx0, 1);  // Column 1 has -5
    EXPECT_DOUBLE_EQ(val0, -5.0);
    
    // Find max abs in row 1
    auto [idx1, val1] = m.argmax_abs_in_row(1);
    EXPECT_EQ(idx1, 2);  // Column 2 has -8
    EXPECT_DOUBLE_EQ(val1, -8.0);
    
    // Find max abs in row 2 starting from column 1
    auto [idx2, val2] = m.argmax_abs_in_row(2, 1);
    EXPECT_EQ(idx2, 3);  // Column 3 has -9
    EXPECT_DOUBLE_EQ(val2, -9.0);
    
    // Test out of range
    EXPECT_THROW(m.argmax_abs_in_row(-1), std::out_of_range);
    EXPECT_THROW(m.argmax_abs_in_row(5), std::out_of_range);
}

// ===== Row Echelon Form Tests =====

TEST_F(MatrixRowOperationsTest, RowEchelonForm_FullRank) {
    // Construct matrix using column vectors
    Matrix<double, 3, 3> m(
        Vector<double, 3>(2, -3, -2),   // col 0
        Vector<double, 3>(1, -1, 1),    // col 1
        Vector<double, 3>(-1, 2, 2)     // col 2
    );

    std::cout << "Original Matrix:\n" << m << std::endl;
    auto rank= m.ref_inplace().rank;
    std::cout << "Row Echelon Form:\n" << m << std::endl;   
    
    EXPECT_EQ(rank, 3);
    
    // Verify row echelon form properties - should have pivots
    // The exact form depends on pivot selection
    // Just verify rank is correct and lower triangle has zeros
    for (int r = 1; r < 3; ++r) {
        for (int c = 0; c < r && c < rank; ++c) {
            EXPECT_NEAR(m.at(r, c), 0.0, 1e-10) 
                << "Expected zero at (" << r << ", " << c << ")";
        }
    }
}

TEST_F(MatrixRowOperationsTest, RowEchelonForm_RankDeficient) {
    Matrix<double, 3, 3> m(
        1, 2, 3,
        2, 4, 6,  // Row 2 = 2 * Row 1
        4, 5, 6
    );
    
    auto rank = m.ref_inplace().rank;
    
    EXPECT_EQ(rank, 2);  // Should be rank 2
}

TEST_F(MatrixRowOperationsTest, RowEchelonForm_RectangularMatrix) {
    // More rows than columns
    Matrix<double, 4, 3> m(
        1, 2, 3,
        4, 5, 6,
        7, 8, 9,
        10, 11, 12
    );
    
    auto rank = m.ref_inplace().rank;
    
    EXPECT_LE(rank, 3);  // Rank cannot exceed min(rows, cols)
}

// ===== Reduced Row Echelon Form Tests =====

TEST_F(MatrixRowOperationsTest, ReducedRowEchelonForm_Identity) {
    // Use column vector construction
    Matrix<double, 3, 3> m(
        Vector<double, 3>(2, -3, -2),
        Vector<double, 3>(1, -1, 1),
        Vector<double, 3>(-1, 2, 2)
    );
    
    auto rank = m.rref_inplace().rank;
    
    EXPECT_EQ(rank, 3);
    
    // For full rank square matrix, RREF should be identity
    Matrix<double, 3, 3> identity = Matrix<double, 3, 3>::identity();
    ExpectMatricesNear(m, identity, 1e-9);  // Relaxed tolerance for numerical errors
}

TEST_F(MatrixRowOperationsTest, ReducedRowEchelonForm_RankDeficient) {
    Matrix<double, 3, 4> m(
        1, 2, -1, -4,
        2, 3, -1, -11,
        -2, 0, -3, 22
    );

    auto rank = m.rref_inplace().rank;

    // Verify RREF properties
    // Leading entry in each row should be 1
    for (int r = 0; r < rank; ++r) {
        bool found_pivot = false;
        for (int c = 0; c < 4; ++c) {
            if (!is_zero(m.at(r, c))) {
                EXPECT_NEAR(m.at(r, c), 1.0, 1e-10) << "First non-zero in row " << r;
                found_pivot = true;
                
                // Column of pivot should have zeros elsewhere
                for (int rr = 0; rr < 3; ++rr) {
                    if (rr != r) {
                        EXPECT_NEAR(m.at(rr, c), 0.0, 1e-10) 
                            << "Expected zero at (" << rr << ", " << c << ")";
                    }
                }
                break;
            }
        }
        EXPECT_TRUE(found_pivot) << "No pivot found in row " << r;
    }
}

// ===== Rank Tests =====

TEST_F(MatrixRowOperationsTest, RankFullRank) {
    Matrix<double, 3, 3> m(
        1, 0, 2,
        0, 1, 3,
        2, 1, 0
    );
    
    EXPECT_EQ(m.rank(), 3);
}

TEST_F(MatrixRowOperationsTest, RankDeficient) {
    Matrix<double, 3, 3> m(
        1, 2, 3,
        2, 4, 6,
        3, 6, 9
    );
    
    EXPECT_EQ(m.rank(), 1);
}

TEST_F(MatrixRowOperationsTest, RankZeroMatrix) {
    Matrix<double, 3, 3> m = Matrix<double, 3, 3>::zeros();
    
    EXPECT_EQ(m.rank(), 0);
}

TEST_F(MatrixRowOperationsTest, RankRectangular) {
    Matrix<double, 2, 4> m(
        1, 2, 3, 4,
        5, 6, 7, 8
    );
    
    int rank = m.rank();
    EXPECT_GE(rank, 1);
    EXPECT_LE(rank, 2);
}

// ===== Inverse via RREF Tests =====

TEST_F(MatrixRowOperationsTest, InverseRREF_2x2) {
    Matrix<double, 2, 2> m(4, 7, 2, 6);
    
    auto inv = m.inversed_rref();
    auto product = m * inv;
    auto identity = Matrix<double, 2, 2>::identity();
    
    ExpectMatricesNear(product, identity, 1e-10);
}

TEST_F(MatrixRowOperationsTest, InverseRREF_3x3) {
    Matrix<double, 3, 3> m(
        2, -1, 0,
        -1, 2, -1,
        0, -1, 2
    );
    
    auto inv = m.inversed_rref();
    auto product = m * inv;
    auto identity = Matrix<double, 3, 3>::identity();
    
    ExpectMatricesNear(product, identity, 1e-10);
    
    // Test in-place version
    Matrix<double, 3, 3> m2 = m;
    m2.inverse_rref();
    ExpectMatricesNear(m2, inv, 1e-10);
}

TEST_F(MatrixRowOperationsTest, InverseRREF_4x4) {
    // Use column vector construction
    Matrix<double, 4, 4> m(
       1, 0, 3, 4,
       5, 6, 7, 8,
       9, 10, 0, 12,
       13, 0, 15, 16
    );

    std::cout << "Is Matrix invertible?: " << m.is_invertible() << std::endl;
    auto inv = m.inversed_rref();
    auto product = m * inv;
    auto identity = Matrix<double, 4, 4>::identity();
    ExpectMatricesNear(product, identity, 1e-9);  // Relaxed tolerance
}

TEST_F(MatrixRowOperationsTest, InverseRREF_CompareWithDefault) {
    // Compare RREF-based inverse with default inversed() method
    
    // 2x2
    Matrix<double, 2, 2> m2(3, 8, 4, 6);
    auto inv_rref_2 = m2.inversed_rref();
    auto inv_default_2 = m2.inversed();
    ExpectMatricesNear(inv_rref_2, inv_default_2, 1e-10);
    
    // 3x3
    Matrix<double, 3, 3> m3(6, 1, 1, 4, -2, 5, 2, 8, 7);
    auto inv_rref_3 = m3.inversed_rref();
    auto inv_default_3 = m3.inversed();
    ExpectMatricesNear(inv_rref_3, inv_default_3, 1e-10);
    
    // 4x4
    Matrix<double, 4, 4> m4(
        2, 0, 0, 1,
        0, 2, 0, 2,
        0, 0, 2, 3,
        0, 0, 0, 1
    );
    auto inv_rref_4 = m4.inversed_rref();
    auto inv_default_4 = m4.inversed();
    ExpectMatricesNear(inv_rref_4, inv_default_4, 1e-10);
}

// ===== Determinant via REF Tests =====

TEST_F(MatrixRowOperationsTest, DeterminantREF_2x2) {
    
    Matrix<double, 2, 2> m(
        Vector<double, 2>(3, 4),
        Vector<double, 2>(8, 6)
    );
    
    double det_ref = m.determinant_ref();
    double det_regular = m.determinant();
    
    EXPECT_NEAR(det_ref, det_regular, 1e-10);
    EXPECT_NEAR(det_ref, 3*6 - 8*4, 1e-10);
}

TEST_F(MatrixRowOperationsTest, DeterminantREF_3x3) {
    // Use column vector construction
    Matrix<double, 3, 3> m(
        Vector<double, 3>(6, 4, 2),  // col 0
        Vector<double, 3>(1, -2, 8), // col 1
        Vector<double, 3>(1, 5, 7)   // col 2
    );
    
    double det_ref = m.determinant_ref();
    double det_regular = m.determinant();
    
    EXPECT_NEAR(det_ref, det_regular, 1e-10);
}

TEST_F(MatrixRowOperationsTest, DeterminantREF_4x4) {
    // Use column vector construction
    Matrix<double, 4, 4> m(
        Vector<double, 4>(1, 3, 2, 1),  // col 0
        Vector<double, 4>(2, 1, 0, 1),  // col 1
        Vector<double, 4>(0, 2, 1, 0),  // col 2
        Vector<double, 4>(1, 0, 3, 2)   // col 3
    );
    
    double det_ref = m.determinant_ref();
    double det_regular = m.determinant();
    
    EXPECT_NEAR(det_ref, det_regular, 1e-10);
}

TEST_F(MatrixRowOperationsTest, DeterminantREF_Singular) {
    Matrix<double, 3, 3> m(
        1, 2, 3,
        2, 4, 6,
        3, 6, 9
    );
    
    double det = m.determinant_ref();
    EXPECT_NEAR(det, 0.0, 1e-10);
}

// ===== Integration Tests =====

TEST_F(MatrixRowOperationsTest, ComplexLinearSystem) {
    // Solve Ax = b using RREF
    // System: 2x + y - z = 8
    //        -3x - y + 2z = -11
    //        -2x + y + 2z = -3
    
    // Use column vector construction for augmented matrix
    Matrix<double, 3, 4> augmented(
        Vector<double, 3>(2, -3, -2),   // col 0 (coefficients of x)
        Vector<double, 3>(1, -1, 1),    // col 1 (coefficients of y)
        Vector<double, 3>(-1, 2, 2),    // col 2 (coefficients of z)
        Vector<double, 3>(8, -11, -3)   // col 3 (constants)
    );
    
    augmented.rref_inplace();
    
    // Solution should be x=2, y=3, z=-1
    // After RREF, the last column contains the solution
    EXPECT_NEAR(augmented.at(0, 3), 2.0, 1e-9);
    EXPECT_NEAR(augmented.at(1, 3), 3.0, 1e-9);
    EXPECT_NEAR(augmented.at(2, 3), -1.0, 1e-9);
}

TEST_F(MatrixRowOperationsTest, ChainedOperations) {
    Matrix<double, 3, 3> m(
        1, 2, 3,
        4, 5, 6,
        7, 8, 10  // Slightly modified to make it full rank
    );
    
    // Chain multiple operations
    m.swap_rows(0, 2)
     .scale_row(1, 0.5)
     .add_scaled_row(2, 0, -1.0);
    
    // Verify operations were applied
    EXPECT_DOUBLE_EQ(m.at(0, 0), 7);   // After swap
    EXPECT_DOUBLE_EQ(m.at(1, 0), 2);   // After scale
    EXPECT_DOUBLE_EQ(m.at(2, 0), 1-7); // After add_scaled
}

TEST_F(MatrixRowOperationsTest, NumericalStability) {
    // Test with a matrix that might have numerical issues
    Matrix<double, 3, 3> m(
        1e10, 1e10, 1e10,
        1.0, 2.0, 3.0,
        2.0, 3.0, 5.0
    );

    auto [mat, rank, swaps, total_scale] = m.ref_inplace();

    // Should handle large values correctly
    EXPECT_GE(rank, 2);
    
    // Check that zeros are properly cleaned up
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (is_zero(m.at(r, c))) {
                EXPECT_DOUBLE_EQ(m.at(r, c), 0.0);
            }
        }
    }
}

// ===== Least Mean Square (LMS) Tests =====

TEST_F(MatrixRowOperationsTest, SolveLMS_SimpleCase) {
    // Test solve_LMS with a simple overdetermined system
    // We want to find M such that M * A ≈ B
    // Example: fit a 2x2 transformation matrix
    
    // A: 2x3 matrix (2 dimensions, 3 constraint points)
    Matrix<double, 2, 3> A(
        Vector<double, 2>(1, 0),
        Vector<double, 2>(0, 1),
        Vector<double, 2>(1, 1)
    );
    
    // B: target values (what we want M*A to equal)
    Matrix<double, 2, 3> B(
        Vector<double, 2>(2, 0),
        Vector<double, 2>(0, 3),
        Vector<double, 2>(2, 3)
    );
    
    auto M = solve_LMS(A, B);
    
    // M should be approximately a 2x2 scaling matrix
    // Verify by checking M * A ≈ B
    auto result = M * A;
    ExpectMatricesNear(result, B, 1e-9);
}

TEST_F(MatrixRowOperationsTest, SolveLMS_IdentityCase) {
    // If A = I and B = I, then M should be I
    Matrix<double, 3, 3> A = Matrix<double, 3, 3>::identity();
    Matrix<double, 3, 3> B = Matrix<double, 3, 3>::identity();
    
    auto M = solve_LMS(A, B);
    auto identity = Matrix<double, 3, 3>::identity();
    
    ExpectMatricesNear(M, identity, 1e-10);
}

TEST_F(MatrixRowOperationsTest, SolveLMS_ScalingTransform) {
    // Test with a known scaling transformation
    // A = points, B = scaled points
    Matrix<double, 2, 4> A(
        Vector<double, 2>(1, 0),
        Vector<double, 2>(0, 1),
        Vector<double, 2>(2, 0),
        Vector<double, 2>(0, 2)
    );
    
    // Expected: scale by [2, 3]
    Matrix<double, 2, 4> B(
        Vector<double, 2>(2, 0),
        Vector<double, 2>(0, 3),
        Vector<double, 2>(4, 0),
        Vector<double, 2>(0, 6)
    );
    
    auto M = solve_LMS(A, B);
    
    // M should be a diagonal matrix with [2, 3] on diagonal
    EXPECT_NEAR(M.at(0, 0), 2.0, 1e-9);
    EXPECT_NEAR(M.at(1, 1), 3.0, 1e-9);
    EXPECT_NEAR(M.at(0, 1), 0.0, 1e-9);
    EXPECT_NEAR(M.at(1, 0), 0.0, 1e-9);
    
    // Verify result
    auto result = M * A;
    ExpectMatricesNear(result, B, 1e-9);
}

TEST_F(MatrixRowOperationsTest, SolveLMS_RotationApproximation) {
    // Test approximating a rotation with data
    // Create some points and their rotated versions
    constexpr double angle = math::pi_v<double> / 4.0; // 45 degrees
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    
    Matrix<double, 2, 5> A(
        Vector<double, 2>(1, 0),
        Vector<double, 2>(0, 1),
        Vector<double, 2>(1, 1),
        Vector<double, 2>(2, 0),
        Vector<double, 2>(0, 2)
    );
    
    // Apply rotation matrix to get B
    Matrix<double, 2, 2> rotation(
        Vector<double, 2>(cos_a, sin_a),
        Vector<double, 2>(-sin_a, cos_a)
    );
    
    Matrix<double, 2, 5> B = rotation * A;
    
    // Solve for M
    auto M = solve_LMS(A, B);
    
    // M should approximate the rotation matrix
    ExpectMatricesNear(M, rotation, 1e-9);
    
    // Verify the result
    auto result = M * A;
    ExpectMatricesNear(result, B, 1e-9);
}

TEST_F(MatrixRowOperationsTest, SolveLMS_OverdeterminedSystem) {
    // Test with more constraints than unknowns
    // Fit a linear transformation with 10 point constraints
    // Use deterministic data instead of random
    Matrix<double, 2, 10> A(
        Vector<double, 2>(1.0, 0.5),
        Vector<double, 2>(2.0, 1.0),
        Vector<double, 2>(3.0, 1.5),
        Vector<double, 2>(4.0, 2.0),
        Vector<double, 2>(5.0, 2.5),
        Vector<double, 2>(0.5, 1.0),
        Vector<double, 2>(1.5, 2.0),
        Vector<double, 2>(2.5, 3.0),
        Vector<double, 2>(3.5, 4.0),
        Vector<double, 2>(4.5, 5.0)
    );
    
    // Create target transformation
    Matrix<double, 2, 2> target_M(
        Vector<double, 2>(1.5, 0.5),
        Vector<double, 2>(-0.3, 2.0)
    );
    
    Matrix<double, 2, 10> B = target_M * A;
    
    // Solve for M
    auto M = solve_LMS(A, B);
    
    // M should be very close to target_M
    ExpectMatricesNear(M, target_M, 1e-9);
    
    // Verify solution
    auto result = M * A;
    ExpectMatricesNear(result, B, 1e-9);
}

TEST_F(MatrixRowOperationsTest, SolveLMS_3DTransformation) {
    // Test with 3D transformations
    Matrix<double, 3, 6> A(
        Vector<double, 3>(1, 0, 0),
        Vector<double, 3>(0, 1, 0),
        Vector<double, 3>(0, 0, 1),
        Vector<double, 3>(1, 1, 0),
        Vector<double, 3>(1, 0, 1),
        Vector<double, 3>(0, 1, 1)
    );
    
    // Create a known 3x3 transformation
    Matrix<double, 3, 3> target_M(
        Vector<double, 3>(2, 0, 0),
        Vector<double, 3>(0, 3, 0),
        Vector<double, 3>(0, 0, 4)
    );
    
    Matrix<double, 3, 6> B = target_M * A;
    
    // Solve for M
    auto M = solve_LMS(A, B);
    
    // Should recover the scaling matrix
    ExpectMatricesNear(M, target_M, 1e-9);
}

}  // namespace pbpt::math::testing
