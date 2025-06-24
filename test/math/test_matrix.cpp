#include "gtest/gtest.h"
#include "math/matrix.hpp" // The header file for the Matrix class to be tested.


using namespace pbpt::math;

/**
 * @brief Helper function to compare two matrices for near-equality.
 * @details This is necessary for floating-point types to avoid precision issues.
 * @tparam T Matrix element type.
 * @tparam R Number of rows.
 * @tparam C Number of columns.
 * @param m1 The first matrix.
 * @param m2 The second matrix.
 * @param tolerance The maximum allowed difference between elements.
 */
template <typename T, int R, int C>
void ExpectMatricesNear(const Matrix<T, R, C>& m1, const Matrix<T, R, C>& m2, T tolerance = 1e-6) {
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            EXPECT_NEAR(m1(r, c), m2(r, c), tolerance);
        }
    }
}

// Test suite for the Matrix class
class MatrixTest : public ::testing::Test {
protected:
    // You can set up shared objects here if needed
};

TEST_F(MatrixTest, DefaultConstructorAndZeros) {
    Mat3 m; // Default constructor should create a zero matrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_EQ(m(r, c), 0.0f);
        }
    }

    Mat2 m2 = Mat2::zeros();
    Mat2 expected_zeros; // Also a zero matrix by default
    ExpectMatricesNear(m2, expected_zeros);
}

TEST_F(MatrixTest, IdentityFactory) {
    Mat4 id = Mat4::identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_EQ(id(r, c), (r == c) ? 1.0f : 0.0f);
        }
    }
}

TEST_F(MatrixTest, VectorConstructor) {
    Vec3 r0(1, 2, 3);
    Vec3 r1(4, 5, 6);
    Vec3 r2(7, 8, 9);
    Mat3 m(r0, r1, r2);

    EXPECT_EQ(m(0, 0), 1); EXPECT_EQ(m(0, 1), 2); EXPECT_EQ(m(0, 2), 3);
    EXPECT_EQ(m(1, 0), 4); EXPECT_EQ(m(1, 1), 5); EXPECT_EQ(m(1, 2), 6);
    EXPECT_EQ(m(2, 0), 7); EXPECT_EQ(m(2, 1), 8); EXPECT_EQ(m(2, 2), 9);
}

TEST_F(MatrixTest, AccessorsAndDimensions) {
    Mat3x4 m;
    EXPECT_EQ(m.row_dims(), 3);
    EXPECT_EQ(m.col_dims(), 4);

    m(1, 2) = 42.0f;
    EXPECT_EQ(m(1, 2), 42.0f);
    
    // Test boundary checks for runtime exceptions
    EXPECT_THROW(m(3, 0), std::out_of_range);
    EXPECT_THROW(m(0, 4), std::out_of_range);
    EXPECT_THROW(m(-1, 0), std::out_of_range);
}

TEST_F(MatrixTest, RowAndColExtractionToVec) {
    Mat2 m(Vec2(1, 2), Vec2(3, 4));
    
    // This now tests the implicit conversion from a view to a Vec
    Vec2 r0 = m.row(0);
    EXPECT_EQ(r0.x(), 1); EXPECT_EQ(r0.y(), 2);

    Vec2 c1 = m.col(1);
    EXPECT_EQ(c1.x(), 2); EXPECT_EQ(c1.y(), 4);
}

TEST_F(MatrixTest, AdditionAndSubtraction) {
    Mat2 m1(Vec2(1, 2), Vec2(3, 4));
    Mat2 m2(Vec2(5, 6), Vec2(7, 8));
    
    Mat2 sum = m1 + m2;
    Mat2 expected_sum(Vec2(6, 8), Vec2(10, 12));
    ExpectMatricesNear(sum, expected_sum);
    
    Mat2 diff = m2 - m1;
    Mat2 expected_diff(Vec2(4, 4), Vec2(4, 4));
    ExpectMatricesNear(diff, expected_diff);
    
    m1 += m2; // Test compound assignment
    ExpectMatricesNear(m1, expected_sum);
}

TEST_F(MatrixTest, ScalarMultiplication) {
    Mat2 m(Vec2(1, 2), Vec2(3, 4));
    
    Mat2 m_times_2 = m * 2.0f;
    Mat2 expected(Vec2(2, 4), Vec2(6, 8));
    ExpectMatricesNear(m_times_2, expected);
    
    Mat2 two_times_m = 2.0f * m;
    ExpectMatricesNear(two_times_m, expected);
    
    m *= 3.0f; // Test compound assignment
    Mat2 expected_3(Vec2(3, 6), Vec2(9, 12));
    ExpectMatricesNear(m, expected_3);
}

TEST_F(MatrixTest, MatrixVectorMultiplication) {
    Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    Vec3 v(2, 1, 3);
    
    Vec3 result = m * v;
    Vec3 expected(13, 31, 49);
    
    EXPECT_NEAR(result.x(), expected.x(), 1e-6);
    EXPECT_NEAR(result.y(), expected.y(), 1e-6);
    EXPECT_NEAR(result.z(), expected.z(), 1e-6);
}

TEST_F(MatrixTest, MatrixMatrixMultiplication) {
    Mat2 m1(Vec2(1, 2), Vec2(3, 4));
    Mat2 m2(Vec2(2, 0), Vec2(1, 2));

    Mat2 result = m1 * m2;
    Mat2 expected(Vec2(4, 4), Vec2(10, 8));
    ExpectMatricesNear(result, expected);
}

TEST_F(MatrixTest, Transpose) {
    Matrix<Float, 2, 3> m(Vec3(1, 2, 3), Vec3(4, 5, 6));
    
    Matrix<Float, 3, 2> mt = m.transpose();
    
    EXPECT_EQ(mt.row_dims(), 3);
    EXPECT_EQ(mt.col_dims(), 2);
    EXPECT_EQ(mt(0, 0), 1);
    EXPECT_EQ(mt(1, 0), 2);
    EXPECT_EQ(mt(2, 1), 6);
}

TEST_F(MatrixTest, Determinant) {
    Mat2 m(Vec2(3, 8), Vec2(4, 6));
    EXPECT_NEAR(m.determinant(), -14.0f, 1e-6);

    Mat3 m3(Vec3(6, 1, 1), Vec3(4, -2, 5), Vec3(2, 8, 7));
    EXPECT_NEAR(m3.determinant(), -306.0f, 1e-6);
}

TEST_F(MatrixTest, Inverse) {
    Mat2 m(Vec2(4, 7), Vec2(2, 6)); // Determinant is 10
    Mat2 m_inv = m.inverse();
    EXPECT_NEAR(m_inv(0, 0), 0.6f, 1e-6);
    EXPECT_NEAR(m_inv(0, 1), -0.7f, 1e-6);
    EXPECT_NEAR(m_inv(1, 0), -0.2f, 1e-6);
    EXPECT_NEAR(m_inv(1, 1), 0.4f, 1e-6);

    Mat2 identity = m * m_inv;

    ExpectMatricesNear(identity, Mat2::identity());
    
    // Test singular matrix exception
    Mat2 singular(Vec2(2, 4), Vec2(2, 4));
    EXPECT_THROW(singular.inverse(), std::runtime_error);
}

TEST_F(MatrixTest, SubmatrixCopy) {
    Mat4 m = Mat4::identity();
    m(0, 1) = 5; m(0, 2) = 6;
    m(1, 1) = 7; m(1, 2) = 8;
    
    // Test contiguous submatrix
    Mat2 sub = m.submatrix<2, 2>(0, 1);
    EXPECT_EQ(sub(0, 0), 5);
    EXPECT_EQ(sub(0, 1), 6);
    EXPECT_EQ(sub(1, 0), 7);
    EXPECT_EQ(sub(1, 1), 8);
}

TEST_F(MatrixTest, MatrixViewModificationAndAssignment) {
    Mat4 m = Mat4::zeros();
    
    // 1. Get a view into a 2x2 sub-region
    auto v = m.view<2, 2>(1, 1);
    
    // 2. Modify an element through the view
    v(0, 0) = 42.0f;
    
    // 3. Verify the original matrix was changed
    EXPECT_EQ(m(1, 1), 42.0f);
    EXPECT_EQ(m(0, 0), 0.0f); // Ensure other elements are untouched
    
    // 4. Assign a new matrix to the view
    Mat2 new_data(Vec2(10, 20), Vec2(30, 40));
    v = new_data;
    
    // 5. Verify the block in the original matrix is updated
    EXPECT_EQ(m(1, 1), 10);
    EXPECT_EQ(m(1, 2), 20);
    EXPECT_EQ(m(2, 1), 30);
    EXPECT_EQ(m(2, 2), 40);
    
    // 6. Convert the view back to a new, owning Matrix and check its contents
    Matrix<Float, 2, 2> copied_matrix = v;
    ExpectMatricesNear(copied_matrix, new_data);
}

TEST_F(MatrixTest, VectorViewTest) {
    Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    
    // --- Test Row View ---
    auto row1_view = m.row(1);
    EXPECT_EQ(row1_view[0], 4);
    EXPECT_EQ(row1_view[2], 6);

    // Modify through row view
    row1_view[1] = 99.0f;
    EXPECT_EQ(m(1, 1), 99.0f);
    
    // Assign to row view
    row1_view = Vec3(10, 20, 30);
    EXPECT_EQ(m(1, 0), 10);
    EXPECT_EQ(m(1, 1), 20);
    EXPECT_EQ(m(1, 2), 30);

    // --- Test Column View ---
    auto col0_view = m.col(0);
    EXPECT_EQ(col0_view[0], 1);
    EXPECT_EQ(col0_view[1], 10); // Check value modified via row_view
    
    // Modify through column view
    col0_view[2] = 77.0f;
    EXPECT_EQ(m(2, 0), 77.0f);
    
    // Assign to column view
    col0_view = Vec3(11, 22, 33);
    EXPECT_EQ(m(0, 0), 11);
    EXPECT_EQ(m(1, 0), 22);
    EXPECT_EQ(m(2, 0), 33);

    // --- Test Const Correctness for VectorView ---
    const Mat3 const_m = m;
    auto const_row_view = const_m.row(0);
    // The following line would cause a compile error, which is correct:
    // const_row_view[0] = 5.0f; 
    EXPECT_EQ(const_row_view[0], 11);
}

TEST_F(MatrixTest, ConstMatrixViewTest) {
    Mat4 m = Mat4::identity();
    m(1, 2) = 123.0f;

    const Mat4& const_m = m;

    // 1. Get a const view from a const matrix
    auto const_view = const_m.view<2, 2>(1, 1);

    // 2. Verify we can read from the const view
    EXPECT_EQ(const_view(0, 0), 1.0f); // From identity
    EXPECT_EQ(const_view(0, 1), 123.0f);

    // 3. Verify that writing to the view is a compile error
    // const_view(0, 0) = 456.0f; // THIS LINE MUST NOT COMPILE!

    // 4. Verify conversion to an owning Matrix works
    Mat2 copy = const_view;
    EXPECT_EQ(copy(0, 1), 123.0f);
}

TEST_F(MatrixTest, ViewApplyWithStdFunction) {
    // Test VectorView::apply
    Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    auto row1_view = m.row(1);

    std::function<void(Float&, int)> multiply_by_index_plus_one = 
        [](Float& elem, int index) {
        elem *= (index + 1);
    };

    row1_view.apply(multiply_by_index_plus_one);

    // Expected new row: [4*1, 5*2, 6*3] = [4, 10, 18]
    EXPECT_EQ(m(1, 0), 4);
    EXPECT_EQ(m(1, 1), 10);
    EXPECT_EQ(m(1, 2), 18);

    // Test MatrixView::apply
    Mat2 m2(Vec2(1, 2), Vec2(3, 4));
    Mat2::View<2, 2> full_view = m2.view<2, 2>(0, 0);

    std::function<void(Float&, int, int)> add_row_and_col = 
        [](Float& elem, int r, int c) {
        elem += (r * 10 + c);
    };

    full_view.apply(add_row_and_col);

    // Expected new matrix:
    // [1 + (0*10+0), 2 + (0*10+1)] = [1, 3]
    // [3 + (1*10+0), 4 + (1*10+1)] = [13, 15]
    EXPECT_EQ(m2(0, 0), 1);
    EXPECT_EQ(m2(0, 1), 3);
    EXPECT_EQ(m2(1, 0), 13);
    EXPECT_EQ(m2(1, 1), 15);
}
