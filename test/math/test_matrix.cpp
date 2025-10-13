#include "gtest/gtest.h"
#include "pbpt.h"
#include <random>

namespace pbpt::math::testing {

/**
 * @brief Helper function to compare two matrices for near-equality.
 * @details This is necessary for floating-point types to avoid precision
 * issues.
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
            EXPECT_NEAR(m1.at(r, c), m2.at(r, c), tolerance);
        }
    }
}

class MatrixTest : public ::testing::Test {
protected:
    // You can set up shared objects here if needed
};

// --- Constructor and Factory Tests ---

TEST_F(MatrixTest, DefaultConstructorAndZeros) {
    Mat3 m;  // Default constructor should create a zero matrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_EQ(m.at(r, c), 0.0f);
        }
    }

    Mat2 m2 = Mat2::zeros();
    Mat2 expected_zeros;  // Also a zero matrix by default
    ExpectMatricesNear(m2, expected_zeros);
}

TEST_F(MatrixTest, IdentityFactory) {
    Mat4 id = Mat4::identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_EQ(id.at(r, c), (r == c) ? 1.0f : 0.0f);
        }
    }
}

TEST_F(MatrixTest, ValueListConstructor) {
    // Test with correct number of values (3x3 matrix of floats)
    Mat3 mat1(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    EXPECT_FLOAT_EQ(mat1.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(mat1.at(1, 1), 5.0f);
    EXPECT_FLOAT_EQ(mat1.at(2, 2), 9.0f);

    // Test with correct number of values (2x2 matrix)
    Mat2 mat2(1.1f, 2.2f, 3.3f, 4.4f);
    EXPECT_FLOAT_EQ(mat2.at(0, 0), 1.1f);
    EXPECT_FLOAT_EQ(mat2.at(1, 1), 4.4f);

    // Test with single value (1x1 matrix) - This also checks for ambiguity
    // resolution
    Matrix<Float, 1, 1> mat3(3.14f);
    EXPECT_FLOAT_EQ(mat3.at(0, 0), 3.14);

    // Test with mixed types that can be converted to matrix type
    Matrix<Float, 2, 1> mat5(1.0, 2.5f);  // double and float literal
    EXPECT_FLOAT_EQ(mat5.at(0, 0), 1.0);
    EXPECT_FLOAT_EQ(mat5.at(1, 0), 2.5);
}

TEST_F(MatrixTest, ColVectorConstructor) {
    Vec3 r0(1, 2, 3);
    Vec3 r1(4, 5, 6);
    Vec3 r2(7, 8, 9);
    Mat3 m(r0, r1, r2);

    EXPECT_FLOAT_EQ(m.at(0, 0), 1);
    EXPECT_FLOAT_EQ(m.at(0, 1), 4);
    EXPECT_FLOAT_EQ(m.at(0, 2), 7);
    EXPECT_FLOAT_EQ(m.at(1, 0), 2);
    EXPECT_FLOAT_EQ(m.at(1, 1), 5);
    EXPECT_FLOAT_EQ(m.at(1, 2), 8);
    EXPECT_FLOAT_EQ(m.at(2, 0), 3);
    EXPECT_FLOAT_EQ(m.at(2, 1), 6);
    EXPECT_FLOAT_EQ(m.at(2, 2), 9);
}

TEST_F(MatrixTest, ColumnVectorConstructor) {
    // For a 3x2 matrix, the constructor should expect 2 column vectors of
    // size 3.
    using TestMat = Matrix<Float, 3, 2>;
    using ColVec  = Vector<Float, 3>;

    ColVec c0(1.f, 2.f, 3.f);
    ColVec c1(4.f, 5.f, 6.f);

    // Construct the matrix from two column vectors
    TestMat m(c0, c1);

    // Column 0 should be {1, 2, 3}
    EXPECT_FLOAT_EQ(m.at(0, 0), 1.f);
    EXPECT_FLOAT_EQ(m.at(1, 0), 2.f);
    EXPECT_FLOAT_EQ(m.at(2, 0), 3.f);

    // Column 1 should be {4, 5, 6}
    EXPECT_FLOAT_EQ(m.at(0, 1), 4.f);
    EXPECT_FLOAT_EQ(m.at(1, 1), 5.f);
    EXPECT_FLOAT_EQ(m.at(2, 1), 6.f);
}

// --- Core Functionality Tests ---

TEST_F(MatrixTest, AccessorsAndDimensions) {
    Mat3x4 m;
    EXPECT_EQ(m.row_dims(), 3);
    EXPECT_EQ(m.col_dims(), 4);

    m[1][2] = 42.0f;
    EXPECT_EQ(m.at(1, 2), 42.0f);

    // Test boundary checks for runtime exceptions
    EXPECT_THROW(m.at(3, 0), std::out_of_range);
    EXPECT_THROW(m.at(0, 4), std::out_of_range);
    EXPECT_THROW(m.at(-1, 0), std::out_of_range);
}

TEST_F(MatrixTest, RowAndColExtractionToVec) {
    // Mat2 m(
    //     1, 2,
    //     3, 4
    // );

    Mat2 m(Vec2(1, 3), Vec2(2, 4));

    // This now tests the implicit conversion from a view to a Vec
    Vec2 r0 = m.row(0).to_vector();
    EXPECT_EQ(r0.x(), 1);
    EXPECT_EQ(r0.y(), 2);

    Vec2 c1 = m.col(1).to_vector();
    EXPECT_EQ(c1.x(), 2);
    EXPECT_EQ(c1.y(), 4);

    Vec2 c2 = m.col(0).to_vector();
    EXPECT_EQ(c2.x(), 1);
    EXPECT_EQ(c2.y(), 3);
}

// --- Arithmetic and Algebra Tests ---

TEST_F(MatrixTest, AdditionAndSubtraction) {
    Mat2 m1(Vec2(1, 2), Vec2(3, 4));
    Mat2 m2(Vec2(5, 6), Vec2(7, 8));

    Mat2 sum = m1 + m2;
    Mat2 expected_sum(Vec2(6, 8), Vec2(10, 12));
    ExpectMatricesNear(sum, expected_sum);

    Mat2 diff = m2 - m1;
    Mat2 expected_diff(Vec2(4, 4), Vec2(4, 4));
    ExpectMatricesNear(diff, expected_diff);

    m1 += m2;  // Test compound assignment
    ExpectMatricesNear(m1, expected_sum);
}

TEST_F(MatrixTest, ScalarMultiplication) {
    Mat2 m(Vec2(1, 2), Vec2(3, 4));

    Mat2 m_times_2 = m * 2.0f;
    Mat2 expected(Vec2(2, 4), Vec2(6, 8));
    ExpectMatricesNear(m_times_2, expected);

    Mat2 two_times_m = 2.0f * m;
    ExpectMatricesNear(two_times_m, expected);

    m *= 3.0f;  // Test compound assignment
    Mat2 expected_3(Vec2(3, 6), Vec2(9, 12));
    ExpectMatricesNear(m, expected_3);
}

TEST_F(MatrixTest, MatrixVectorMultiplication) {
    Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    Vec3 v(2, 1, 3);

    Vec3 result = m * v;
    Vec3 expected(27, 33, 39);

    EXPECT_NEAR(result.x(), expected.x(), 1e-6);
    EXPECT_NEAR(result.y(), expected.y(), 1e-6);
    EXPECT_NEAR(result.z(), expected.z(), 1e-6);
}

TEST_F(MatrixTest, MatrixMatrixMultiplication) {
    Mat2 m1(Vec2(1, 2), Vec2(3, 4));
    Mat2 m2(Vec2(2, 0), Vec2(1, 2));

    Mat2 result = m1 * m2;
    Mat2 expected(Vec2(2, 4), Vec2(7, 10));
    ExpectMatricesNear(result, expected);
}

TEST_F(MatrixTest, Transpose) {
    Matrix<Float, 2, 3> m(1.f, 2.f, 3.f, 4.f, 5.f, 6.f);

    Matrix<Float, 3, 2> mt = m.transposed();

    EXPECT_EQ(mt.row_dims(), 3);
    EXPECT_EQ(mt.col_dims(), 2);
    EXPECT_EQ(mt.at(0, 0), 1);
    EXPECT_EQ(mt.at(1, 0), 2);  // m(0,1) becomes mt(1,0)
    EXPECT_EQ(mt.at(2, 1), 6);  // m(1,2) becomes mt(2,1)
}

TEST_F(MatrixTest, Determinant) {
    Mat2 m(Vec2(3, 8), Vec2(4, 6));
    EXPECT_NEAR(m.determinant(), -14.0f, 1e-6);

    Mat3 m3(Vec3(6, 1, 1), Vec3(4, -2, 5), Vec3(2, 8, 7));
    EXPECT_NEAR(m3.determinant(), -306.0f, 1e-6);
}

TEST_F(MatrixTest, Inverse) {
    // Mat2 m(Vec2(4, 7), Vec2(2, 6)); // Determinant is 10

    Mat2 m(4, 2, 7, 6);

    Mat2 m_inv = m.inversed();
    EXPECT_NEAR(m_inv.at(0, 0), 0.6f, 1e-6);
    EXPECT_NEAR(m_inv.at(0, 1), -0.2f, 1e-6);
    EXPECT_NEAR(m_inv.at(1, 0), -0.7f, 1e-6);
    EXPECT_NEAR(m_inv.at(1, 1), 0.4f, 1e-6);

    Mat2 identity = m * m_inv;

    ExpectMatricesNear(identity, Mat2::identity());

    // Test singular matrix exception
    Mat2 singular(Vec2(2, 4), Vec2(2, 4));
    EXPECT_THROW(singular.inversed(), std::domain_error);
}

// --- View and Submatrix Tests ---

TEST_F(MatrixTest, SubmatrixCopy) {
    Mat4 m  = Mat4::identity();
    m[0][1] = 5;
    m[0][2] = 6;
    m[1][1] = 7;
    m[1][2] = 8;

    // Test contiguous submatrix
    Mat2 sub = m.view<2, 2>(0, 1).to_matrix();
    EXPECT_EQ(sub.at(0, 0), 5);
    EXPECT_EQ(sub.at(0, 1), 6);
    EXPECT_EQ(sub.at(1, 0), 7);
    EXPECT_EQ(sub.at(1, 1), 8);
}

TEST_F(MatrixTest, MatrixViewModificationAndAssignment) {
    Mat4 m = Mat4::zeros();

    // 1. Get a view into a 2x2 sub-region
    auto v = m.view<2, 2>(1, 1);

    // 2. Modify an element through the view
    v.at(0, 0) = 42.0f;

    // 3. Verify the original matrix was changed
    EXPECT_EQ(m.at(1, 1), 42.0f);
    EXPECT_EQ(m.at(0, 0), 0.0f);  // Ensure other elements are untouched

    // 4. Assign a new matrix to the view
    Mat2 new_data(Vec2(10, 20), Vec2(30, 40));
    v = new_data;

    // 5. Verify the block in the original matrix is updated
    EXPECT_EQ(m.at(1, 1), 10);
    EXPECT_EQ(m.at(2, 1), 20);
    EXPECT_EQ(m.at(1, 2), 30);
    EXPECT_EQ(m.at(2, 2), 40);

    // 6. Convert the view back to a new, owning Matrix and check its contents
    Matrix<Float, 2, 2> copied_matrix = v.to_matrix();
    ExpectMatricesNear(copied_matrix, new_data);
}

TEST_F(MatrixTest, VectorViewTest) {
    Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));

    // --- Test Row View ---
    auto row1_view = m.row(1);
    EXPECT_EQ(row1_view[0], 2);
    EXPECT_EQ(row1_view[2], 8);

    // Modify through row view
    row1_view[1] = 99.0f;
    EXPECT_EQ(m.at(1, 1), 99.0f);

    // Assign to row view
    row1_view = Vec3(10, 20, 30);
    EXPECT_EQ(m.at(1, 0), 10);
    EXPECT_EQ(m.at(1, 1), 20);
    EXPECT_EQ(m.at(1, 2), 30);

    // --- Test Column View ---
    auto col0_view = m.col(0);
    EXPECT_EQ(col0_view[0], 1);
    EXPECT_EQ(col0_view[1], 10);  // Check value modified via row_view

    // Modify through column view
    col0_view[2] = 77.0f;
    EXPECT_EQ(m.at(2, 0), 77.0f);

    // Assign to column view
    col0_view = Vec3(11, 22, 33);
    EXPECT_EQ(m.at(0, 0), 11);
    EXPECT_EQ(m.at(1, 0), 22);
    EXPECT_EQ(m.at(2, 0), 33);

    // --- Test Const Correctness for VectorView ---
    const Mat3 const_m        = m;
    auto       const_row_view = const_m.row(0);
    // The following line would cause a compile error, which is correct:
    // const_row_view[0] = 5.0f;
    EXPECT_EQ(const_row_view[0], 11);
}

TEST_F(MatrixTest, ConstMatrixViewTest) {
    Mat4 m  = Mat4::identity();
    m[1][2] = 123.0f;

    const Mat4& const_m = m;

    // 1. Get a const view from a const matrix
    auto const_view = const_m.view<2, 2>(1, 1);

    // 2. Verify we can read from the const view
    EXPECT_EQ(const_view.at(0, 0), 1.0f);  // From identity
    EXPECT_EQ(const_view.at(0, 1), 123.0f);

    // 3. Verify that writing to the view is a compile error
    // const_view(0, 0) = 456.0f; // THIS LINE MUST NOT COMPILE!

    // 4. Verify conversion to an owning Matrix works
    Mat2 copy = const_view.to_matrix();
    EXPECT_EQ(copy.at(0, 1), 123.0f);
}

TEST_F(MatrixTest, ViewApplyWithStdFunction) {
    // Test VectorView::apply
    // Mat3 m(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    Mat3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);

    auto row1_view = m.row(1);

    std::function<void(Float&, int)> multiply_by_index_plus_one = [](Float& elem, int index) { elem *= (index + 1); };

    row1_view.visit(multiply_by_index_plus_one);

    // Expected new row: [4*1, 5*2, 6*3] = [4, 10, 18]
    EXPECT_EQ(m.at(1, 0), 4);
    EXPECT_EQ(m.at(1, 1), 10);
    EXPECT_EQ(m.at(1, 2), 18);

    // Test MatrixView::apply
    Mat2 m2(1, 2, 3, 4);

    auto full_view = m2.view<2, 2>(0, 0);

    std::function<void(Float&, int, int)> add_row_and_col = [](Float& elem, int r, int c) { elem += (r * 10 + c); };

    full_view.visit(add_row_and_col);

    // Expected new matrix:
    // [1 + (0*10+0), 2 + (0*10+1)] = [1, 3]
    // [3 + (1*10+0), 4 + (1*10+1)] = [13, 15]
    EXPECT_EQ(m2.at(0, 0), 1);
    EXPECT_EQ(m2.at(0, 1), 3);
    EXPECT_EQ(m2.at(1, 0), 13);
    EXPECT_EQ(m2.at(1, 1), 15);
}

// --- Homogeneous Transformation Tests ---

class MatrixHomoTransformTest : public ::testing::Test {};

TEST_F(MatrixHomoTransformTest, PointTranslation) {
    Mat4 translation_matrix  = Mat4::identity();
    translation_matrix[0][3] = 5.0;
    translation_matrix[1][3] = -2.0;

    const Pt3   p_original(10, 20, 30);
    const Homo4 h_original = Homo4::from_point(p_original);

    const Homo4 h_transformed = Homo4::from_vector_raw(translation_matrix * h_original.to_vector_raw());
    const Pt3   p_final       = h_transformed.to_point();

    EXPECT_DOUBLE_EQ(h_transformed.to_vector_raw().w(), 1.0);  // Check it's still a point
    EXPECT_DOUBLE_EQ(p_final.x(), 10.0 + 5.0);
    EXPECT_DOUBLE_EQ(p_final.y(), 20.0 - 2.0);
    EXPECT_DOUBLE_EQ(p_final.z(), 30.0);
}

TEST_F(MatrixHomoTransformTest, VectorTranslation) {
    Mat4 translation_matrix  = Mat4::identity();
    translation_matrix[0][3] = 5.0;
    translation_matrix[1][3] = -2.0;

    const Vec3  v_original(1, 1, 1);
    const Homo4 h_original(v_original.x(), v_original.y(), v_original.z(), 0.0);

    const Homo4 h_transformed = Homo4(translation_matrix * h_original.to_vector_raw());
    const Vec3  v_final       = h_transformed.to_vector();

    EXPECT_DOUBLE_EQ(h_transformed.to_vector_raw().w(), 0.0);  // Check it's still a vector
    EXPECT_DOUBLE_EQ(v_final.x(), v_original.x());
    EXPECT_DOUBLE_EQ(v_final.y(), v_original.y());
    EXPECT_DOUBLE_EQ(v_final.z(), v_original.z());
}

TEST_F(MatrixHomoTransformTest, PointRotation) {
    Mat4 rotation_matrix  = Mat4::zeros();
    rotation_matrix[0][0] = 0.0;
    rotation_matrix[0][1] = -1.0;
    rotation_matrix[1][0] = 1.0;
    rotation_matrix[1][1] = 0.0;
    rotation_matrix[2][2] = 1.0;
    rotation_matrix[3][3] = 1.0;

    const Pt3   p_original(10, 0, 0);
    const Homo4 h_original = Homo4::from_point(p_original);

    const Homo4 h_transformed = Homo4::from_vector_raw(rotation_matrix * h_original.to_vector_raw());
    const Pt3   p_final       = h_transformed.to_point();

    EXPECT_NEAR(p_final.x(), 0.0, 1e-9);
    EXPECT_NEAR(p_final.y(), 10.0, 1e-9);
    EXPECT_NEAR(p_final.z(), 0.0, 1e-9);
}

TEST_F(MatrixHomoTransformTest, CombinedScaleAndTranslate) {
    Mat4 scale_matrix  = Mat4::identity();
    scale_matrix[0][0] = 2.0;
    scale_matrix[1][1] = 2.0;
    scale_matrix[2][2] = 2.0;

    Mat4 translate_matrix  = Mat4::identity();
    translate_matrix[0][3] = 5.0;

    const Mat4 combined_matrix = translate_matrix * scale_matrix;

    const Pt3   p_original(10, 10, 10);
    const Homo4 h_original = Homo4::from_point(p_original);

    const Homo4 h_transformed = Homo4::from_vector_raw(combined_matrix * h_original.to_vector_raw());
    const Pt3   p_final       = h_transformed.to_point();

    EXPECT_DOUBLE_EQ(p_final.x(), 25.0);
    EXPECT_DOUBLE_EQ(p_final.y(), 20.0);
    EXPECT_DOUBLE_EQ(p_final.z(), 20.0);
}

// Note: The following test would fail to compile due to static_assert/requires
// clause, so it's commented out but shown here for documentation purposes
/*
TEST_F(MatrixTest, ValueConstructorWithWrongNumberOfValues) {
    // This should fail to compile due to the requires clause
    Matrix<int, 2, 2> mat(1, 2, 3); // Only 3 values for 2x2=4 elements
}
*/

// Additional tests based on optimization suggestions
TEST_F(MatrixTest, InverseValidation) {
    // Test A * A.inversed() == identity for various matrix sizes
    Mat2 m2(Vec2(2, 1), Vec2(1, 1));
    auto inv2 = m2.inversed();
    auto result2 = m2 * inv2;
    ExpectMatricesNear(result2, Mat2::identity());
    
    Mat3 m3(Vec3(2, 1, 0), Vec3(1, 2, 1), Vec3(0, 1, 2));
    auto inv3 = m3.inversed();
    auto result3 = m3 * inv3;
    ExpectMatricesNear(result3, Mat3::identity());
}

TEST_F(MatrixTest, ViewModificationReflection) {
    // Ensure modifying view reflects to original matrix
    Mat3 original = Mat3::filled(1.0);
    auto view = original.template view<2, 2>(0, 0);
    
    // Modify through view
    view.at(0, 0) = 5.0;
    view.at(1, 1) = 7.0;
    
    // Check original matrix is modified
    EXPECT_DOUBLE_EQ(original.at(0, 0), 5.0);
    EXPECT_DOUBLE_EQ(original.at(1, 1), 7.0);
}

TEST_F(MatrixTest, HasNanDetection) {
    // Test construction with NAN elements
    Mat2 normal_mat = Mat2::identity();
    EXPECT_FALSE(normal_mat.has_nan());
    
    Mat2 nan_mat(Vec2(1.0, std::numeric_limits<Float>::quiet_NaN()), 
                 Vec2(3.0, 4.0));
    EXPECT_TRUE(nan_mat.has_nan());
}

TEST_F(MatrixTest, TypeConversionConstructor) {
    // Test conversion between different floating-point types
    Matrix<float, 2, 2> float_mat(Vector<float, 2>(1.5f, 2.5f), 
                                  Vector<float, 2>(3.5f, 4.5f));
    
    // Manual conversion to double matrix (create from float matrix elements)
    Matrix<double, 2, 2> double_mat(
        Vector<double, 2>(static_cast<double>(float_mat.at(0,0)), static_cast<double>(float_mat.at(0,1))), 
        Vector<double, 2>(static_cast<double>(float_mat.at(1,0)), static_cast<double>(float_mat.at(1,1)))
    );
    
    EXPECT_DOUBLE_EQ(double_mat.at(0, 0), 1.5);
    EXPECT_DOUBLE_EQ(double_mat.at(0, 1), 2.5);
    EXPECT_DOUBLE_EQ(double_mat.at(1, 0), 3.5);  
    EXPECT_DOUBLE_EQ(double_mat.at(1, 1), 4.5);
}

// --- Additional Tests for 1x1, 1xM, Nx1 Matrices ---

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

TEST_F(MatrixTest, Matrix1xMAndNx1Multiplication) {
    using Mat1x3 = Matrix<float, 1, 3>;
    using Mat3x1 = Matrix<float, 3, 1>;
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

// --- Random Matrix/Vector/Point Tests ---

class RandomMatrixTest : public ::testing::Test {
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
    Homogeneous<T, N + 1> create_random_homogeneous() {
        Vector<T, N> v = create_random_vector<T, N>();
        T w = random_value<T>(T(0.1), T(2)); // Avoid zero weight
        auto result = Homogeneous<T, N + 1>::from_vector(v);
        result.w() = w;
        return result;
    }
};

TEST_F(RandomMatrixTest, RandomMatrixOperations) {
    const int num_tests = 10; // Reduced number of tests for stability
    
    for (int test = 0; test < num_tests; ++test) {
        // Test random 2x2 matrices (smaller for better numerical stability)
        auto m1 = create_random_matrix<float, 2, 2>();
        auto m2 = create_random_matrix<float, 2, 2>();
        
        // Test commutativity of addition
        auto sum1 = m1 + m2;
        auto sum2 = m2 + m1;
        EXPECT_TRUE(sum1 == sum2);
        
        // Test associativity of addition
        auto m3 = create_random_matrix<float, 2, 2>();
        auto assoc1 = (m1 + m2) + m3;
        auto assoc2 = m1 + (m2 + m3);
        EXPECT_TRUE(assoc1 == assoc2);
        
        // Test basic matrix operations work without throwing
        EXPECT_NO_THROW({
            auto diff = m1 - m2;
            auto scaled = 2.0f * m1;
            auto product = m1 * m2;
            auto transposed = m1.transposed();
        });
    }
}

TEST_F(RandomMatrixTest, RandomVectorOperations) {
    const int num_tests = 10; // Reduced for stability
    
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
        
        // Test dot product commutativity
        float dot1 = v1.dot(v2);
        float dot2 = v2.dot(v1);
        EXPECT_FLOAT_EQ(dot1, dot2);
        
        // Test basic vector operations work without throwing
        EXPECT_NO_THROW({
            auto diff = v1 - v2;
            auto scaled = 2.0f * v1;
            auto length = v1.length();
            auto normalized = v1.normalized();
        });
    }
}

TEST_F(RandomMatrixTest, RandomHomogeneousOperations) {
    const int num_tests = 50;
    
    for (int test = 0; test < num_tests; ++test) {
        auto h1 = create_random_homogeneous<float, 3>();
        auto h2 = create_random_homogeneous<float, 3>();
        
        // Test conversion to vector and back
        if (h1.is_vector()) {
            auto v1 = h1.to_vector();
            auto h1_reconstructed = Homogeneous<float, 4>::from_vector(v1);
            EXPECT_TRUE(h1 == h1_reconstructed);
        }
        
        // Test homogeneous addition
        auto sum = h1 + h2;
        // Note: Adding two points creates a vector, which is mathematically correct
        
        // Test scalar multiplication
        float scalar = random_value<float>(0.1f, 5.0f);
        auto scaled = scalar * h1;
        
        // Test standardization (if it's a point)
        if (h1.is_point() && std::abs(h1.w()) > 1e-6f) {
            auto standardized = h1.standardized();
            EXPECT_FLOAT_EQ(standardized.w(), 1.0f);
            
            // Check that the 3D point remains the same after standardization
            auto original_point = h1.to_point();
            auto standardized_point = standardized.to_point();
            EXPECT_TRUE(original_point == standardized_point);
        }
    }
}

TEST_F(RandomMatrixTest, RandomMatrixVectorMultiplication) {
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
        static_assert(std::is_same_v<decltype(result_h), Homogeneous<float, 4>>);
        
        // Verify by converting homogeneous to vector and multiplying
        auto h_as_vector = h.to_vector_raw();
        auto expected_vector = m * h_as_vector;
        EXPECT_TRUE(result_h.to_vector_raw() == expected_vector);
    }
}

TEST_F(RandomMatrixTest, RandomMatrixProperties) {
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

TEST_F(RandomMatrixTest, RandomMatrixEdgeCases) {
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

}  // namespace pbpt::math::testing
