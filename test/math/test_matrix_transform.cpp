#include <type_traits>

#include "gtest/gtest.h"
#include "pbpt/pbpt.h"

namespace pbpt::math::testing {

template <typename T, int R, int C>
static void expect_matrices_near(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs, T eps = T(1e-6)) {
    for (int row = 0; row < R; ++row) {
        for (int col = 0; col < C; ++col) {
            EXPECT_NEAR(lhs.at(row, col), rhs.at(row, col), eps);
        }
    }
}

template <typename TM, typename TP>
static Point<TM, 3> mat_apply_point(const Matrix<TM, 4, 4>& m, const Point<TP, 3>& p) {
    const auto h = m * Vector<TM, 4>(TM(p.x()), TM(p.y()), TM(p.z()), TM(1));
    const TM   inv_w = TM(1) / h[3];
    return Point<TM, 3>(h[0] * inv_w, h[1] * inv_w, h[2] * inv_w);
}

TEST(MatrixTransformTest, TranslateAndScaleReturnTypedMatrices) {
    const auto translation = translate(Vector<double, 3>(1.5, -2.0, 4.25));
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(translation)>, Matrix<double, 4, 4>>);
    expect_matrices_near(translation,
                         Matrix<double, 4, 4>(1.0, 0.0, 0.0, 1.5, 0.0, 1.0, 0.0, -2.0, 0.0, 0.0, 1.0, 4.25, 0.0,
                                              0.0, 0.0, 1.0));

    const auto uniform_scale = scale(2.0f);
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(uniform_scale)>, Matrix<float, 4, 4>>);
    expect_matrices_near(uniform_scale, Matrix<float, 4, 4>(2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f,
                                                             0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
}

TEST(MatrixTransformTest, AxisAngleRotationMatchesSpecializedRotation) {
    const double angle = deg2rad(90.0);
    const auto   axis_rotation = rotate(angle, Vector<double, 3>(0.0, 0.0, 1.0));
    const auto   z_rotation = rotate_z(angle);
    expect_matrices_near(axis_rotation, z_rotation);
}

TEST(MatrixTransformTest, QuaternionRotationMatchesAxisAngleRotation) {
    const double              angle = deg2rad(37.0);
    const Vector<double, 3>   axis(1.0, 2.0, 3.0);
    const Quaternion<double>  q = Quaternion<double>::from_axis_angle(angle, axis);
    const auto                q_rotation = rotate(q);
    const auto                axis_rotation = rotate(angle, axis);
    expect_matrices_near(q_rotation, axis_rotation, 1e-9);
}

TEST(MatrixTransformTest, FromMat3x3PromotesToAffineMat4) {
    const Matrix<double, 3, 3> basis(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    const auto                 promoted = from_mat3x3(basis);
    expect_matrices_near(promoted, Matrix<double, 4, 4>(1.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0, 0.0, 7.0, 8.0, 9.0, 0.0,
                                                         0.0, 0.0, 0.0, 1.0));
}

TEST(MatrixTransformTest, ComposeTrsMatchesTranslateRotateScaleChain) {
    const Vector<double, 3>  translation(1.5, -2.0, 4.25);
    const Quaternion<double> rotation = Quaternion<double>::from_axis_angle(deg2rad(33.0), Vector<double, 3>(1.0, 2.0, 3.0));
    const Vector<double, 3>  scaling(2.0, 0.5, 1.25);

    const auto composed = compose_trs(translation, rotation, scaling);
    const auto chained = translate(translation) * rotate(rotation) * scale(scaling);
    expect_matrices_near(composed, chained, 1e-9);
}

TEST(MatrixTransformTest, LookAtVectorAndPointOverloadsMatch) {
    const auto point_view =
        look_at(Point<float, 3>(1.0f, 2.0f, 3.0f), Point<float, 3>(1.0f, 2.0f, 2.0f), Vector<float, 3>(0.0f, 1.0f, 0.0f));
    const auto vector_view =
        look_at(Vector<float, 3>(1.0f, 2.0f, 3.0f), Vector<float, 3>(1.0f, 2.0f, 2.0f), Vector<float, 3>(0.0f, 1.0f, 0.0f));
    expect_matrices_near(point_view, vector_view);
}

TEST(MatrixTransformTest, ExtractHelpersRecoverTrsComponents) {
    const Vector<double, 3>  translation(3.0, -5.0, 7.5);
    const Quaternion<double> rotation = Quaternion<double>::from_axis_angle(deg2rad(41.0), Vector<double, 3>(0.25, 1.0, -0.5));
    const Vector<double, 3>  scaling(1.5, 2.5, 0.75);
    const auto               trs = compose_trs(translation, rotation, scaling);

    const auto extracted_translation = extract_translation(trs);
    const auto extracted_scale = extract_scale(trs);
    const auto extracted_rotation = extract_rotation(trs);

    EXPECT_NEAR(extracted_translation.x(), translation.x(), 1e-9);
    EXPECT_NEAR(extracted_translation.y(), translation.y(), 1e-9);
    EXPECT_NEAR(extracted_translation.z(), translation.z(), 1e-9);

    EXPECT_NEAR(extracted_scale.x(), scaling.x(), 1e-9);
    EXPECT_NEAR(extracted_scale.y(), scaling.y(), 1e-9);
    EXPECT_NEAR(extracted_scale.z(), scaling.z(), 1e-9);

    const auto expected_rotation = rotate(rotation);
    const auto actual_rotation = rotate(extracted_rotation);
    expect_matrices_near(actual_rotation, expected_rotation, 1e-9);
}

TEST(MatrixTransformTest, OrthographicMapsNearAndFarDepths) {
    const auto ortho = orthographic(-10.0, 10.0, -5.0, 5.0, 1.0, 101.0);
    const auto near_ndc = mat_apply_point(ortho, Point<double, 3>(-10.0, -5.0, 1.0));
    const auto far_ndc = mat_apply_point(ortho, Point<double, 3>(10.0, 5.0, 101.0));

    EXPECT_NEAR(near_ndc.x(), -1.0, 1e-6);
    EXPECT_NEAR(near_ndc.y(), -1.0, 1e-6);
    EXPECT_NEAR(near_ndc.z(), 0.0, 1e-6);
    EXPECT_NEAR(far_ndc.x(), 1.0, 1e-6);
    EXPECT_NEAR(far_ndc.y(), 1.0, 1e-6);
    EXPECT_NEAR(far_ndc.z(), 1.0, 1e-6);
}

TEST(MatrixTransformTest, PerspectiveMatchesTransformFactory) {
    const auto matrix_perspective = perspective(deg2rad(60.0), 1.5, 0.5, 50.0);
    const auto transform_perspective = geometry::Transform<double>::perspective(deg2rad(60.0), 1.5, 0.5, 50.0).matrix();
    expect_matrices_near(matrix_perspective, transform_perspective, 1e-9);
}

TEST(MatrixTransformTest, ViewportMapsClipToRasterSpace) {
    const auto vp = viewport(800.0f, 600.0f);
    expect_matrices_near(vp, Matrix<float, 4, 4>(400.0f, 0.0f, 0.0f, 400.0f, 0.0f, 300.0f, 0.0f, 300.0f, 0.0f, 0.0f,
                                                  1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));
}

}  // namespace pbpt::math::testing
