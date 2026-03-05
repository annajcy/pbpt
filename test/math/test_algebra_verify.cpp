#include <gtest/gtest.h>

#include <type_traits>

#include "pbpt/pbpt.h"

namespace pbpt::math::test {

namespace {

template <typename T>
void expect_vec3_approx(const T& lhs, const T& rhs) {
    EXPECT_TRUE(is_approx(lhs, rhs));
}

template <typename T, int R, int C>
void expect_matrix_approx(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) {
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            EXPECT_TRUE(is_equal(lhs.at(r, c), rhs.at(r, c)));
        }
    }
}

}  // namespace

TEST(AlgebraVerifyConcepts, StructuralContracts) {
    static_assert(MathObject<Vec3>);
    static_assert(MetricVectorSpace<Vec3, Float>);
    static_assert(VectorSpace3D<Vec3, Float>);
    static_assert(CovectorSpace<Normal3, Vec3, Float>);
    static_assert(AffineSpace<Pt3, Vec3, Float>);
    static_assert(MatrixAlgebra<Mat4, Float>);

    static_assert(!AdditiveMonoid<Pt3>);
    static_assert(!Scalable<Pt3, Float>);
    static_assert(!InnerProduct<Normal3, Float>);
}

TEST(AlgebraVerifyVectorLaws, AdditionCommutativeAndAssociative) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, -1.0f, 2.0f);
    const Vec3 c(-3.0f, 5.0f, 1.0f);

    expect_vec3_approx(a + b, b + a);
    expect_vec3_approx((a + b) + c, a + (b + c));
}

TEST(AlgebraVerifyVectorLaws, ScalarDistributiveAndCompatible) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, -1.0f, 2.0f);
    const Float s = 2.5f;
    const Float t = -0.75f;

    expect_vec3_approx(s * (a + b), s * a + s * b);
    expect_vec3_approx((s + t) * a, s * a + t * a);
    expect_vec3_approx((s * t) * a, s * (t * a));
}

TEST(AlgebraVerifyVectorLaws, CrossProductAntiCommutativeAndDistributive) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, -1.0f, 2.0f);
    const Vec3 c(-3.0f, 5.0f, 1.0f);

    expect_vec3_approx(cross(a, b), -cross(b, a));
    expect_vec3_approx(cross(a + b, c), cross(a, c) + cross(b, c));
}

TEST(AlgebraVerifyPointLaws, AffineConsistency) {
    const Pt3 p(1.0f, 2.0f, 3.0f);
    const Pt3 q(4.0f, 5.0f, 6.0f);
    const Vec3 v(-2.0f, 1.0f, 3.0f);

    const Pt3 q_roundtrip = p + (q - p);
    expect_vec3_approx(q_roundtrip.to_vector(), q.to_vector());

    expect_vec3_approx((p + v) - p, v);
    EXPECT_TRUE(is_equal(distance(p, q), distance(q, p)));
}

TEST(AlgebraVerifyMatrixLaws, AdditionCommutativeAndAssociative) {
    const Mat3 a(1.0f, 2.0f, 3.0f,
                 0.0f, 1.0f, 4.0f,
                 5.0f, 6.0f, 0.0f);
    const Mat3 b(2.0f, 0.0f, 1.0f,
                 1.0f, 3.0f, 0.0f,
                 4.0f, 1.0f, 2.0f);
    const Mat3 c(0.5f, 1.0f, 0.0f,
                 2.0f, 0.0f, 1.0f,
                 1.0f, 1.5f, 3.0f);

    expect_matrix_approx(a + b, b + a);
    expect_matrix_approx((a + b) + c, a + (b + c));
}

TEST(AlgebraVerifyMatrixLaws, MultiplicationAssociativeAndDistributive) {
    const Mat3 a(1.0f, 2.0f, 3.0f,
                 0.0f, 1.0f, 4.0f,
                 5.0f, 6.0f, 0.0f);
    const Mat3 b(2.0f, 0.0f, 1.0f,
                 1.0f, 3.0f, 0.0f,
                 4.0f, 1.0f, 2.0f);
    const Mat3 c(0.5f, 1.0f, 0.0f,
                 2.0f, 0.0f, 1.0f,
                 1.0f, 1.5f, 3.0f);

    expect_matrix_approx((a * b) * c, a * (b * c));
    expect_matrix_approx(a * (b + c), a * b + a * c);
    expect_matrix_approx((a + b) * c, a * c + b * c);
}

TEST(AlgebraVerifyMatrixLaws, MultiplicativeIdentity) {
    const Mat4 m(1.0f, 2.0f, 3.0f, 4.0f,
                 0.0f, 1.0f, 4.0f, 5.0f,
                 0.0f, 0.0f, 2.0f, 6.0f,
                 0.0f, 0.0f, 0.0f, 1.0f);
    const Mat4 i = Mat4::identity();

    expect_matrix_approx(m * i, m);
    expect_matrix_approx(i * m, m);
}

}  // namespace pbpt::math::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
