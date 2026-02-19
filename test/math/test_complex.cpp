/**
 * @file test_complex.cpp
 * @brief Unit tests for the Complex class
 */

#include <gtest/gtest.h>
#include <sstream>
#include <cmath>

#include "pbpt/math/complex.hpp"
#include "pbpt/math/format.hpp"

namespace pbpt::math::testing {

class ComplexTest : public ::testing::Test {
protected:
    Complex<double> c1{1.0, 2.0};
    Complex<double> c2{3.0, 4.0};
};

TEST_F(ComplexTest, DefaultConstruction) {
    Complex<double> c;
    EXPECT_EQ(c.real(), 0.0);
    EXPECT_EQ(c.imag(), 0.0);
}

TEST_F(ComplexTest, ValueConstruction) {
    EXPECT_EQ(c1.real(), 1.0);
    EXPECT_EQ(c1.imag(), 2.0);
}

TEST_F(ComplexTest, Accessors) {
    Complex<double> c;
    c.real() = 5.0;
    c.imag() = 6.0;
    EXPECT_EQ(c.real(), 5.0);
    EXPECT_EQ(c.imag(), 6.0);
}

TEST_F(ComplexTest, Addition) {
    Complex<double> res = c1 + c2;
    EXPECT_EQ(res.real(), 4.0);
    EXPECT_EQ(res.imag(), 6.0);
}

TEST_F(ComplexTest, Subtraction) {
    Complex<double> res = c1 - c2;
    EXPECT_EQ(res.real(), -2.0);
    EXPECT_EQ(res.imag(), -2.0);
}

TEST_F(ComplexTest, MultiplicationComplex) {
    // (1 + 2i) * (3 + 4i) = 3 + 4i + 6i - 8 = -5 + 10i
    Complex<double> res = c1 * c2;
    EXPECT_EQ(res.real(), -5.0);
    EXPECT_EQ(res.imag(), 10.0);
}

TEST_F(ComplexTest, MultiplicationScalar) {
    Complex<double> res = c1 * 2.0;
    EXPECT_EQ(res.real(), 2.0);
    EXPECT_EQ(res.imag(), 4.0);

    Complex<double> res2 = 2.0 * c1;
    EXPECT_EQ(res2.real(), 2.0);
    EXPECT_EQ(res2.imag(), 4.0);
}

TEST_F(ComplexTest, DivisionComplex) {
    // (1 + 2i) / (3 + 4i) = ((1+2i)(3-4i)) / 25 = (3 - 4i + 6i + 8) / 25 = (11 + 2i) / 25
    Complex<double> res = c1 / c2;
    EXPECT_DOUBLE_EQ(res.real(), 11.0 / 25.0);
    EXPECT_DOUBLE_EQ(res.imag(), 2.0 / 25.0);
}

TEST_F(ComplexTest, DivisionScalar) {
    Complex<double> res = c1 / 2.0;
    EXPECT_EQ(res.real(), 0.5);
    EXPECT_EQ(res.imag(), 1.0);
}

TEST_F(ComplexTest, UnaryMinus) {
    Complex<double> res = -c1;
    EXPECT_EQ(res.real(), -1.0);
    EXPECT_EQ(res.imag(), -2.0);
}

TEST_F(ComplexTest, CompoundAssignment) {
    Complex<double> c = c1;
    c += c2;
    EXPECT_EQ(c.real(), 4.0);
    EXPECT_EQ(c.imag(), 6.0);

    c = c1;
    c -= c2;
    EXPECT_EQ(c.real(), -2.0);
    EXPECT_EQ(c.imag(), -2.0);

    c = c1;
    c *= c2;
    EXPECT_EQ(c.real(), -5.0);
    EXPECT_EQ(c.imag(), 10.0);

    c = c1;
    c /= c2;
    EXPECT_DOUBLE_EQ(c.real(), 11.0 / 25.0);
    EXPECT_DOUBLE_EQ(c.imag(), 2.0 / 25.0);

    c = c1;
    c *= 2.0;
    EXPECT_EQ(c.real(), 2.0);
    EXPECT_EQ(c.imag(), 4.0);

    c = c1;
    c /= 2.0;
    EXPECT_EQ(c.real(), 0.5);
    EXPECT_EQ(c.imag(), 1.0);
}

TEST_F(ComplexTest, Magnitude) {
    // (3, 4) magnitude is 5
    EXPECT_EQ(c2.length(), 5.0);
}

TEST_F(ComplexTest, Conjugate) {
    Complex<double> conj = c1.conjugate();
    EXPECT_EQ(conj.real(), 1.0);
    EXPECT_EQ(conj.imag(), -2.0);
}

TEST_F(ComplexTest, FromToVector) {
    Vector<double, 2> vec(1.0, 2.0);
    Complex<double> c = Complex<double>::from_vector(vec);
    EXPECT_EQ(c.real(), 1.0);
    EXPECT_EQ(c.imag(), 2.0);

    Vector<double, 2> back = c.to_vector();
    EXPECT_EQ(back.x(), 1.0);
    EXPECT_EQ(back.y(), 2.0);
}

TEST_F(ComplexTest, FromPolar) {
    Complex<double> c = Complex<double>::from_polar(5.0, 0.0);  // 5 angle 0
    EXPECT_DOUBLE_EQ(c.real(), 5.0);
    EXPECT_NEAR(c.imag(), 0.0, 1e-10);  // slightly fuzzy for sin(0) maybe?

    // 5 at 90 degrees (pi/2) -> 5i
    Complex<double> c_pi_2 = Complex<double>::from_polar(5.0, M_PI / 2.0);
    EXPECT_NEAR(c_pi_2.real(), 0.0, 1e-10);
    EXPECT_DOUBLE_EQ(c_pi_2.imag(), 5.0);
}

TEST_F(ComplexTest, Exp) {
    // e^(0 + i*pi) = -1
    Complex<double> z(0.0, M_PI);
    Complex<double> res = z.exp();
    EXPECT_NEAR(res.real(), -1.0, 1e-10);
    EXPECT_NEAR(res.imag(), 0.0, 1e-10);
}

TEST_F(ComplexTest, ToString) {
    Complex<double> c(1.5, -2.5);
    std::string s = to_string(c);
    // Format: Complex<type>(real, imag)
    // Check if it contains "Complex<" and "1.5" and "-2.5"
    EXPECT_NE(s.find("Complex<"), std::string::npos);
    EXPECT_NE(s.find("1.500000"), std::string::npos);
    EXPECT_NE(s.find("-2.500000"), std::string::npos);
}

TEST_F(ComplexTest, StreamOperator) {
    Complex<double> c(1.5, -2.5);
    std::stringstream ss;
    ss << c;
    std::string s = ss.str();
    EXPECT_NE(s.find("Complex<"), std::string::npos);
}

TEST_F(ComplexTest, ConstexprCheck) {
    // Verify constexpr constructors and basic ops work at compile time
    constexpr Complex<double> c1(1.0, 2.0);
    constexpr Complex<double> c2(3.0, 4.0);
    constexpr Complex<double> res = c1 + c2;
    static_assert(res.real() == 4.0, "Constexpr addition failed");
    static_assert(res.imag() == 6.0, "Constexpr addition failed");

    constexpr double magSq = c2.real() * c2.real() + c2.imag() * c2.imag();
    static_assert(magSq == 25.0, "Constexpr check failed");
}

}  // namespace pbpt::math::testing
