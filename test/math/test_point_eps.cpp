/**
 * @file test_point_eps.cpp
 * @brief Unit tests for PointEps (Point with Interval coordinates)
 */

#include <gtest/gtest.h>
#include "math/point.hpp"
#include "math/interval.hpp"

namespace pbpt::math::testing {

class PointEpsTest : public ::testing::Test {
protected:
    // Test intervals
    Interval<float> interval1{1.0f, 2.0f};
    Interval<float> interval2{2.0f, 3.0f};
    Interval<float> interval3{0.5f, 1.5f};
    
    // Test points with intervals
    PointInterval<float, 3> point1{interval1, interval2, interval3};
    PointInterval<float, 3> point2{Interval<float>{2.0f, 3.0f}, 
                              Interval<float>{1.0f, 2.0f}, 
                              Interval<float>{3.0f, 4.0f}};
};

TEST_F(PointEpsTest, Construction) {
    // Default construction
    PointInterval<float, 3> default_point;
    EXPECT_TRUE(default_point[0].is_empty());
    EXPECT_TRUE(default_point[1].is_empty());
    EXPECT_TRUE(default_point[2].is_empty());
    
    // Explicit construction
    EXPECT_EQ(point1[0].m_low, 1.0f);
    EXPECT_EQ(point1[0].m_high, 2.0f);
    EXPECT_EQ(point1[1].m_low, 2.0f);
    EXPECT_EQ(point1[1].m_high, 3.0f);
}

TEST_F(PointEpsTest, VectorConversion) {
    // 创建一个精确向量
    Vector<float, 3> exact_vec{1.5f, 2.5f, 1.0f};
    
    // 将向量转换为PointEps（每个分量都是精确值的区间）
    auto point_from_vec = PointInterval<float, 3>::from_vector(
        Vector<Interval<float>, 3>{
            Interval<float>{1.5f}, 
            Interval<float>{2.5f}, 
            Interval<float>{1.0f}
        }
    );
    
    EXPECT_EQ(point_from_vec[0].m_low, 1.5f);
    EXPECT_EQ(point_from_vec[0].m_high, 1.5f);
    EXPECT_EQ(point_from_vec[1].m_low, 2.5f);
    EXPECT_EQ(point_from_vec[1].m_high, 2.5f);
}

TEST_F(PointEpsTest, PointArithmetic) {
    // Point + Vector = Point
    Vector<Interval<float>, 3> vec{
        Interval<float>{0.5f, 1.0f}, 
        Interval<float>{-1.0f, 0.0f}, 
        Interval<float>{1.0f, 2.0f}
    };
    
    auto result = point1 + vec;
    
    // 检查结果的区间
    EXPECT_EQ(result[0].m_low, 1.5f);  // 1.0 + 0.5
    EXPECT_EQ(result[0].m_high, 3.0f); // 2.0 + 1.0
    EXPECT_EQ(result[1].m_low, 1.0f);  // 2.0 + (-1.0)
    EXPECT_EQ(result[1].m_high, 3.0f); // 3.0 + 0.0
}

TEST_F(PointEpsTest, PointSubtraction) {
    // Point - Point = Vector
    auto diff = point1 - point2;
    
    // 区间减法：[a,b] - [c,d] = [a-d, b-c]
    // point1[0] = [1,2], point2[0] = [2,3]
    // [1,2] - [2,3] = [1-3, 2-2] = [-2, 0]
    EXPECT_EQ(diff[0].m_low, -2.0f);  // 1.0 - 3.0
    EXPECT_EQ(diff[0].m_high, 0.0f);  // 2.0 - 2.0
    
    // point1[1] = [2,3], point2[1] = [1,2] 
    // [2,3] - [1,2] = [2-2, 3-1] = [0, 2]
    EXPECT_EQ(diff[1].m_low, 0.0f);   // 2.0 - 2.0
    EXPECT_EQ(diff[1].m_high, 2.0f);  // 3.0 - 1.0
}

TEST_F(PointEpsTest, Distance) {
    // 创建两个简单的点进行距离测试
    PointInterval<float, 2> p1{Interval<float>{0.0f, 1.0f}, Interval<float>{0.0f, 1.0f}};
    PointInterval<float, 2> p2{Interval<float>{3.0f, 4.0f}, Interval<float>{4.0f, 5.0f}};
    
    auto dist_squared = p1.distance_squared(p2);
    
    // 距离的平方也应该是一个区间
    // 最小距离：从p1的最大值到p2的最小值
    // 最大距离：从p1的最小值到p2的最大值
    // (3-1)^2 + (4-1)^2 = 4 + 9 = 13 (最小)
    // (4-0)^2 + (5-0)^2 = 16 + 25 = 41 (最大)
    EXPECT_EQ(dist_squared.m_low, 13.0f);
    EXPECT_EQ(dist_squared.m_high, 41.0f);
}

TEST_F(PointEpsTest, Midpoint) {
    auto mid = point1.mid(point2);
    
    // 中点的每个分量也是区间
    EXPECT_EQ(mid[0].m_low, 1.5f);   // (1+2)/2
    EXPECT_EQ(mid[0].m_high, 2.5f);  // (2+3)/2
    EXPECT_EQ(mid[1].m_low, 1.5f);   // (2+1)/2
    EXPECT_EQ(mid[1].m_high, 2.5f);  // (3+2)/2
}

TEST_F(PointEpsTest, Clamp) {
    PointInterval<float, 3> low_bound{
        Interval<float>{0.0f, 0.5f}, 
        Interval<float>{1.0f, 1.5f}, 
        Interval<float>{0.0f, 1.0f}
    };
    
    PointInterval<float, 3> high_bound{
        Interval<float>{2.5f, 3.0f}, 
        Interval<float>{3.5f, 4.0f}, 
        Interval<float>{2.0f, 3.0f}
    };
    
    auto clamped = point1.clamp(low_bound, high_bound);
    
    // point1[0] = [1,2] 在 [0.5,3] 范围内，所以保持不变
    EXPECT_EQ(clamped[0].m_low, 1.0f);
    EXPECT_EQ(clamped[0].m_high, 2.0f);
    
    // point1[1] = [2,3] 在 [1.5,4] 范围内，所以保持不变
    EXPECT_EQ(clamped[1].m_low, 2.0f);
    EXPECT_EQ(clamped[1].m_high, 3.0f);
}

TEST_F(PointEpsTest, TypeAliases) {
    // 测试项目提供的类型别名
    Pt3Interv point_eps;
    EXPECT_TRUE(point_eps[0].is_empty());
    EXPECT_TRUE(point_eps[1].is_empty());
    EXPECT_TRUE(point_eps[2].is_empty());
    
    Pt3Interv point_with_values{
        Interval<float>::from_value_with_error(1.0f, 0.1f),
        Interval<float>::from_value_with_error(2.0f, 0.1f),
        Interval<float>::from_value_with_error(3.0f, 0.1f)
    };
    
    EXPECT_NEAR(point_with_values[0].midpoint(), 1.0f, 1e-6f);
    EXPECT_NEAR(point_with_values[1].midpoint(), 2.0f, 1e-6f);
    EXPECT_NEAR(point_with_values[2].midpoint(), 3.0f, 1e-6f);
    
    EXPECT_NEAR(point_with_values[0].width(), 0.2f, 1e-6f);
    EXPECT_NEAR(point_with_values[1].width(), 0.2f, 1e-6f);
    EXPECT_NEAR(point_with_values[2].width(), 0.2f, 1e-6f);
}

TEST_F(PointEpsTest, ErrorPropagation) {
    // 测试误差传播
    PointInterval<float, 2> p1{
        Interval<float>::from_value_with_error(1.0f, 0.1f),  // [0.9, 1.1]
        Interval<float>::from_value_with_error(2.0f, 0.1f)   // [1.9, 2.1]
    };
    
    PointInterval<float, 2> p2{
        Interval<float>::from_value_with_error(3.0f, 0.1f),  // [2.9, 3.1]
        Interval<float>::from_value_with_error(4.0f, 0.1f)   // [3.9, 4.1]
    };
    
    auto diff = p2 - p1;
    
    // 误差应该合理传播
    // x方向：[2.9, 3.1] - [0.9, 1.1] = [2.9-1.1, 3.1-0.9] = [1.8, 2.2]
    EXPECT_NEAR(diff[0].m_low, 1.8f, 1e-6f);
    EXPECT_NEAR(diff[0].m_high, 2.2f, 1e-6f);
    
    // y方向：[3.9, 4.1] - [1.9, 2.1] = [3.9-2.1, 4.1-1.9] = [1.8, 2.2]
    EXPECT_NEAR(diff[1].m_low, 1.8f, 1e-6f);
    EXPECT_NEAR(diff[1].m_high, 2.2f, 1e-6f);
}

}  // namespace pbpt::math::testing
