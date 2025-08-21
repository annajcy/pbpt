#include <gtest/gtest.h>
#include <type_traits>
#include <limits>

#include "core/radiometry.hpp"

using namespace pbpt::core;
using namespace pbpt::math;

class RadiometryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试用的常量值
        epsilon = 1e-6f;
        test_value_1 = 42.5f;
        test_value_2 = 17.3f;
        test_scalar = 2.5f;
        zero_value = 0.0f;
    }

    float epsilon;
    float test_value_1;
    float test_value_2;
    float test_scalar;
    float zero_value;
};

// =============================================================================
// 基础构造和访问测试
// =============================================================================

TEST_F(RadiometryTest, BasicConstruction) {
    // 测试默认构造
    Radiance<float> default_radiance;
    EXPECT_FLOAT_EQ(default_radiance.value(), 0.0f);
    
    // 测试有参构造
    Radiance<float> radiance(test_value_1);
    EXPECT_FLOAT_EQ(radiance.value(), test_value_1);
    
    // 测试其他类型
    Flux<double> flux(100.5);
    EXPECT_DOUBLE_EQ(flux.value(), 100.5);
    
    Intensity<float> intensity(50.0f);
    EXPECT_FLOAT_EQ(intensity.value(), 50.0f);
    
    Irradiance<float> irradiance(75.2f);
    EXPECT_FLOAT_EQ(irradiance.value(), 75.2f);
    
    Area<float> area(25.8f);
    EXPECT_FLOAT_EQ(area.value(), 25.8f);
    
    SolidAngle<float> solid_angle(3.14f);
    EXPECT_FLOAT_EQ(solid_angle.value(), 3.14f);
}

TEST_F(RadiometryTest, ValueAccessors) {
    Radiance<float> radiance(test_value_1);
    
    // 测试 const 访问器
    const auto& const_radiance = radiance;
    EXPECT_FLOAT_EQ(const_radiance.value(), test_value_1);
    
    // 测试非 const 访问器和修改
    radiance.value() = test_value_2;
    EXPECT_FLOAT_EQ(radiance.value(), test_value_2);
}

// =============================================================================
// 算术运算符测试
// =============================================================================

TEST_F(RadiometryTest, Addition) {
    Radiance<float> r1(test_value_1);
    Radiance<float> r2(test_value_2);
    
    auto result = r1 + r2;
    EXPECT_FLOAT_EQ(result.value(), test_value_1 + test_value_2);
    
    // 测试类型提升
    Radiance<float> rf(10.5f);
    Radiance<double> rd(20.3);
    auto promoted_result = rf + rd;
    static_assert(std::is_same_v<decltype(promoted_result), Radiance<double>>);
    EXPECT_DOUBLE_EQ(promoted_result.value(), 10.5 + 20.3);
}

TEST_F(RadiometryTest, Subtraction) {
    Radiance<float> r1(test_value_1);
    Radiance<float> r2(test_value_2);
    
    auto result = r1 - r2;
    EXPECT_FLOAT_EQ(result.value(), test_value_1 - test_value_2);
    
    // 测试类型提升
    Radiance<float> rf(30.5f);
    Radiance<double> rd(10.3);
    auto promoted_result = rf - rd;
    static_assert(std::is_same_v<decltype(promoted_result), Radiance<double>>);
    EXPECT_DOUBLE_EQ(promoted_result.value(), 30.5 - 10.3);
}

TEST_F(RadiometryTest, ScalarMultiplication) {
    Radiance<float> radiance(test_value_1);
    
    // 测试右乘
    auto result1 = radiance * test_scalar;
    EXPECT_FLOAT_EQ(result1.value(), test_value_1 * test_scalar);
    
    // 测试左乘
    auto result2 = test_scalar * radiance;
    EXPECT_FLOAT_EQ(result2.value(), test_value_1 * test_scalar);
    
    // 测试类型提升
    Radiance<float> rf(10.0f);
    double scalar = 2.5;
    auto promoted_result = rf * scalar;
    static_assert(std::is_same_v<decltype(promoted_result), Radiance<double>>);
    EXPECT_DOUBLE_EQ(promoted_result.value(), 10.0 * 2.5);
}

TEST_F(RadiometryTest, ScalarDivision) {
    Radiance<float> radiance(test_value_1);
    
    auto result = radiance / test_scalar;
    EXPECT_FLOAT_EQ(result.value(), test_value_1 / test_scalar);
    
    // 测试类型提升
    Radiance<float> rf(20.0f);
    double scalar = 4.0;
    auto promoted_result = rf / scalar;
    static_assert(std::is_same_v<decltype(promoted_result), Radiance<double>>);
    EXPECT_DOUBLE_EQ(promoted_result.value(), 20.0 / 4.0);
}

TEST_F(RadiometryTest, DivisionByZero) {
    Radiance<float> radiance(test_value_1);
    
    // 除以零应该抛出异常
    EXPECT_THROW(radiance / zero_value, std::exception);
}

// =============================================================================
// 赋值运算符测试
// =============================================================================

TEST_F(RadiometryTest, AdditionAssignment) {
    Radiance<float> r1(test_value_1);
    Radiance<float> r2(test_value_2);
    
    r1 += r2;
    EXPECT_FLOAT_EQ(r1.value(), test_value_1 + test_value_2);
    
    // 测试类型提升（将 double 加到 float）
    Radiance<float> rf(10.0f);
    Radiance<double> rd(5.5);
    rf += rd;
    EXPECT_FLOAT_EQ(rf.value(), 15.5f);
}

TEST_F(RadiometryTest, SubtractionAssignment) {
    Radiance<float> r1(test_value_1);
    Radiance<float> r2(test_value_2);
    
    r1 -= r2;
    EXPECT_FLOAT_EQ(r1.value(), test_value_1 - test_value_2);
}

TEST_F(RadiometryTest, MultiplicationAssignment) {
    Radiance<float> radiance(test_value_1);
    
    radiance *= test_scalar;
    EXPECT_FLOAT_EQ(radiance.value(), test_value_1 * test_scalar);
}

TEST_F(RadiometryTest, DivisionAssignment) {
    Radiance<float> radiance(test_value_1);
    
    radiance /= test_scalar;
    EXPECT_FLOAT_EQ(radiance.value(), test_value_1 / test_scalar);
    
    // 测试除以零
    Radiance<float> r2(test_value_2);
    EXPECT_THROW(r2 /= zero_value, std::exception);
}

// =============================================================================
// 比较运算符测试
// =============================================================================

TEST_F(RadiometryTest, EqualityComparison) {
    Radiance<float> r1(test_value_1);
    Radiance<float> r2(test_value_1);
    Radiance<float> r3(test_value_2);
    
    EXPECT_TRUE(r1 == r2);
    EXPECT_FALSE(r1 == r3);
    EXPECT_TRUE(r1 != r3);
    EXPECT_FALSE(r1 != r2);
    
    // 测试类型提升
    Radiance<float> rf(10.0f);
    Radiance<double> rd(10.0);
    EXPECT_TRUE(rf == rd);
}

TEST_F(RadiometryTest, OrderingComparison) {
    Radiance<float> r1(test_value_1);  // 42.5
    Radiance<float> r2(test_value_2);  // 17.3
    
    EXPECT_TRUE(r1 > r2);
    EXPECT_TRUE(r1 >= r2);
    EXPECT_TRUE(r2 < r1);
    EXPECT_TRUE(r2 <= r1);
    
    Radiance<float> r3(test_value_1);
    EXPECT_TRUE(r1 >= r3);
    EXPECT_TRUE(r1 <= r3);
    
    // 测试类型提升
    Radiance<float> rf(20.0f);
    Radiance<double> rd(15.0);
    EXPECT_TRUE(rf > rd);
}

// =============================================================================
// 物理单位运算测试
// =============================================================================

TEST_F(RadiometryTest, IntensityTimesSolidAngleEqualsFlux) {
    Intensity<float> intensity(100.0f);
    SolidAngle<float> solid_angle(0.5f);
    
    auto flux = intensity * solid_angle;
    EXPECT_TRUE((std::is_same_v<decltype(flux), Flux<float>>));
    EXPECT_FLOAT_EQ(flux.value(), 50.0f);
    
    // 测试交换律
    auto flux2 = solid_angle * intensity;
    EXPECT_TRUE((std::is_same_v<decltype(flux2), Flux<float>>));
    EXPECT_FLOAT_EQ(flux2.value(), 50.0f);
    
    // 测试类型提升
    Intensity<float> if_val(20.0f);
    SolidAngle<double> wd_val(1.5);
    auto promoted_flux = if_val * wd_val;
    EXPECT_TRUE((std::is_same_v<decltype(promoted_flux), Flux<double>>));
    EXPECT_DOUBLE_EQ(promoted_flux.value(), 30.0);
}

TEST_F(RadiometryTest, IrradianceTimesAreaEqualsFlux) {
    Irradiance<float> irradiance(200.0f);
    Area<float> area(0.25f);
    
    auto flux = irradiance * area;
    static_assert(std::is_same_v<decltype(flux), Flux<float>>);
    EXPECT_FLOAT_EQ(flux.value(), 50.0f);
    
    // 测试交换律
    auto flux2 = area * irradiance;
    static_assert(std::is_same_v<decltype(flux2), Flux<float>>);
    EXPECT_FLOAT_EQ(flux2.value(), 50.0f);
}

TEST_F(RadiometryTest, FluxDividedByAreaEqualsIrradiance) {
    Flux<float> flux(100.0f);
    Area<float> area(2.0f);
    
    auto irradiance = flux / area;
    static_assert(std::is_same_v<decltype(irradiance), Irradiance<float>>);
    EXPECT_FLOAT_EQ(irradiance.value(), 50.0f);
    
    // 测试除以零面积
    Area<float> zero_area(0.0f);
    EXPECT_THROW(flux / zero_area, std::exception);
}

TEST_F(RadiometryTest, RadianceTimesSolidAngleEqualsIrradiance) {
    Radiance<float> radiance(150.0f);
    SolidAngle<float> solid_angle(0.4f);
    
    auto irradiance = radiance * solid_angle;
    static_assert(std::is_same_v<decltype(irradiance), Irradiance<float>>);
    EXPECT_FLOAT_EQ(irradiance.value(), 60.0f);
    
    // 测试交换律
    auto irradiance2 = solid_angle * radiance;
    static_assert(std::is_same_v<decltype(irradiance2), Irradiance<float>>);
    EXPECT_FLOAT_EQ(irradiance2.value(), 60.0f);
}

TEST_F(RadiometryTest, FluxDividedBySolidAngleEqualsIntensity) {
    Flux<float> flux(120.0f);
    SolidAngle<float> solid_angle(3.0f);
    
    auto intensity = flux / solid_angle;
    static_assert(std::is_same_v<decltype(intensity), Intensity<float>>);
    EXPECT_FLOAT_EQ(intensity.value(), 40.0f);
    
    // 测试除以零立体角
    SolidAngle<float> zero_angle(0.0f);
    EXPECT_THROW(flux / zero_angle, std::exception);
}

TEST_F(RadiometryTest, RadianceTimesAreaEqualsIntensity) {
    Radiance<float> radiance(80.0f);
    Area<float> area(1.25f);
    
    auto intensity = radiance * area;
    static_assert(std::is_same_v<decltype(intensity), Intensity<float>>);
    EXPECT_FLOAT_EQ(intensity.value(), 100.0f);
    
    // 测试交换律
    auto intensity2 = area * radiance;
    static_assert(std::is_same_v<decltype(intensity2), Intensity<float>>);
    EXPECT_FLOAT_EQ(intensity2.value(), 100.0f);
}

TEST_F(RadiometryTest, IrradianceDividedBySolidAngleEqualsRadiance) {
    Irradiance<float> irradiance(90.0f);
    SolidAngle<float> solid_angle(1.8f);
    
    auto radiance = irradiance / solid_angle;
    static_assert(std::is_same_v<decltype(radiance), Radiance<float>>);
    EXPECT_FLOAT_EQ(radiance.value(), 50.0f);
    
    // 测试除以零立体角
    SolidAngle<float> zero_angle(0.0f);
    EXPECT_THROW(irradiance / zero_angle, std::exception);
}

TEST_F(RadiometryTest, IntensityDividedByAreaEqualsRadiance) {
    Intensity<float> intensity(75.0f);
    Area<float> area(1.5f);
    
    auto radiance = intensity / area;
    static_assert(std::is_same_v<decltype(radiance), Radiance<float>>);
    EXPECT_FLOAT_EQ(radiance.value(), 50.0f);
    
    // 测试除以零面积
    Area<float> zero_area(0.0f);
    EXPECT_THROW(intensity / zero_area, std::exception);
}

// =============================================================================
// 便利函数测试
// =============================================================================

TEST_F(RadiometryTest, ProjectAreaCos) {
    Area<float> area(10.0f);
    float cos_theta = 0.5f;
    
    auto projected_area = project_area_cos(area, cos_theta);
    static_assert(std::is_same_v<decltype(projected_area), Area<float>>);
    EXPECT_FLOAT_EQ(projected_area.value(), 5.0f);
    
    // 测试 cos_theta = 1 的情况（正对）
    auto full_area = project_area_cos(area, 1.0f);
    EXPECT_FLOAT_EQ(full_area.value(), area.value());
    
    // 测试 cos_theta = 0 的情况（垂直）
    auto zero_area = project_area_cos(area, 0.0f);
    EXPECT_FLOAT_EQ(zero_area.value(), 0.0f);
}

TEST_F(RadiometryTest, HemisphereSolidAngle) {
    auto hemisphere_sr_f = hemisphere_sr<float>();
    static_assert(std::is_same_v<decltype(hemisphere_sr_f), SolidAngle<float>>);
    EXPECT_NEAR(hemisphere_sr_f.value(), 2.0f * 3.14159265f, epsilon);
    
    auto hemisphere_sr_d = hemisphere_sr<double>();
    static_assert(std::is_same_v<decltype(hemisphere_sr_d), SolidAngle<double>>);
    EXPECT_NEAR(hemisphere_sr_d.value(), 2.0 * 3.141592653589793, 1e-10);
}

TEST_F(RadiometryTest, SphereSolidAngle) {
    auto sphere_sr_f = sphere_sr<float>();
    static_assert(std::is_same_v<decltype(sphere_sr_f), SolidAngle<float>>);
    EXPECT_NEAR(sphere_sr_f.value(), 4.0f * 3.14159265f, epsilon);
    
    auto sphere_sr_d = sphere_sr<double>();
    static_assert(std::is_same_v<decltype(sphere_sr_d), SolidAngle<double>>);
    EXPECT_NEAR(sphere_sr_d.value(), 4.0 * 3.141592653589793, 1e-10);
}

// =============================================================================
// 单位信息和字符串转换测试
// =============================================================================

TEST_F(RadiometryTest, RadiometryUnitInfoSymbols) {
    EXPECT_STREQ(RadiometryUnitInfo<RadianceTag>::symbol, "W/(m²·sr)");
    EXPECT_STREQ(RadiometryUnitInfo<FluxTag>::symbol, "W");
    EXPECT_STREQ(RadiometryUnitInfo<IntensityTag>::symbol, "W/sr");
    EXPECT_STREQ(RadiometryUnitInfo<IrradianceTag>::symbol, "W/m²");
    EXPECT_STREQ(RadiometryUnitInfo<SolidAngleTag>::symbol, "sr");
    EXPECT_STREQ(RadiometryUnitInfo<AreaTag>::symbol, "m²");
}

TEST_F(RadiometryTest, RadiometryUnitInfoDimensions) {
    // 测试辐射度的维度
    EXPECT_EQ(RadiometryUnitInfo<RadianceTag>::dim_work, 1);
    EXPECT_EQ(RadiometryUnitInfo<RadianceTag>::dim_length, -2);
    EXPECT_EQ(RadiometryUnitInfo<RadianceTag>::dim_solid_angle, -1);
    
    // 测试通量的维度
    EXPECT_EQ(RadiometryUnitInfo<FluxTag>::dim_work, 1);
    EXPECT_EQ(RadiometryUnitInfo<FluxTag>::dim_length, 0);
    EXPECT_EQ(RadiometryUnitInfo<FluxTag>::dim_solid_angle, 0);
    
    // 测试强度的维度
    EXPECT_EQ(RadiometryUnitInfo<IntensityTag>::dim_work, 1);
    EXPECT_EQ(RadiometryUnitInfo<IntensityTag>::dim_length, 0);
    EXPECT_EQ(RadiometryUnitInfo<IntensityTag>::dim_solid_angle, -1);
    
    // 测试辐照度的维度
    EXPECT_EQ(RadiometryUnitInfo<IrradianceTag>::dim_work, 1);
    EXPECT_EQ(RadiometryUnitInfo<IrradianceTag>::dim_length, -2);
    EXPECT_EQ(RadiometryUnitInfo<IrradianceTag>::dim_solid_angle, 0);
    
    // 测试面积的维度
    EXPECT_EQ(RadiometryUnitInfo<AreaTag>::dim_work, 0);
    EXPECT_EQ(RadiometryUnitInfo<AreaTag>::dim_length, 2);
    EXPECT_EQ(RadiometryUnitInfo<AreaTag>::dim_solid_angle, 0);
    
    // 测试立体角的维度
    EXPECT_EQ(RadiometryUnitInfo<SolidAngleTag>::dim_work, 0);
    EXPECT_EQ(RadiometryUnitInfo<SolidAngleTag>::dim_length, 0);
    EXPECT_EQ(RadiometryUnitInfo<SolidAngleTag>::dim_solid_angle, -1);
}

TEST_F(RadiometryTest, ToString) {
    Radiance<float> radiance(123.45678f);
    std::string radiance_str = to_string(radiance);
    std::cout << "Radiance: " << radiance_str << std::endl;
    EXPECT_TRUE(radiance_str.find("W/(m²·sr)") != std::string::npos);
    
    Flux<double> flux(987.654321);
    std::string flux_str = to_string(flux);
    EXPECT_TRUE(flux_str.find("W") != std::string::npos);
    
    Area<float> area(42.0f);
    std::string area_str = to_string(area);
    EXPECT_TRUE(area_str.find("m²") != std::string::npos);
}

// =============================================================================
// 边界和错误情况测试
// =============================================================================

TEST_F(RadiometryTest, NegativeValues) {
    // 虽然物理量通常为正，但数学上应该支持负值
    Radiance<float> negative_radiance(-10.0f);
    EXPECT_FLOAT_EQ(negative_radiance.value(), -10.0f);
    
    // 测试负值的运算
    Radiance<float> positive_radiance(5.0f);
    auto result = negative_radiance + positive_radiance;
    EXPECT_FLOAT_EQ(result.value(), -5.0f);
}

TEST_F(RadiometryTest, VerySmallValues) {
    float small_value = std::numeric_limits<float>::min();
    Radiance<float> small_radiance(small_value);
    EXPECT_FLOAT_EQ(small_radiance.value(), small_value);
    
    // 测试小值的运算
    auto doubled = small_radiance * 2.0f;
    EXPECT_FLOAT_EQ(doubled.value(), small_value * 2.0f);
}

TEST_F(RadiometryTest, VeryLargeValues) {
    float large_value = std::numeric_limits<float>::max() / 2.0f;  // 避免溢出
    Radiance<float> large_radiance(large_value);
    EXPECT_FLOAT_EQ(large_radiance.value(), large_value);
    
    // 测试大值的运算
    auto halved = large_radiance / 2.0f;
    EXPECT_FLOAT_EQ(halved.value(), large_value / 2.0f);
}

TEST_F(RadiometryTest, InfinityAndNaN) {
    // 测试无穷大
    Radiance<float> inf_radiance(std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isinf(inf_radiance.value()));
    
    // 测试 NaN
    Radiance<float> nan_radiance(std::numeric_limits<float>::quiet_NaN());
    EXPECT_TRUE(std::isnan(nan_radiance.value()));
}

// =============================================================================
// 类型提升和模板测试
// =============================================================================

TEST_F(RadiometryTest, TypePromotion) {
    // float + double -> double
    Radiance<float> rf(10.0f);
    Radiance<double> rd(20.0);
    auto result_fd = rf + rd;
    static_assert(std::is_same_v<decltype(result_fd), Radiance<double>>);
    
    // int scalar * float -> float
    Radiance<float> rf2(5.0f);
    int scalar = 3;
    auto result_if = rf2 * scalar;
    static_assert(std::is_same_v<decltype(result_if), Radiance<float>>);
    
    // float / double -> double
    Radiance<float> rf3(15.0f);
    double scalar_d = 3.0;
    auto result_fd2 = rf3 / scalar_d;
    static_assert(std::is_same_v<decltype(result_fd2), Radiance<double>>);
}

TEST_F(RadiometryTest, ConstexprOperations) {
    // 测试编译时常量表达式
    constexpr Radiance<float> r1(10.0f);
    constexpr Radiance<float> r2(5.0f);
    constexpr auto sum = r1 + r2;
    static_assert(sum.value() == 15.0f);
    
    constexpr auto product = r1 * 2.0f;
    static_assert(product.value() == 20.0f);
    
    constexpr auto quotient = r1 / 2.0f;
    static_assert(quotient.value() == 5.0f);
}

// =============================================================================
// 物理关系验证测试
// =============================================================================

TEST_F(RadiometryTest, PhysicalRelationshipsConsistency) {
    // 设置物理量
    Radiance<float> radiance(100.0f);  // W/(m²·sr)
    Area<float> area(2.0f);            // m²
    SolidAngle<float> solid_angle(0.5f); // sr
    
    // 验证 Radiance * Area * SolidAngle = Flux
    auto intensity = radiance * area;           // W/sr
    auto flux1 = intensity * solid_angle;      // W
    
    auto irradiance = radiance * solid_angle;  // W/m²
    auto flux2 = irradiance * area;            // W
    
    EXPECT_FLOAT_EQ(flux1.value(), flux2.value());
    EXPECT_FLOAT_EQ(flux1.value(), 100.0f * 2.0f * 0.5f);
}

TEST_F(RadiometryTest, InverseOperations) {
    // 测试运算的可逆性
    Flux<float> original_flux(200.0f);
    Area<float> area(4.0f);
    SolidAngle<float> solid_angle(2.0f);
    
    // Flux -> Irradiance -> Flux
    auto irradiance = original_flux / area;
    auto reconstructed_flux1 = irradiance * area;
    EXPECT_FLOAT_EQ(reconstructed_flux1.value(), original_flux.value());
    
    // Flux -> Intensity -> Flux
    auto intensity = original_flux / solid_angle;
    auto reconstructed_flux2 = intensity * solid_angle;
    EXPECT_FLOAT_EQ(reconstructed_flux2.value(), original_flux.value());
    
    // Irradiance -> Radiance -> Irradiance
    auto radiance = irradiance / solid_angle;
    auto reconstructed_irradiance = radiance * solid_angle;
    EXPECT_FLOAT_EQ(reconstructed_irradiance.value(), irradiance.value());
}

// =============================================================================
// 性能和内存布局测试
// =============================================================================

TEST_F(RadiometryTest, SizeAndAlignment) {
    // 验证类型的大小和对齐
    EXPECT_EQ(sizeof(Radiance<float>), sizeof(float));
    EXPECT_EQ(sizeof(Radiance<double>), sizeof(double));
    EXPECT_EQ(alignof(Radiance<float>), alignof(float));
    EXPECT_EQ(alignof(Radiance<double>), alignof(double));
    
    // 验证是平凡类型（可以进行位复制等优化）
    EXPECT_TRUE(std::is_trivially_copyable_v<Radiance<float>>);
    EXPECT_TRUE(std::is_trivially_destructible_v<Radiance<float>>);
}

TEST_F(RadiometryTest, CopyAndMoveSemantics) {
    Radiance<float> original(42.0f);
    
    // 复制构造
    Radiance<float> copied(original);
    EXPECT_FLOAT_EQ(copied.value(), original.value());
    
    // 移动构造
    Radiance<float> moved(std::move(copied));
    EXPECT_FLOAT_EQ(moved.value(), original.value());
    
    // 复制赋值
    Radiance<float> assigned;
    assigned = original;
    EXPECT_FLOAT_EQ(assigned.value(), original.value());
    
    // 移动赋值
    Radiance<float> move_assigned;
    move_assigned = std::move(assigned);
    EXPECT_FLOAT_EQ(move_assigned.value(), original.value());
}