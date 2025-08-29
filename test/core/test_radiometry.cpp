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

// =============================================================================
// 新功能测试 - TagTrait 系统
// =============================================================================

TEST_F(RadiometryTest, TagTraitBasic) {
    // 测试普通标签
    static_assert(!TagTrait<RadianceTag>::is_spectral());
    static_assert(!TagTrait<FluxTag>::is_spectral());
    static_assert(!TagTrait<IntensityTag>::is_spectral());
    static_assert(!TagTrait<IrradianceTag>::is_spectral());
    
    using radiance_tag = TagTrait<RadianceTag>::tag_type;
    static_assert(std::is_same_v<radiance_tag, RadianceTag>);
}

TEST_F(RadiometryTest, TagTraitSpectral) {
    // 测试光谱标签
    static_assert(TagTrait<SpectralTag<RadianceTag>>::is_spectral());
    static_assert(TagTrait<SpectralTag<FluxTag>>::is_spectral());
    static_assert(TagTrait<SpectralTag<IntensityTag>>::is_spectral());
    static_assert(TagTrait<SpectralTag<IrradianceTag>>::is_spectral());
    
    using spectral_radiance_tag = TagTrait<SpectralTag<RadianceTag>>::tag_type;
    static_assert(std::is_same_v<spectral_radiance_tag, SpectralTag<RadianceTag>>);
    
    using original_radiance_tag = TagTrait<SpectralTag<RadianceTag>>::original_tag_type;
    static_assert(std::is_same_v<original_radiance_tag, RadianceTag>);
}

// =============================================================================
// 新功能测试 - 光谱辐射量类型
// =============================================================================

TEST_F(RadiometryTest, SpectralRadianceConstruction) {
    SpectralRadiance<float> spectral_radiance;
    EXPECT_FLOAT_EQ(spectral_radiance.value(), 0.0f);
    
    SpectralRadiance<float> spectral_radiance_val(100.5f);
    EXPECT_FLOAT_EQ(spectral_radiance_val.value(), 100.5f);
    
    // 测试类型检查
    static_assert(std::is_same_v<SpectralRadiance<float>::unit_tag, SpectralTag<RadianceTag>>);
}

TEST_F(RadiometryTest, SpectralIntensityConstruction) {
    SpectralIntensity<double> spectral_intensity(250.75);
    EXPECT_DOUBLE_EQ(spectral_intensity.value(), 250.75);
    
    static_assert(std::is_same_v<SpectralIntensity<double>::unit_tag, SpectralTag<IntensityTag>>);
}

TEST_F(RadiometryTest, SpectralIrradianceConstruction) {
    SpectralIrradiance<float> spectral_irradiance(75.25f);
    EXPECT_FLOAT_EQ(spectral_irradiance.value(), 75.25f);
    
    static_assert(std::is_same_v<SpectralIrradiance<float>::unit_tag, SpectralTag<IrradianceTag>>);
}

TEST_F(RadiometryTest, SpectralFluxConstruction) {
    SpectralFlux<double> spectral_flux(500.125);
    EXPECT_DOUBLE_EQ(spectral_flux.value(), 500.125);
    
    static_assert(std::is_same_v<SpectralFlux<double>::unit_tag, SpectralTag<FluxTag>>);
}

TEST_F(RadiometryTest, SpectralQuantityArithmetic) {
    SpectralRadiance<float> sr1(100.0f);
    SpectralRadiance<float> sr2(50.0f);
    
    // 测试加法
    auto sum = sr1 + sr2;
    EXPECT_FLOAT_EQ(sum.value(), 150.0f);
    static_assert(std::is_same_v<decltype(sum), SpectralRadiance<float>>);
    
    // 测试减法
    auto diff = sr1 - sr2;
    EXPECT_FLOAT_EQ(diff.value(), 50.0f);
    
    // 测试标量乘法
    auto scaled = sr1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled.value(), 200.0f);
    
    // 测试标量除法
    auto divided = sr1 / 4.0f;
    EXPECT_FLOAT_EQ(divided.value(), 25.0f);
}

// =============================================================================
// 新功能测试 - 波长类型
// =============================================================================

TEST_F(RadiometryTest, WaveLengthConstruction) {
    WaveLength<float> wavelength;
    EXPECT_FLOAT_EQ(wavelength.value(), 0.0f);
    
    WaveLength<float> wavelength_val(550.0f);  // 典型的可见光波长（纳米）
    EXPECT_FLOAT_EQ(wavelength_val.value(), 550.0f);
    
    WaveLength<double> wavelength_double(700.5);
    EXPECT_DOUBLE_EQ(wavelength_double.value(), 700.5);
    
    static_assert(std::is_same_v<WaveLength<float>::unit_tag, WaveLengthTag>);
}

TEST_F(RadiometryTest, WaveLengthArithmetic) {
    WaveLength<float> w1(600.0f);
    WaveLength<float> w2(100.0f);
    
    auto sum = w1 + w2;
    EXPECT_FLOAT_EQ(sum.value(), 700.0f);
    
    auto diff = w1 - w2;
    EXPECT_FLOAT_EQ(diff.value(), 500.0f);
    
    auto scaled = w1 * 2.0f;
    EXPECT_FLOAT_EQ(scaled.value(), 1200.0f);
    
    auto divided = w1 / 3.0f;
    EXPECT_FLOAT_EQ(divided.value(), 200.0f);
}

// =============================================================================
// 新功能测试 - 光谱量与波长的乘法
// =============================================================================

TEST_F(RadiometryTest, SpectralQuantityTimesWaveLength) {
    SpectralRadiance<float> spectral_radiance(10.0f);
    WaveLength<float> wavelength(550.0f);
    
    auto radiance = spectral_radiance * wavelength;
    static_assert(std::is_same_v<decltype(radiance), Radiance<float>>);
    EXPECT_FLOAT_EQ(radiance.value(), 10.0f * 550.0f);
    
    // 测试类型提升
    SpectralIntensity<float> spectral_intensity(5.0f);
    WaveLength<double> wavelength_double(600.0);
    auto intensity = spectral_intensity * wavelength_double;
    static_assert(std::is_same_v<decltype(intensity), Intensity<double>>);
    EXPECT_DOUBLE_EQ(intensity.value(), 5.0 * 600.0);
}

TEST_F(RadiometryTest, SpectralFluxTimesWaveLength) {
    SpectralFlux<double> spectral_flux(25.5);
    WaveLength<float> wavelength(450.0f);
    
    auto flux = spectral_flux * wavelength;
    static_assert(std::is_same_v<decltype(flux), Flux<double>>);
    EXPECT_DOUBLE_EQ(flux.value(), 25.5 * 450.0);
}

TEST_F(RadiometryTest, SpectralIrradianceTimesWaveLength) {
    SpectralIrradiance<float> spectral_irradiance(15.5f);
    WaveLength<double> wavelength(700.0);
    
    auto irradiance = spectral_irradiance * wavelength;
    static_assert(std::is_same_v<decltype(irradiance), Irradiance<double>>);
    EXPECT_DOUBLE_EQ(irradiance.value(), 15.5 * 700.0);
}

// =============================================================================
// 新功能测试 - DirectionalSolidAngle
// =============================================================================

TEST_F(RadiometryTest, DirectionalSolidAngleConstruction) {
    SolidAngle<float> angle(1.5f);
    pbpt::math::Vector<float, 3> direction(1.0f, 0.0f, 0.0f);
    
    DirectionalSolidAngle<float> dir_angle{angle, direction};
    EXPECT_FLOAT_EQ(dir_angle.solid_angle.value(), 1.5f);
    EXPECT_FLOAT_EQ(dir_angle.direction[0], 1.0f);
    EXPECT_FLOAT_EQ(dir_angle.direction[1], 0.0f);
    EXPECT_FLOAT_EQ(dir_angle.direction[2], 0.0f);
}

TEST_F(RadiometryTest, DirectionalSolidAngleWithNormalizedDirection) {
    SolidAngle<double> angle(2.0);
    pbpt::math::Vector<double, 3> direction(0.0, 1.0, 0.0);  // Y轴方向
    
    DirectionalSolidAngle<double> dir_angle{angle, direction};
    
    // 验证方向向量的长度
    double length_squared = direction[0] * direction[0] + 
                           direction[1] * direction[1] + 
                           direction[2] * direction[2];
    EXPECT_NEAR(length_squared, 1.0, 1e-10);
    
    EXPECT_DOUBLE_EQ(dir_angle.solid_angle.value(), 2.0);
    EXPECT_DOUBLE_EQ(dir_angle.direction[1], 1.0);
}


TEST_F(RadiometryTest, SpectralQuantityComparison) {
    SpectralRadiance<float> sr1(100.0f);
    SpectralRadiance<float> sr2(100.0f);
    SpectralRadiance<float> sr3(150.0f);
    
    EXPECT_TRUE(sr1 == sr2);
    EXPECT_FALSE(sr1 == sr3);
    EXPECT_TRUE(sr1 != sr3);
    EXPECT_TRUE(sr3 > sr1);
    EXPECT_TRUE(sr1 < sr3);
    EXPECT_TRUE(sr1 <= sr2);
    EXPECT_TRUE(sr2 >= sr1);
}

TEST_F(RadiometryTest, SpectralQuantityTypePromotion) {
    SpectralRadiance<float> srf(10.0f);
    SpectralRadiance<double> srd(20.0);
    
    auto result = srf + srd;
    static_assert(std::is_same_v<decltype(result), SpectralRadiance<double>>);
    EXPECT_DOUBLE_EQ(result.value(), 30.0);
    
    // 光谱量与标量的类型提升
    auto scaled = srf * 2.5;
    static_assert(std::is_same_v<decltype(scaled), SpectralRadiance<double>>);
    EXPECT_DOUBLE_EQ(scaled.value(), 25.0);
}

TEST_F(RadiometryTest, WaveLengthTypicalValues) {
    // 测试典型的可见光波长（纳米）
    WaveLength<float> red(700.0f);
    WaveLength<float> green(550.0f);
    WaveLength<float> blue(450.0f);
    
    EXPECT_TRUE(red > green);
    EXPECT_TRUE(green > blue);
    EXPECT_FLOAT_EQ((red - blue).value(), 250.0f);
    
    // 测试红外和紫外
    WaveLength<double> infrared(1000.0);
    WaveLength<double> ultraviolet(300.0);
    
    EXPECT_TRUE(infrared.value() > red.value());
    EXPECT_TRUE(ultraviolet.value() < blue.value());
}

TEST_F(RadiometryTest, SpectralQuantityPhysicalRealism) {
    // 测试物理现实的光谱量计算
    SpectralRadiance<float> solar_spectral_radiance(2000.0f);  // W/(m²·sr·nm)
    WaveLength<float> wavelength_interval(1.0f);  // 1 nm 间隔
    
    auto radiance = solar_spectral_radiance * wavelength_interval;
    static_assert(std::is_same_v<decltype(radiance), Radiance<float>>);
    EXPECT_FLOAT_EQ(radiance.value(), 2000.0f);  // W/(m²·sr)
    
    // 模拟在多个波长上的积分
    float total_radiance = 0.0f;
    for (int lambda = 400; lambda <= 700; lambda += 10) {  // 可见光范围
        WaveLength<float> wl_interval(10.0f);  // 10 nm 间隔
        auto contrib = solar_spectral_radiance * wl_interval;
        total_radiance += contrib.value();
    }
    
    // 验证总辐射度在合理范围内
    EXPECT_GT(total_radiance, 0.0f);
    EXPECT_LT(total_radiance, 1e6f);  // 合理的上限
}

TEST_F(RadiometryTest, ErrorHandlingInNewFeatures) {
    // 测试光谱量除以零的错误处理
    SpectralFlux<float> spectral_flux(100.0f);
    EXPECT_THROW(spectral_flux / 0.0f, std::exception);
    
    // 测试波长除以零的错误处理
    WaveLength<double> wavelength(550.0);
    EXPECT_THROW(wavelength / 0.0, std::exception);
}

TEST_F(RadiometryTest, ConstexprSpectralOperations) {
    // 测试光谱量的编译时计算
    constexpr SpectralRadiance<float> sr1(10.0f);
    constexpr SpectralRadiance<float> sr2(5.0f);
    constexpr auto spectral_sum = sr1 + sr2;
    static_assert(spectral_sum.value() == 15.0f);
    
    constexpr WaveLength<float> wl(100.0f);
    constexpr auto spectral_product = sr1 * 2.0f;
    static_assert(spectral_product.value() == 20.0f);
}