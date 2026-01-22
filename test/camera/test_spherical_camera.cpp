/**
 * @file test_spherical_camera.cpp
 * @brief Comprehensive unit tests for SphericalCamera class
 * 
 * This file contains thorough tests for the SphericalCamera class template,
 * including tests for both EqualRectangular and EqualArea mapping modes.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "camera/camera.hpp"
#include "geometry/ray.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/format.hpp"

namespace pbpt::camera::testing {

// ============================================================================
// SphericalCamera - EqualRectangular Mapping Tests
// ============================================================================

TEST(SphericalCameraTest, ConstructionEqualRectangular) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    EXPECT_EQ(camera.mapping(), SphericalCameraMapping::EqualRectangular);
    EXPECT_EQ(camera.film_resolution().x(), 1920);
    EXPECT_EQ(camera.film_resolution().y(), 960);
}

TEST(SphericalCameraTest, ConstructionEqualArea) {
    math::Vector<int, 2> resolution(2048, 1024);
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualArea);
    
    EXPECT_EQ(camera.mapping(), SphericalCameraMapping::EqualArea);
    EXPECT_EQ(camera.film_resolution().x(), 2048);
    EXPECT_EQ(camera.film_resolution().y(), 1024);
}

TEST(SphericalCameraTest, DefaultMapping) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution);
    
    // Default should be EqualRectangular
    EXPECT_EQ(camera.mapping(), SphericalCameraMapping::EqualRectangular);
}

TEST(SphericalCameraTest, GenerateRayOrigin) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray = camera.generate_ray(sample);
    
    // All rays should originate from (0, 0, 0)
    EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.origin().z(), 0.0f, 1e-5f);
}

TEST(SphericalCameraTest, GenerateRayDirectionNormalized) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test multiple points
    std::vector<math::Point<float, 2>> test_points;
    test_points.push_back(math::Point<float, 2>(0.0f, 0.0f));
    test_points.push_back(math::Point<float, 2>(960.0f, 480.0f));
    test_points.push_back(math::Point<float, 2>(1920.0f, 960.0f));
    test_points.push_back(math::Point<float, 2>(480.0f, 240.0f));
    test_points.push_back(math::Point<float, 2>(1440.0f, 720.0f));
    
    for (const auto& p : test_points) {
        CameraSample<float> sample;
        sample.p_film = p;
        
        auto ray = camera.generate_ray(sample);
        
        // Direction should be normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f) << "Ray direction not normalized at film point (" 
                                          << p.x() << ", " << p.y() << ")";
    }
}

TEST(SphericalCameraTest, EqualRectangularCenterDirection) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Center of film should map to forward direction (considering y/z swap)
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Center ray direction: " << ray.direction() << std::endl;
    
    // At center (u=0.5, v=0.5): theta=pi/2, phi=pi
    // Spherical: (sin(pi/2)*cos(pi), sin(pi/2)*sin(pi), cos(pi/2)) = (-1, 0, 0)
    // After y/z swap: (-1, 0, 0) -> (-1, 0, 0)
    EXPECT_NEAR(ray.direction().x(), -1.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().y(), 0.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().z(), 0.0f, 1e-4f);
}

TEST(SphericalCameraTest, EqualRectangularLeftEdge) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Left edge (u=0, v=0.5): theta=0, phi=pi
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(0.0f, 480.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Left edge ray direction: " << ray.direction() << std::endl;
    
    // Spherical: (sin(0)*cos(pi), sin(0)*sin(pi), cos(0)) = (0, 0, 1)
    // After y/z swap: (0, 0, 1) -> (0, 1, 0)
    EXPECT_NEAR(ray.direction().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(std::abs(ray.direction().y()), 1.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().z(), 0.0f, 1e-4f);
}

TEST(SphericalCameraTest, EqualRectangularRightEdge) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Right edge (u=1, v=0.5): theta=pi, phi=pi
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(1920.0f, 480.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Right edge ray direction: " << ray.direction() << std::endl;
    
    // Spherical: (sin(pi)*cos(pi), sin(pi)*sin(pi), cos(pi)) = (0, 0, -1)
    // After y/z swap: (0, 0, -1) -> (0, -1, 0)
    EXPECT_NEAR(ray.direction().x(), 0.0f, 1e-4f);
    EXPECT_NEAR(std::abs(ray.direction().y()), 1.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().z(), 0.0f, 1e-4f);
}

TEST(SphericalCameraTest, EqualRectangularTopEdge) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Top edge (u=0.5, v=0): theta=pi/2, phi=0
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 0.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Top edge ray direction: " << ray.direction() << std::endl;
    
    // Spherical: (sin(pi/2)*cos(0), sin(pi/2)*sin(0), cos(pi/2)) = (1, 0, 0)
    EXPECT_NEAR(ray.direction().x(), 1.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().y(), 0.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().z(), 0.0f, 1e-4f);
}

TEST(SphericalCameraTest, EqualRectangularBottomEdge) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Bottom edge (u=0.5, v=1): theta=pi/2, phi=2*pi (=0)
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 960.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Bottom edge ray direction: " << ray.direction() << std::endl;
    
    // Should be same as top edge due to phi wrapping
    EXPECT_NEAR(ray.direction().x(), 1.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().y(), 0.0f, 1e-4f);
    EXPECT_NEAR(ray.direction().z(), 0.0f, 1e-4f);
}

TEST(SphericalCameraTest, EqualRectangularCorners) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test all four corners
    std::vector<math::Point<float, 2>> corners;
    corners.push_back(math::Point<float, 2>(0.0f, 0.0f));       // Top-left
    corners.push_back(math::Point<float, 2>(1920.0f, 0.0f));    // Top-right
    corners.push_back(math::Point<float, 2>(0.0f, 960.0f));     // Bottom-left
    corners.push_back(math::Point<float, 2>(1920.0f, 960.0f));  // Bottom-right
    
    for (const auto& corner : corners) {
        CameraSample<float> sample;
        sample.p_film = corner;
        
        auto ray = camera.generate_ray(sample);
        
        // All rays should be normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f);
        
        // Origin should be (0, 0, 0)
        EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-5f);
        EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-5f);
        EXPECT_NEAR(ray.origin().z(), 0.0f, 1e-5f);
    }
}

TEST(SphericalCameraTest, EqualRectangularFullCoverage) {
    math::Vector<int, 2> resolution(360, 180);  // 1 degree per pixel
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Sample multiple points to ensure full sphere coverage
    int sample_count = 0;
    for (int y = 0; y < 180; y += 30) {
        for (int x = 0; x < 360; x += 60) {
            CameraSample<double> sample;
            sample.p_film = math::Point<double, 2>(static_cast<double>(x), static_cast<double>(y));
            
            auto ray = camera.generate_ray(sample);
            
            // Direction should be normalized
            double length = std::sqrt(
                ray.direction().x() * ray.direction().x() +
                ray.direction().y() * ray.direction().y() +
                ray.direction().z() * ray.direction().z()
            );
            EXPECT_NEAR(length, 1.0, 1e-5);
            sample_count++;
        }
    }
    
    EXPECT_GT(sample_count, 0) << "Should have sampled multiple points";
}

// ============================================================================
// SphericalCamera - EqualArea Mapping Tests
// ============================================================================

TEST(SphericalCameraTest, EqualAreaDirectionNormalized) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualArea);
    
    // Test multiple points
    std::vector<math::Point<float, 2>> test_points;
    test_points.push_back(math::Point<float, 2>(0.0f, 0.0f));
    test_points.push_back(math::Point<float, 2>(960.0f, 480.0f));
    test_points.push_back(math::Point<float, 2>(1920.0f, 960.0f));
    test_points.push_back(math::Point<float, 2>(480.0f, 240.0f));
    test_points.push_back(math::Point<float, 2>(1440.0f, 720.0f));
    
    for (const auto& p : test_points) {
        CameraSample<float> sample;
        sample.p_film = p;
        
        auto ray = camera.generate_ray(sample);
        
        // Direction should be normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f) << "Ray direction not normalized at film point (" 
                                          << p.x() << ", " << p.y() << ")";
        
        // Origin should be (0, 0, 0)
        EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-5f);
        EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-5f);
        EXPECT_NEAR(ray.origin().z(), 0.0f, 1e-5f);
    }
}

TEST(SphericalCameraTest, EqualAreaCenterDirection) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualArea);
    
    // Center of film
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray = camera.generate_ray(sample);
    std::cout << "Equal area center ray direction: " << ray.direction() << std::endl;
    
    // Direction should be normalized
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, EqualAreaCorners) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualArea);
    
    // Test all four corners
    std::vector<math::Point<float, 2>> corners;
    corners.push_back(math::Point<float, 2>(0.0f, 0.0f));
    corners.push_back(math::Point<float, 2>(1920.0f, 0.0f));
    corners.push_back(math::Point<float, 2>(0.0f, 960.0f));
    corners.push_back(math::Point<float, 2>(1920.0f, 960.0f));
    
    for (const auto& corner : corners) {
        CameraSample<float> sample;
        sample.p_film = corner;
        
        auto ray = camera.generate_ray(sample);
        
        // All rays should be normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f);
    }
}

// ============================================================================
// SphericalCamera - Differential Ray Tests
// ============================================================================

TEST(SphericalCameraTest, DifferentialRayEqualRectangular) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray_diff = camera.generate_differential_ray(sample);
    
    // Main ray should be normalized
    float length = std::sqrt(
        ray_diff.main_ray().direction().x() * ray_diff.main_ray().direction().x() +
        ray_diff.main_ray().direction().y() * ray_diff.main_ray().direction().y() +
        ray_diff.main_ray().direction().z() * ray_diff.main_ray().direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
    
    // Differential rays should also be normalized
    float length_x = std::sqrt(
        ray_diff.x().direction().x() * ray_diff.x().direction().x() +
        ray_diff.x().direction().y() * ray_diff.x().direction().y() +
        ray_diff.x().direction().z() * ray_diff.x().direction().z()
    );
    EXPECT_NEAR(length_x, 1.0f, 1e-5f);
    
    float length_y = std::sqrt(
        ray_diff.y().direction().x() * ray_diff.y().direction().x() +
        ray_diff.y().direction().y() * ray_diff.y().direction().y() +
        ray_diff.y().direction().z() * ray_diff.y().direction().z()
    );
    EXPECT_NEAR(length_y, 1.0f, 1e-5f);
    
    // All origins should be (0, 0, 0)
    EXPECT_NEAR(ray_diff.main_ray().origin().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray_diff.x().origin().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray_diff.y().origin().x(), 0.0f, 1e-5f);
}

TEST(SphericalCameraTest, DifferentialRayEqualArea) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualArea);
    
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(960.0, 480.0);
    
    auto ray_diff = camera.generate_differential_ray(sample);
    
    // Main ray should be normalized
    double length = std::sqrt(
        ray_diff.main_ray().direction().x() * ray_diff.main_ray().direction().x() +
        ray_diff.main_ray().direction().y() * ray_diff.main_ray().direction().y() +
        ray_diff.main_ray().direction().z() * ray_diff.main_ray().direction().z()
    );
    EXPECT_NEAR(length, 1.0, 1e-5);
    
    // Differential rays should also be normalized
    double length_x = std::sqrt(
        ray_diff.x().direction().x() * ray_diff.x().direction().x() +
        ray_diff.x().direction().y() * ray_diff.x().direction().y() +
        ray_diff.x().direction().z() * ray_diff.x().direction().z()
    );
    EXPECT_NEAR(length_x, 1.0, 1e-5);
    
    double length_y = std::sqrt(
        ray_diff.y().direction().x() * ray_diff.y().direction().x() +
        ray_diff.y().direction().y() * ray_diff.y().direction().y() +
        ray_diff.y().direction().z() * ray_diff.y().direction().z()
    );
    EXPECT_NEAR(length_y, 1.0, 1e-5);
}

TEST(SphericalCameraTest, DifferentialRayDirectionDifference) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray_diff = camera.generate_differential_ray(sample);
    
    // X-differential ray should have different direction
    bool x_different = 
        std::abs(ray_diff.main_ray().direction().x() - ray_diff.x().direction().x()) > 1e-6f ||
        std::abs(ray_diff.main_ray().direction().y() - ray_diff.x().direction().y()) > 1e-6f ||
        std::abs(ray_diff.main_ray().direction().z() - ray_diff.x().direction().z()) > 1e-6f;
    EXPECT_TRUE(x_different) << "X-differential ray should have different direction";
    
    // Y-differential ray should have different direction
    bool y_different = 
        std::abs(ray_diff.main_ray().direction().x() - ray_diff.y().direction().x()) > 1e-6f ||
        std::abs(ray_diff.main_ray().direction().y() - ray_diff.y().direction().y()) > 1e-6f ||
        std::abs(ray_diff.main_ray().direction().z() - ray_diff.y().direction().z()) > 1e-6f;
    EXPECT_TRUE(y_different) << "Y-differential ray should have different direction";
}

TEST(SphericalCameraTest, DifferentialRayConsistency) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    // Generate differential ray
    auto ray_diff = camera.generate_differential_ray(sample);
    
    // Manually generate the same rays
    auto main_ray = camera.generate_ray(sample);
    
    CameraSample<float> sample_x = sample;
    sample_x.p_film = sample.p_film + math::Vector<float, 2>(1e-3f, 0);
    auto ray_x = camera.generate_ray(sample_x);
    
    CameraSample<float> sample_y = sample;
    sample_y.p_film = sample.p_film + math::Vector<float, 2>(0, 1e-3f);
    auto ray_y = camera.generate_ray(sample_y);
    
    // Differential ray should match manual calculation
    EXPECT_NEAR(ray_diff.main_ray().direction().x(), main_ray.direction().x(), 1e-5f);
    EXPECT_NEAR(ray_diff.main_ray().direction().y(), main_ray.direction().y(), 1e-5f);
    EXPECT_NEAR(ray_diff.main_ray().direction().z(), main_ray.direction().z(), 1e-5f);
}

// ============================================================================
// SphericalCamera - Type Template Tests
// ============================================================================

TEST(SphericalCameraTest, FloatAndDoubleTypes) {
    math::Vector<int, 2> resolution_f(1920, 960);
    math::Vector<int, 2> resolution_d(1920, 960);
    
    // Test float types
    SphericalCamera<float> camera_f(resolution_f, SphericalCameraMapping::EqualRectangular);
    EXPECT_EQ(camera_f.film_resolution().x(), 1920);
    
    // Test double types
    SphericalCamera<double> camera_d(resolution_d, SphericalCameraMapping::EqualArea);
    EXPECT_EQ(camera_d.film_resolution().x(), 1920);
    
    SUCCEED() << "Both float and double spherical camera types compile and construct";
}

TEST(SphericalCameraTest, CRTPBehavior) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> spherical_cam(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test that spherical camera uses CRTP for static polymorphism
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    // Should be able to call generate_ray through the camera
    auto ray = spherical_cam.generate_ray(sample);
    
    // Ray should be normalized
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
    
    // Origin should be at (0, 0, 0)
    EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.origin().z(), 0.0f, 1e-5f);
}

// ============================================================================
// SphericalCamera - Edge Cases and Boundary Tests
// ============================================================================

TEST(SphericalCameraTest, NegativeFilmCoordinates) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test with negative coordinates (edge case)
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(-10.0f, -10.0f);
    
    auto ray = camera.generate_ray(sample);
    
    // Should still produce a valid normalized ray
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, LargeFilmCoordinates) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test with coordinates beyond film resolution
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(3000.0f, 2000.0f);
    
    auto ray = camera.generate_ray(sample);
    
    // Should still produce a valid normalized ray
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, ZeroFilmCoordinates) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualArea);
    
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(0.0, 0.0);
    
    auto ray = camera.generate_ray(sample);
    
    // Should produce a valid ray
    double length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0, 1e-5);
    
    EXPECT_NEAR(ray.origin().x(), 0.0, 1e-10);
    EXPECT_NEAR(ray.origin().y(), 0.0, 1e-10);
    EXPECT_NEAR(ray.origin().z(), 0.0, 1e-10);
}

// ============================================================================
// SphericalCamera - Resolution Tests
// ============================================================================

TEST(SphericalCameraTest, SquareResolution) {
    math::Vector<int, 2> resolution(1024, 1024);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    EXPECT_EQ(camera.film_resolution().x(), 1024);
    EXPECT_EQ(camera.film_resolution().y(), 1024);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(512.0f, 512.0f);
    
    auto ray = camera.generate_ray(sample);
    
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, SmallResolution) {
    math::Vector<int, 2> resolution(32, 16);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualArea);
    
    EXPECT_EQ(camera.film_resolution().x(), 32);
    EXPECT_EQ(camera.film_resolution().y(), 16);
    
    // Test center
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(16.0f, 8.0f);
    
    auto ray = camera.generate_ray(sample);
    
    float length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, LargeResolution) {
    math::Vector<int, 2> resolution(8192, 4096);
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    EXPECT_EQ(camera.film_resolution().x(), 8192);
    EXPECT_EQ(camera.film_resolution().y(), 4096);
    
    // Test center
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(4096.0, 2048.0);
    
    auto ray = camera.generate_ray(sample);
    
    double length = std::sqrt(
        ray.direction().x() * ray.direction().x() +
        ray.direction().y() * ray.direction().y() +
        ray.direction().z() * ray.direction().z()
    );
    EXPECT_NEAR(length, 1.0, 1e-5);
}

// ============================================================================
// SphericalCamera - Mapping Comparison Tests
// ============================================================================

TEST(SphericalCameraTest, MappingComparisonCenter) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera_rect(resolution, SphericalCameraMapping::EqualRectangular);
    SphericalCamera<float> camera_area(resolution, SphericalCameraMapping::EqualArea);
    
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(960.0f, 480.0f);
    
    auto ray_rect = camera_rect.generate_ray(sample);
    auto ray_area = camera_area.generate_ray(sample);
    
    // Both should produce normalized rays
    float length_rect = std::sqrt(
        ray_rect.direction().x() * ray_rect.direction().x() +
        ray_rect.direction().y() * ray_rect.direction().y() +
        ray_rect.direction().z() * ray_rect.direction().z()
    );
    float length_area = std::sqrt(
        ray_area.direction().x() * ray_area.direction().x() +
        ray_area.direction().y() * ray_area.direction().y() +
        ray_area.direction().z() * ray_area.direction().z()
    );
    
    EXPECT_NEAR(length_rect, 1.0f, 1e-5f);
    EXPECT_NEAR(length_area, 1.0f, 1e-5f);
}

TEST(SphericalCameraTest, MappingComparisonCorners) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera_rect(resolution, SphericalCameraMapping::EqualRectangular);
    SphericalCamera<float> camera_area(resolution, SphericalCameraMapping::EqualArea);
    
    std::vector<math::Point<float, 2>> corners;
    corners.push_back(math::Point<float, 2>(0.0f, 0.0f));
    corners.push_back(math::Point<float, 2>(1920.0f, 0.0f));
    corners.push_back(math::Point<float, 2>(0.0f, 960.0f));
    corners.push_back(math::Point<float, 2>(1920.0f, 960.0f));
    
    for (const auto& corner : corners) {
        CameraSample<float> sample;
        sample.p_film = corner;
        
        auto ray_rect = camera_rect.generate_ray(sample);
        auto ray_area = camera_area.generate_ray(sample);
        
        // Both mappings should produce normalized rays
        float length_rect = std::sqrt(
            ray_rect.direction().x() * ray_rect.direction().x() +
            ray_rect.direction().y() * ray_rect.direction().y() +
            ray_rect.direction().z() * ray_rect.direction().z()
        );
        float length_area = std::sqrt(
            ray_area.direction().x() * ray_area.direction().x() +
            ray_area.direction().y() * ray_area.direction().y() +
            ray_area.direction().z() * ray_area.direction().z()
        );
        
        EXPECT_NEAR(length_rect, 1.0f, 1e-5f);
        EXPECT_NEAR(length_area, 1.0f, 1e-5f);
    }
}

// ============================================================================
// SphericalCamera - Ray Direction Distribution Tests
// ============================================================================

TEST(SphericalCameraTest, RayDirectionDistributionUniformity) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    const int num_samples = 10000;
    const int num_phi_bins = 8;    // 8 bins for azimuthal angle (0 to 2π)
    const int num_theta_bins = 4;  // 4 bins for polar angle (0 to π)
    
    std::vector<std::vector<int>> bins(num_phi_bins, std::vector<int>(num_theta_bins, 0));
    
    const float pi = 3.14159265358979323846f;
    
    // Generate samples across the film uniformly
    for (int i = 0; i < num_samples; ++i) {
        // Uniform sampling across film
        float u = (static_cast<float>(i % 100) + 0.5f) / 100.0f * resolution.x();
        float v = (static_cast<float>((i / 100) % 100) + 0.5f) / 100.0f * resolution.y();
        
        CameraSample<float> sample;
        sample.p_film = math::Point<float, 2>(u, v);
        
        auto ray = camera.generate_ray(sample);
        
        // Verify ray direction is on unit sphere
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f);
        
        // Calculate spherical coordinates from Cartesian
        // Note: The camera uses (x, y, z) where y is up
        // theta: polar angle from +y axis [0, π]
        // phi: azimuthal angle around y axis [-π, π]
        float theta = std::acos(std::clamp(ray.direction().y(), -1.0f, 1.0f));
        float phi = std::atan2(ray.direction().z(), ray.direction().x());
        if (phi < 0) phi += 2.0f * pi;  // Convert to [0, 2π]
        
        // Map to bins
        int phi_bin = std::min(static_cast<int>(phi / (2.0f * pi) * num_phi_bins), num_phi_bins - 1);
        int theta_bin = std::min(static_cast<int>(theta / pi * num_theta_bins), num_theta_bins - 1);
        
        bins[phi_bin][theta_bin]++;
    }
    
    // Verify distribution uniformity
    // For EqualRectangular mapping, film coordinates map uniformly to (theta, phi)
    // However, solid angle ∝ sin(theta), so bins near equator should have more samples
    float expected_per_bin = static_cast<float>(num_samples) / (num_phi_bins * num_theta_bins);
    
    int non_zero_bins = 0;
    for (int phi_bin = 0; phi_bin < num_phi_bins; ++phi_bin) {
        for (int theta_bin = 0; theta_bin < num_theta_bins; ++theta_bin) {
            if (bins[phi_bin][theta_bin] > 0) {
                non_zero_bins++;
            }
        }
    }
    
    // At least 80% of bins should have samples (accounting for our sampling pattern)
    EXPECT_GT(non_zero_bins, static_cast<int>(num_phi_bins * num_theta_bins * 0.8))
        << "Distribution should cover most bins on the sphere";
}

TEST(SphericalCameraTest, RayDirectionOnUnitSphere) {
    math::Vector<int, 2> resolution(360, 180);
    SphericalCamera<double> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test a grid of points to verify all rays are on unit sphere
    const int num_test_points = 1000;
    double max_deviation = 0.0;
    double sum_deviation = 0.0;
    
    for (int i = 0; i < num_test_points; ++i) {
        double u = (i % 100) * 3.6;  // 0 to 360
        double v = (i / 100.0) * 180.0; // 0 to 180
        
        CameraSample<double> sample;
        sample.p_film = math::Point<double, 2>(u, v);
        
        auto ray = camera.generate_ray(sample);
        
        // Calculate length
        double length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        
        double deviation = std::abs(length - 1.0);
        max_deviation = std::max(max_deviation, deviation);
        sum_deviation += deviation;
        
        EXPECT_NEAR(length, 1.0, 1e-10) << "Ray at (" << u << ", " << v << ") not on unit sphere";
    }
    
    // Check statistical properties
    double avg_deviation = sum_deviation / num_test_points;
    EXPECT_LT(avg_deviation, 1e-12) << "Average deviation from unit sphere too large";
    EXPECT_LT(max_deviation, 1e-10) << "Maximum deviation from unit sphere too large";
}

TEST(SphericalCameraTest, FullSphereCoverage) {
    math::Vector<int, 2> resolution(360, 180);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // SphericalCamera mapping explanation:
    // 1. film.x / resolution.x -> u in [0,1] -> theta = pi * u (polar angle)
    // 2. film.y / resolution.y -> v in [0,1] -> phi = 2*pi * v (azimuthal angle)
    // 3. Spherical to Cartesian: x = sin(theta)*cos(phi), y = sin(theta)*sin(phi), z = cos(theta)
    // 4. Swap Y and Z: final = (x, z, y) -> (sin(theta)*cos(phi), cos(theta), sin(theta)*sin(phi))
    
    struct TestDirection {
        float x, y, z;           // Expected direction
        const char* name;
        float film_u, film_v;    // Film coordinates (pixels)
    };
    
    const float pi = 3.14159265358979323846f;
    
    std::vector<TestDirection> test_directions = {
        // Poles (theta = 0 and theta = pi)
        {0.0f, 1.0f, 0.0f, "+Y (North Pole)", 0.0f, 90.0f},      // theta=0: dir=(0,1,0)
        {0.0f, -1.0f, 0.0f, "-Y (South Pole)", 360.0f, 90.0f},   // theta=pi: dir=(0,-1,0)
        
        // Equator cardinal directions (theta = pi/2, so cos(theta)=0)
        {1.0f, 0.0f, 0.0f, "+X", 180.0f, 0.0f},                  // phi=0: cos(0)=1, sin(0)=0
        {-1.0f, 0.0f, 0.0f, "-X", 180.0f, 180.0f},               // phi=pi: cos(pi)=-1, sin(pi)=0
        {0.0f, 0.0f, 1.0f, "+Z", 180.0f, 45.0f},                 // phi=pi/2: cos(pi/2)=0, sin(pi/2)=1
        {0.0f, 0.0f, -1.0f, "-Z", 180.0f, 315.0f},               // phi=3pi/2: cos(3pi/2)=0, sin(3pi/2)=-1
    };
    
    for (const auto& expected : test_directions) {
        CameraSample<float> sample;
        sample.p_film = math::Point<float, 2>(expected.film_u, expected.film_v);
        
        auto ray = camera.generate_ray(sample);
        
        // Verify the ray is normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f) << "Direction: " << expected.name;
        
        // Verify the direction (with some tolerance due to coordinate system)
        float dot_product = 
            ray.direction().x() * expected.x +
            ray.direction().y() * expected.y +
            ray.direction().z() * expected.z;
        
        EXPECT_GT(std::abs(dot_product), 0.99f) 
            << "Direction mismatch for: " << expected.name
            << "\nExpected direction: (" << expected.x << ", " << expected.y << ", " << expected.z << ")"
            << "\nActual direction: (" << ray.direction().x() << ", " 
            << ray.direction().y() << ", " << ray.direction().z() << ")"
            << "\nDot product: " << dot_product;
    }
}

TEST(SphericalCameraTest, SolidAngleDistributionEqualArea) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualArea);
    
    // For EqualArea mapping, equal areas on the film should correspond to equal solid angles
    // We test this by sampling uniformly on the film and checking the distribution
    
    const int num_samples = 5000;
    const int num_bins = 10;  // Divide sphere into regions
    
    std::vector<int> hemisphere_bins(2, 0);  // Upper and lower hemisphere
    
    for (int i = 0; i < num_samples; ++i) {
        float u = (static_cast<float>(i % 100) + 0.5f) / 100.0f * resolution.x();
        float v = (static_cast<float>((i / 100) % 50) + 0.5f) / 50.0f * resolution.y();
        
        CameraSample<float> sample;
        sample.p_film = math::Point<float, 2>(u, v);
        
        auto ray = camera.generate_ray(sample);
        
        // Verify normalized
        float length = std::sqrt(
            ray.direction().x() * ray.direction().x() +
            ray.direction().y() * ray.direction().y() +
            ray.direction().z() * ray.direction().z()
        );
        EXPECT_NEAR(length, 1.0f, 1e-5f);
        
        // Check which hemisphere
        if (ray.direction().y() >= 0) {
            hemisphere_bins[0]++;  // Upper hemisphere (+Y)
        } else {
            hemisphere_bins[1]++;  // Lower hemisphere (-Y)
        }
    }
    
    // For EqualArea mapping with uniform film sampling, we should get roughly equal counts
    // in both hemispheres (within statistical variation)
    float ratio = static_cast<float>(hemisphere_bins[0]) / static_cast<float>(hemisphere_bins[1]);
    EXPECT_GT(ratio, 0.8f) << "Upper hemisphere: " << hemisphere_bins[0] 
                            << ", Lower hemisphere: " << hemisphere_bins[1];
    EXPECT_LT(ratio, 1.25f) << "Upper hemisphere: " << hemisphere_bins[0] 
                             << ", Lower hemisphere: " << hemisphere_bins[1];
}

TEST(SphericalCameraTest, RayDirectionContinuity) {
    math::Vector<int, 2> resolution(1920, 960);
    SphericalCamera<float> camera(resolution, SphericalCameraMapping::EqualRectangular);
    
    // Test that small changes in film coordinates lead to small changes in ray direction
    const float epsilon = 0.1f;  // Small step in film coordinates
    const int num_tests = 100;
    
    for (int i = 0; i < num_tests; ++i) {
        float u = (i % 10) * 192.0f + 96.0f;
        float v = (i / 10.0f) * 96.0f + 48.0f;
        
        CameraSample<float> sample1;
        sample1.p_film = math::Point<float, 2>(u, v);
        auto ray1 = camera.generate_ray(sample1);
        
        CameraSample<float> sample2;
        sample2.p_film = math::Point<float, 2>(u + epsilon, v);
        auto ray2 = camera.generate_ray(sample2);
        
        // Calculate angle between directions
        float dot = ray1.direction().x() * ray2.direction().x() +
                    ray1.direction().y() * ray2.direction().y() +
                    ray1.direction().z() * ray2.direction().z();
        dot = std::clamp(dot, -1.0f, 1.0f);
        float angle = std::acos(dot);
        
        // Angle should be small (less than 1 degree for 0.1 pixel movement)
        EXPECT_LT(angle, 0.02f) << "Discontinuity detected at film point (" << u << ", " << v << ")";
    }
}

}  // namespace pbpt::camera::testing
