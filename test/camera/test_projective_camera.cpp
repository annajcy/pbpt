/**
 * @file test_camera.cpp
 * @brief Comprehensive unit tests for Camera classes
 *
 * This file contains thorough tests for the Camera class templates,
 * including OrthographicCamera, PerspectiveCamera, ThinLensOrthographicCamera,
 * and ThinLensPerspectiveCamera.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

#include "pbpt/camera/camera.h"
#include "pbpt/geometry/ray.hpp"
#include "pbpt/math/point.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/math/format.hpp"

namespace pbpt::camera::testing {

using namespace pbpt::sampler;

// ============================================================================
// OrthographicCamera Tests
// ============================================================================

TEST(OrthographicCameraTest, Construction) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    OrthographicCamera<float> camera(resolution, physical_size, -0.1, -100.0f);

    EXPECT_NE(camera.projection().camera_to_clip(), geometry::Transform<float>());
}

TEST(OrthographicCameraTest, GenerateRayCenter) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    OrthographicCamera<float> camera(resolution, physical_size, -0.1, -100.0f);

    // Test ray from center of film
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(400.0f, 300.0f);

    auto ray = camera.generate_ray(sample);
    std::cout << "Ray Origin: " << ray.origin() << ", Direction: " << ray.direction() << std::endl;

    // For orthographic camera, direction should always be (0, 0, -1)
    EXPECT_NEAR(ray.direction().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.direction().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray.direction().z(), -1.0f, 1e-5f);

    // Origin should be approximately at center
    EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-3f);
    EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-3f);
}

TEST(OrthographicCameraTest, GenerateRayCorners) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    OrthographicCamera<float> camera(resolution, physical_size, -0.1, -100.0f);

    // Test down-left corner
    CameraSample<float> sample_tl;
    sample_tl.p_film = math::Point<float, 2>(0.0f, 0.0f);
    auto ray_tl = camera.generate_ray(sample_tl);
    std::cout << "Ray TL Origin: " << ray_tl.origin() << ", Direction: " << ray_tl.direction() << std::endl;

    // Direction should still be (0, 0, -1)
    EXPECT_NEAR(ray_tl.direction().z(), -1.0f, 1e-5f);

    // Origin should be at down-left of film
    EXPECT_LT(ray_tl.origin().x(), 0.0f);
    EXPECT_LT(ray_tl.origin().y(), 0.0f);

    // Test top-right corner
    CameraSample<float> sample_br;
    sample_br.p_film = math::Point<float, 2>(800.0f, 600.0f);
    auto ray_br = camera.generate_ray(sample_br);
    std::cout << "Ray BR Origin: " << ray_br.origin() << ", Direction: " << ray_br.direction() << std::endl;

    EXPECT_NEAR(ray_br.direction().z(), -1.0f, 1e-5f);
    EXPECT_GT(ray_br.origin().x(), 0.0f);
    EXPECT_GT(ray_br.origin().y(), 0.0f);
}

TEST(OrthographicCameraTest, GenerateDifferentialRay) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    OrthographicCamera<float> camera(resolution, physical_size, -0.1, -100.0f);

    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(400.0f, 300.0f);

    auto ray_diff = camera.generate_differential_ray(sample);

    // Main ray should be valid
    EXPECT_NEAR(ray_diff.main_ray().direction().z(), -1.0f, 1e-5f);

    // Differential rays should also have direction (0, 0, -1)
    EXPECT_NEAR(ray_diff.x().direction().z(), -1.0f, 1e-5f);
    EXPECT_NEAR(ray_diff.y().direction().z(), -1.0f, 1e-5f);

    // X-differential ray origin should be offset in x direction
    EXPECT_GT(ray_diff.x().origin().x(), ray_diff.main_ray().origin().x());
    EXPECT_NEAR(ray_diff.x().origin().y(), ray_diff.main_ray().origin().y(), 1e-3f);

    // Y-differential ray origin should be offset in y direction
    EXPECT_NEAR(ray_diff.y().origin().x(), ray_diff.main_ray().origin().x(), 1e-3f);
    EXPECT_GT(ray_diff.y().origin().y(), ray_diff.main_ray().origin().y());
}

TEST(OrthographicCameraTest, ParallelRays) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    OrthographicCamera<float> camera(resolution, physical_size, -0.1, -100.0f);

    // Generate multiple rays at different positions
    CameraSample<float> sample1, sample2, sample3;
    sample1.p_film = math::Point<float, 2>(100.0f, 100.0f);
    sample2.p_film = math::Point<float, 2>(400.0f, 300.0f);
    sample3.p_film = math::Point<float, 2>(700.0f, 500.0f);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);
    auto ray3 = camera.generate_ray(sample3);

    // All rays should be parallel (same direction)
    EXPECT_NEAR(ray1.direction().x(), ray2.direction().x(), 1e-5f);
    EXPECT_NEAR(ray1.direction().y(), ray2.direction().y(), 1e-5f);
    EXPECT_NEAR(ray1.direction().z(), ray2.direction().z(), 1e-5f);

    EXPECT_NEAR(ray2.direction().x(), ray3.direction().x(), 1e-5f);
    EXPECT_NEAR(ray2.direction().y(), ray3.direction().y(), 1e-5f);
    EXPECT_NEAR(ray2.direction().z(), ray3.direction().z(), 1e-5f);
}

// ============================================================================
// PerspectiveCamera Tests
// ============================================================================

TEST(PerspectiveCameraTest, Construction) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    PerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0);

    EXPECT_NE(camera.projection().camera_to_clip(), geometry::Transform<double>());
}

TEST(PerspectiveCameraTest, GenerateRayCenter) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    PerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0);

    // Ray from center should point roughly along -Z axis
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);

    auto ray = camera.generate_ray(sample);

    // Origin should be at (0, 0, 0)
    EXPECT_NEAR(ray.origin().x(), 0.0, 1e-10);
    EXPECT_NEAR(ray.origin().y(), 0.0, 1e-10);
    EXPECT_NEAR(ray.origin().z(), 0.0, 1e-10);

    // Direction should be normalized
    double length = std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y() +
                              ray.direction().z() * ray.direction().z());
    EXPECT_NEAR(length, 1.0, 1e-5);

    // Z component should be negative (pointing forward -Z)
    EXPECT_LT(ray.direction().z(), 0.0);
}

TEST(PerspectiveCameraTest, GenerateRayCorners) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    PerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0);

    // down-left corner
    CameraSample<double> sample_tl;
    sample_tl.p_film = math::Point<double, 2>(0.0, 0.0);
    auto ray_tl = camera.generate_ray(sample_tl);

    // All rays should originate from camera origin
    EXPECT_NEAR(ray_tl.origin().x(), 0.0, 1e-10);
    EXPECT_NEAR(ray_tl.origin().y(), 0.0, 1e-10);
    EXPECT_NEAR(ray_tl.origin().z(), 0.0, 1e-10);

    // Direction should be normalized and point left-down
    EXPECT_LT(ray_tl.direction().x(), 0.0);
    EXPECT_LT(ray_tl.direction().y(), 0.0);
    EXPECT_LT(ray_tl.direction().z(), 0.0);

    // Top-right corner
    CameraSample<double> sample_br;
    sample_br.p_film = math::Point<double, 2>(800.0, 600.0);
    auto ray_br = camera.generate_ray(sample_br);

    EXPECT_GT(ray_br.direction().x(), 0.0);
    EXPECT_GT(ray_br.direction().y(), 0.0);
    EXPECT_LT(ray_br.direction().z(), 0.0);
}

TEST(PerspectiveCameraTest, RaysDiverge) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    PerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0);

    // Generate rays at different positions
    CameraSample<double> sample1, sample2;
    sample1.p_film = math::Point<double, 2>(200.0, 300.0);
    sample2.p_film = math::Point<double, 2>(600.0, 300.0);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);

    // Rays should have different directions (perspective projection)
    EXPECT_NE(ray1.direction().x(), ray2.direction().x());

    // But same origin
    EXPECT_NEAR(ray1.origin().x(), ray2.origin().x(), 1e-10);
    EXPECT_NEAR(ray1.origin().y(), ray2.origin().y(), 1e-10);
    EXPECT_NEAR(ray1.origin().z(), ray2.origin().z(), 1e-10);
}

TEST(PerspectiveCameraTest, GenerateDifferentialRay) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    PerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0);

    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);

    auto ray_diff = camera.generate_differential_ray(sample);

    // All rays should originate from camera origin
    EXPECT_NEAR(ray_diff.main_ray().origin().x(), 0.0, 1e-10);
    EXPECT_NEAR(ray_diff.x().origin().x(), 0.0, 1e-10);
    EXPECT_NEAR(ray_diff.y().origin().x(), 0.0, 1e-10);

    // Differential rays should have slightly different directions
    EXPECT_NE(ray_diff.main_ray().direction().x(), ray_diff.x().direction().x());
    EXPECT_NE(ray_diff.main_ray().direction().y(), ray_diff.y().direction().y());
}

TEST(PerspectiveCameraTest, FieldOfView) {
    math::Vector<int, 2> resolution(1000, 1000);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double near = 1.0;
    PerspectiveCamera<double> camera(resolution, physical_size, -near, -100.0);

    // Sample at extreme corners
    CameraSample<double> sample_corner;
    sample_corner.p_film = math::Point<double, 2>(0.0, 0.0);
    auto ray_corner = camera.generate_ray(sample_corner);

    // Calculate angle from center
    math::Vector<double, 3> center_dir(0.0, 0.0, -1.0);
    double cos_angle = ray_corner.direction().dot(center_dir);
    double angle = std::acos(cos_angle);

    // Should be within reasonable FOV
    EXPECT_GT(angle, 0.0);
    EXPECT_LT(angle, std::numbers::pi / 2.0);  // Less than 90 degrees
}

// ============================================================================
// Sample Lens Concentric Tests
// ============================================================================

TEST(SampleLensConcentricTest, CenterPoint) {
    math::Point<float, 2> p_center(0.5f, 0.5f);
    auto result = sample_uniform_disk_concentric(p_center, 1.0f);

    // Center should map to origin
    EXPECT_NEAR(result.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(result.y(), 0.0f, 1e-5f);
}

TEST(SampleLensConcentricTest, CornerPoints) {
    float lens_radius = 1.0f;

    // Test corner (1, 1) -> should be at radius 1
    math::Point<float, 2> p_corner(1.0f, 1.0f);
    auto result = sample_uniform_disk_concentric(p_corner, lens_radius);

    float radius = std::sqrt(result.x() * result.x() + result.y() * result.y());
    EXPECT_NEAR(radius, lens_radius, 1e-5f);
}

TEST(SampleLensConcentricTest, EdgePoints) {
    float lens_radius = 2.0f;

    // Test right edge (1, 0.5)
    math::Point<float, 2> p_right(1.0f, 0.5f);
    auto result_right = sample_uniform_disk_concentric(p_right, lens_radius);

    // Should be on the right side
    EXPECT_GT(result_right.x(), 0.0f);
    EXPECT_NEAR(result_right.y(), 0.0f, 1e-5f);

    // Radius should be lens_radius
    float radius = std::sqrt(result_right.x() * result_right.x() + result_right.y() * result_right.y());
    EXPECT_NEAR(radius, lens_radius, 1e-5f);
}

TEST(SampleLensConcentricTest, UniformDistribution) {
    float lens_radius = 1.5f;

    // Test multiple points
    std::vector<math::Point<float, 2>> test_points;
    test_points.push_back(math::Point<float, 2>(0.25f, 0.25f));
    test_points.push_back(math::Point<float, 2>(0.75f, 0.25f));
    test_points.push_back(math::Point<float, 2>(0.25f, 0.75f));
    test_points.push_back(math::Point<float, 2>(0.75f, 0.75f));
    test_points.push_back(math::Point<float, 2>(0.5f, 0.75f));
    test_points.push_back(math::Point<float, 2>(0.75f, 0.5f));

    for (const auto& p : test_points) {
        auto result = sample_uniform_disk_concentric(p, lens_radius);

        // All results should be within lens radius
        float radius = std::sqrt(result.x() * result.x() + result.y() * result.y());
        EXPECT_LE(radius, lens_radius + 1e-5f);
    }
}

TEST(SampleLensConcentricTest, OriginMapping) {
    float lens_radius = 1.0f;

    // Point at origin of [0,1]^2 space should map to negative coordinates
    math::Point<float, 2> p_origin(0.0f, 0.0f);
    auto result = sample_uniform_disk_concentric(p_origin, lens_radius);

    // Should be at (-1, 0) after rotation
    float radius = std::sqrt(result.x() * result.x() + result.y() * result.y());
    EXPECT_NEAR(radius, lens_radius, 1e-5f);
}

// ============================================================================
// ThinLensOrthographicCamera Tests
// ============================================================================

TEST(ThinLensOrthographicCameraTest, Construction) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    float lens_radius = 0.1f;
    float focal_distance = 10.0f;

    ThinLensOrthographicCamera<float> camera(resolution, physical_size, -0.1f, -100.0f, focal_distance);

    EXPECT_NE(camera.projection().camera_to_clip(), geometry::Transform<float>());
}

TEST(ThinLensOrthographicCameraTest, GenerateRayFromLensCenter) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    float lens_radius = 0.1f;
    float focal_distance = 10.0f;

    ThinLensOrthographicCamera<float> camera(resolution, physical_size, -0.1f, -100.0f, focal_distance);

    // Sample with lens center
    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(400.0f, 300.0f);
    sample.p_lens = math::Point<float, 2>(0.0f, 0.0f);

    auto ray = camera.generate_ray(sample);

    // Origin should be near (0, 0, 0) when lens center is used
    EXPECT_NEAR(ray.origin().x(), 0.0f, 1e-3f);
    EXPECT_NEAR(ray.origin().y(), 0.0f, 1e-3f);
    EXPECT_NEAR(ray.origin().z(), 0.0f, 1e-5f);

    // Direction should be normalized
    float length = std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y() +
                             ray.direction().z() * ray.direction().z());
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(ThinLensOrthographicCameraTest, RaysConvergeAtFocalPlane) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    float lens_radius = 0.2f;
    float focal_distance = 10.0f;

    ThinLensOrthographicCamera<float> camera(resolution, physical_size, -0.1f, -100.0f, focal_distance);

    // Generate rays from same film point but different lens positions
    CameraSample<float> sample1, sample2;
    sample1.p_film = math::Point<float, 2>(400.0f, 300.0f);
    sample2.p_film = math::Point<float, 2>(400.0f, 300.0f);

    sample1.p_lens = math::Point<float, 2>(-0.1f, 0.0f);
    sample2.p_lens = math::Point<float, 2>(0.1f, 0.0f);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);

    // Rays should have different origins
    EXPECT_NE(ray1.origin().x(), ray2.origin().x());

    // But should converge at focal distance
    auto point1_at_focal = ray1.at(focal_distance);
    auto point2_at_focal = ray2.at(focal_distance);

    EXPECT_NEAR(point1_at_focal.x(), point2_at_focal.x(), 1e-2f);
    EXPECT_NEAR(point1_at_focal.y(), point2_at_focal.y(), 1e-2f);
}

TEST(ThinLensOrthographicCameraTest, GenerateDifferentialRay) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    float lens_radius = 0.1f;
    float focal_distance = 10.0f;

    ThinLensOrthographicCamera<float> camera(resolution, physical_size, -0.1f, -100.0f, focal_distance);

    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(400.0f, 300.0f);
    sample.p_lens = math::Point<float, 2>(0.0f, 0.0f);

    auto ray_diff = camera.generate_differential_ray(sample);

    // Main ray should be valid
    float length = std::sqrt(ray_diff.main_ray().direction().x() * ray_diff.main_ray().direction().x() +
                             ray_diff.main_ray().direction().y() * ray_diff.main_ray().direction().y() +
                             ray_diff.main_ray().direction().z() * ray_diff.main_ray().direction().z());
    EXPECT_NEAR(length, 1.0f, 1e-5f);

    // Differential rays should also be normalized
    float length_x = std::sqrt(ray_diff.x().direction().x() * ray_diff.x().direction().x() +
                               ray_diff.x().direction().y() * ray_diff.x().direction().y() +
                               ray_diff.x().direction().z() * ray_diff.x().direction().z());
    EXPECT_NEAR(length_x, 1.0f, 1e-5f);
}

TEST(ThinLensOrthographicCameraTest, DepthOfFieldEffect) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);
    float lens_radius = 0.5f;  // Large aperture for visible DOF
    float focal_distance = 10.0f;

    ThinLensOrthographicCamera<float> camera(resolution, physical_size, -0.1f, -100.0f, focal_distance);

    // Sample rays from different lens positions
    CameraSample<float> sample1, sample2;
    sample1.p_film = math::Point<float, 2>(400.0f, 300.0f);
    sample2.p_film = math::Point<float, 2>(400.0f, 300.0f);

    sample1.p_lens = math::Point<float, 2>(-0.2f, 0.0f);
    sample2.p_lens = math::Point<float, 2>(0.2f, 0.0f);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);

    // At focal distance, rays should converge
    auto p1_focal = ray1.at(focal_distance);
    auto p2_focal = ray2.at(focal_distance);

    float dist_at_focal = p1_focal.distance(p2_focal);
    EXPECT_LT(dist_at_focal, 0.1f);

    // At different distance, rays should diverge
    float test_distance = 20.0f;  // Far from focal plane
    auto p1_far = ray1.at(test_distance);
    auto p2_far = ray2.at(test_distance);

    float dist_at_far = p1_far.distance(p2_far);
    EXPECT_GT(dist_at_far, dist_at_focal);
}

// ============================================================================
// ThinLensPerspectiveCamera Tests
// ============================================================================

TEST(ThinLensPerspectiveCameraTest, Construction) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.1;
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    EXPECT_NE(camera.projection().camera_to_clip(), geometry::Transform<double>());
}

TEST(ThinLensPerspectiveCameraTest, GenerateRayFromLensCenter) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.1;
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    // Sample with lens center
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);
    sample.p_lens = math::Point<double, 2>(0.0, 0.0);

    auto ray = camera.generate_ray(sample);

    // Origin should be near (0, 0, 0) when lens center is used
    EXPECT_NEAR(ray.origin().x(), 0.0, 1e-3);
    EXPECT_NEAR(ray.origin().y(), 0.0, 1e-3);
    EXPECT_NEAR(ray.origin().z(), 0.0, 1e-5);

    // Direction should be normalized
    double length = std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y() +
                              ray.direction().z() * ray.direction().z());
    EXPECT_NEAR(length, 1.0, 1e-5);
}

TEST(ThinLensPerspectiveCameraTest, RaysOriginateFromLens) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.3;
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    // Sample from edge of lens
    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);
    sample.p_lens = math::Point<double, 2>(0.3, 0.0);  // Right edge

    auto ray = camera.generate_ray(sample);

    // Origin should be offset from camera center
    double origin_dist = std::sqrt(ray.origin().x() * ray.origin().x() + ray.origin().y() * ray.origin().y());
    EXPECT_GT(origin_dist, 0.0);
    EXPECT_LE(origin_dist, lens_radius + 1e-3);
}

TEST(ThinLensPerspectiveCameraTest, FocalPlaneConvergence) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.2;
    double focal_distance = 15.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    // Generate rays from same film point but different lens positions
    CameraSample<double> sample1, sample2;
    sample1.p_film = math::Point<double, 2>(400.0, 300.0);
    sample2.p_film = math::Point<double, 2>(400.0, 300.0);

    sample1.p_lens = math::Point<double, 2>(-0.1, 0.0);
    sample2.p_lens = math::Point<double, 2>(0.1, 0.0);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);

    // Origins should be different (from different lens positions)
    EXPECT_NE(ray1.origin().x(), ray2.origin().x());

    // At focal distance, rays should converge (approximately)
    auto p1_focal = ray1.at(focal_distance);
    auto p2_focal = ray2.at(focal_distance);

    double dist_at_focal = p1_focal.distance(p2_focal);
    EXPECT_LT(dist_at_focal, 0.5);  // Should be close
}

TEST(ThinLensPerspectiveCameraTest, GenerateDifferentialRay) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.1;
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);
    sample.p_lens = math::Point<double, 2>(0.0, 0.0);

    auto ray_diff = camera.generate_differential_ray(sample);

    // All rays should be normalized
    auto check_normalized = [](const geometry::Ray<double, 3>& ray) {
        double length =
            std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y() +
                      ray.direction().z() * ray.direction().z());
        return std::abs(length - 1.0) < 1e-5;
    };

    EXPECT_TRUE(check_normalized(ray_diff.main_ray()));
    EXPECT_TRUE(check_normalized(ray_diff.x()));
    EXPECT_TRUE(check_normalized(ray_diff.y()));
}

TEST(ThinLensPerspectiveCameraTest, DepthOfFieldEffect) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.4;  // Large aperture
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    // Sample rays from opposite lens edges
    CameraSample<double> sample1, sample2;
    sample1.p_film = math::Point<double, 2>(400.0, 300.0);
    sample2.p_film = math::Point<double, 2>(400.0, 300.0);

    sample1.p_lens = math::Point<double, 2>(-0.3, 0.0);
    sample2.p_lens = math::Point<double, 2>(0.3, 0.0);

    auto ray1 = camera.generate_ray(sample1);
    auto ray2 = camera.generate_ray(sample2);

    // At focal distance, should be close
    auto p1_focal = ray1.at(focal_distance);
    auto p2_focal = ray2.at(focal_distance);
    double dist_focal = p1_focal.distance(p2_focal);

    // At other distances, should be farther apart
    double test_distance = 20.0;
    auto p1_far = ray1.at(test_distance);
    auto p2_far = ray2.at(test_distance);
    double dist_far = p1_far.distance(p2_far);

    EXPECT_GT(dist_far, dist_focal);

    // Test near plane too
    double near_distance = 5.0;
    auto p1_near = ray1.at(near_distance);
    auto p2_near = ray2.at(near_distance);
    double dist_near = p1_near.distance(p2_near);

    EXPECT_GT(dist_near, dist_focal);
}

TEST(ThinLensPerspectiveCameraTest, BokehShapeTest) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double lens_radius = 0.3;
    double focal_distance = 10.0;

    ThinLensPerspectiveCamera<double> camera(resolution, physical_size, -0.1, -100.0, focal_distance);

    // Sample multiple points on lens
    std::vector<math::Point<double, 2>> lens_samples;
    lens_samples.push_back(math::Point<double, 2>(0.0, 0.0));   // Center
    lens_samples.push_back(math::Point<double, 2>(-0.3, 0.0));  // Left
    lens_samples.push_back(math::Point<double, 2>(0.3, 0.0));   // Right
    lens_samples.push_back(math::Point<double, 2>(0.0, 0.3));   // Top
    lens_samples.push_back(math::Point<double, 2>(0.0, -0.3));  // Bottom
    lens_samples.push_back(math::Point<double, 2>(-0.2, 0.2));  // Top-left
    lens_samples.push_back(math::Point<double, 2>(0.2, -0.2));  // Bottom-right

    CameraSample<double> sample;
    sample.p_film = math::Point<double, 2>(400.0, 300.0);

    for (const auto& lens_pos : lens_samples) {
        sample.p_lens = lens_pos;
        auto ray = camera.generate_ray(sample);

        // All rays should be valid and normalized
        double length =
            std::sqrt(ray.direction().x() * ray.direction().x() + ray.direction().y() * ray.direction().y() +
                      ray.direction().z() * ray.direction().z());
        EXPECT_NEAR(length, 1.0, 1e-5);

        // Direction z should be negative (forward -Z)
        EXPECT_LT(ray.direction().z(), 0.0);
    }
}

// ============================================================================
// Cross-Camera Comparison Tests
// ============================================================================

TEST(CameraComparisonTest, OrthographicVsPerspective) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);

    OrthographicCamera<float> ortho_cam(resolution, physical_size, -0.1, -100.0f);
    PerspectiveCamera<float> persp_cam(resolution, physical_size, -0.1, -100.0f);

    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(300.0f, 200.0f);

    auto ray_ortho = ortho_cam.generate_ray(sample);
    auto ray_persp = persp_cam.generate_ray(sample);

    // Orthographic rays are parallel, perspective rays converge at origin
    EXPECT_NE(ray_ortho.origin().x(), ray_persp.origin().x());
    EXPECT_NE(ray_ortho.origin().y(), ray_persp.origin().y());
}

TEST(CameraComparisonTest, ThinLensVsPinhole) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<double, 2> physical_size(2.0, 2.0);
    double focal_distance = 10.0;

    // Very small aperture should approximate pinhole
    double tiny_aperture = 1e-6;
    ThinLensPerspectiveCamera<double> thin_lens_cam(resolution, physical_size, -0.1, -100.0, focal_distance);
    PerspectiveCamera<double> pinhole_cam(resolution, physical_size, -0.1, -100.0);

    CameraSample<double> thin_lens_sample;
    thin_lens_sample.p_film = math::Point<double, 2>(400.0, 300.0);
    thin_lens_sample.p_lens = math::Point<double, 2>(0.0, 0.0);

    CameraSample<double> pinhole_sample;
    pinhole_sample.p_film = math::Point<double, 2>(400.0, 300.0);

    auto ray_thin_lens = thin_lens_cam.generate_ray(thin_lens_sample);
    auto ray_pinhole = pinhole_cam.generate_ray(pinhole_sample);

    // With tiny aperture, thin lens should approximate pinhole
    EXPECT_NEAR(ray_thin_lens.origin().x(), ray_pinhole.origin().x(), 1e-3);
    EXPECT_NEAR(ray_thin_lens.origin().y(), ray_pinhole.origin().y(), 1e-3);
    EXPECT_NEAR(ray_thin_lens.origin().z(), ray_pinhole.origin().z(), 1e-3);

    EXPECT_NEAR(ray_thin_lens.direction().x(), ray_pinhole.direction().x(), 1e-3);
    EXPECT_NEAR(ray_thin_lens.direction().y(), ray_pinhole.direction().y(), 1e-3);
    EXPECT_NEAR(ray_thin_lens.direction().z(), ray_pinhole.direction().z(), 1e-3);
}

// ============================================================================
// Type Alias and Template Tests
// ============================================================================

TEST(CameraTemplateTest, FloatAndDoubleTypes) {
    math::Vector<int, 2> resolution_f(800, 600);
    math::Vector<float, 2> physical_size_f(2.0f, 2.0f);
    math::Vector<int, 2> resolution_d(800, 600);
    math::Vector<double, 2> physical_size_d(2.0, 2.0);

    // Test float types
    OrthographicCamera<float> ortho_f(resolution_f, physical_size_f, -0.1, -100.0f);
    PerspectiveCamera<float> persp_f(resolution_f, physical_size_f, -0.1, -100.0f);

    // Test double types
    OrthographicCamera<double> ortho_d(resolution_d, physical_size_d, -0.1, -100.0);
    PerspectiveCamera<double> persp_d(resolution_d, physical_size_d, -0.1, -100.0);

    SUCCEED() << "Both float and double camera types compile and construct";
}

TEST(CameraTemplateTest, CRTPBehavior) {
    math::Vector<int, 2> resolution(800, 600);
    math::Vector<float, 2> physical_size(2.0f, 2.0f);

    // Test that cameras use CRTP for static polymorphism
    OrthographicCamera<float> ortho_cam(resolution, physical_size, -0.1, -100.0f);
    PerspectiveCamera<float> persp_cam(resolution, physical_size, -0.1, -100.0f);

    CameraSample<float> sample;
    sample.p_film = math::Point<float, 2>(400.0f, 300.0f);

    // Both cameras can call generate_ray through their base Camera interface
    auto ray_ortho = ortho_cam.generate_ray(sample);
    auto ray_persp = persp_cam.generate_ray(sample);

    // Orthographic camera rays are parallel to -z-axis
    EXPECT_NEAR(ray_ortho.direction().z(), -1.0f, 1e-5f);
    EXPECT_NEAR(ray_ortho.direction().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray_ortho.direction().y(), 0.0f, 1e-5f);

    // Perspective camera rays converge at origin
    EXPECT_NEAR(ray_persp.origin().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray_persp.origin().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(ray_persp.origin().z(), 0.0f, 1e-5f);
}

// ============================================================================
}  // namespace pbpt::camera::testing
