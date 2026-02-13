#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "pbpt/pbpt.h"
#include "pbpt/integrator/domain.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/loader/scene_loader.hpp"

namespace pbpt::integrator::testing {

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& prefix) {
        const auto stamp = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()
        );
        path = std::filesystem::temp_directory_path() / (prefix + "_" + stamp);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

std::string write_tiny_cbox_scene_xml(const std::filesystem::path& xml_path) {
    const auto pbpt_root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto mesh_dir = pbpt_root / "asset" / "scene" / "cbox" / "meshes";
    const auto floor_obj = (mesh_dir / "cbox_floor.obj").generic_string();
    const auto light_obj = (mesh_dir / "cbox_luminaire.obj").generic_string();

    std::ofstream out(xml_path);
    if (!out) {
        throw std::runtime_error("Failed to open temp scene xml: " + xml_path.string());
    }

    out
        << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        << "<scene version=\"0.4.0\">\n"
        << "  <integrator type=\"path\">\n"
        << "    <integer name=\"maxDepth\" value=\"2\"/>\n"
        << "  </integrator>\n"
        << "  <sensor type=\"perspective\">\n"
        << "    <string name=\"fovAxis\" value=\"smaller\"/>\n"
        << "    <float name=\"nearClip\" value=\"10\"/>\n"
        << "    <float name=\"farClip\" value=\"2800\"/>\n"
        << "    <float name=\"focusDistance\" value=\"1000\"/>\n"
        << "    <transform name=\"toWorld\">\n"
        << "      <lookAt origin=\"278, 273, -800\" target=\"278, 273, -799\" up=\"0, 1, 0\"/>\n"
        << "    </transform>\n"
        << "    <float name=\"fov\" value=\"39.3077\"/>\n"
        << "    <sampler type=\"ldsampler\">\n"
        << "      <integer name=\"sampleCount\" value=\"1\"/>\n"
        << "    </sampler>\n"
        << "    <film type=\"hdrfilm\">\n"
        << "      <integer name=\"width\" value=\"8\"/>\n"
        << "      <integer name=\"height\" value=\"8\"/>\n"
        << "      <rfilter type=\"gaussian\"/>\n"
        << "    </film>\n"
        << "  </sensor>\n"
        << "  <bsdf type=\"diffuse\" id=\"white\">\n"
        << "    <spectrum name=\"reflectance\" value=\"400:0.7, 500:0.7, 600:0.7, 700:0.7\"/>\n"
        << "  </bsdf>\n"
        << "  <bsdf type=\"diffuse\" id=\"light\">\n"
        << "    <spectrum name=\"reflectance\" value=\"400:0.78, 500:0.78, 600:0.78, 700:0.78\"/>\n"
        << "  </bsdf>\n"
        << "  <shape type=\"obj\">\n"
        << "    <string name=\"filename\" value=\"" << light_obj << "\"/>\n"
        << "    <ref id=\"light\"/>\n"
        << "    <emitter type=\"area\">\n"
        << "      <spectrum name=\"radiance\" value=\"400:0, 500:8, 600:15.6, 700:18.4\"/>\n"
        << "    </emitter>\n"
        << "  </shape>\n"
        << "  <shape type=\"obj\">\n"
        << "    <string name=\"filename\" value=\"" << floor_obj << "\"/>\n"
        << "    <ref id=\"white\"/>\n"
        << "  </shape>\n"
        << "</scene>\n";
    out.close();

    return xml_path.string();
}

} // namespace

TEST(Radiometric, UniformHemisphereIntegral_Cosine)
{
    math::RandomGenerator<double, 2> rng2d(123);
    UniformHemisphereDomain<double> hemisphere;
    math::Normal3 n(0.0, 0.0, 1.0);
    int sample_count = 1000000;
    auto res = integrate<double>(hemisphere, [&n](const math::Vector<double, 3>& wi) {
        return n.to_vector().dot(wi);
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(Radiometric, CosineWeightedHemisphereIntegral_Cosine)
{
    math::RandomGenerator<double, 2> rng2d(789);
    CosineWeightedHemisphereDomain<double> proj_hemi;
    math::Normal3 n(0.0, 0.0, 1.0);
    int sample_count = 100000;
    auto res = integrate<double>(proj_hemi, [&n](const math::Vector<double, 3>& wi) {
        return n.to_vector().dot(wi);
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(Radiometric, HemisphereCosineVarianceComparison)
{
    math::Normal3 n(0.0, 0.0, 1.0);
    const auto n_vec = n.to_vector();
    constexpr int trials = 64;
    constexpr int sample_count = 4096;

    auto variance_of_estimator = [&](auto&& domain_factory) {
        std::vector<double> estimates;
        estimates.reserve(trials);
        for (int i = 0; i < trials; ++i) {
            math::RandomGenerator<double, 2> rng2d(1000 + i);
            auto domain = domain_factory();
            double estimate = integrate<double>(domain, [&n_vec](const math::Vector<double, 3>& wi) {
                return n_vec.dot(wi);
            }, sample_count, rng2d);
            estimates.push_back(estimate);
        }

        double mean = std::accumulate(estimates.begin(), estimates.end(), 0.0) / static_cast<double>(estimates.size());
        double var_sum = std::accumulate(estimates.begin(), estimates.end(), 0.0, [mean](double acc, double v) {
            double d = v - mean;
            return acc + d * d;
        });
        return var_sum / static_cast<double>(estimates.size() - 1);
    };

    auto var_uniform = variance_of_estimator([]() { return UniformHemisphereDomain<double>{}; });
    auto var_cosine  = variance_of_estimator([]() { return CosineWeightedHemisphereDomain<double>{}; });

    std::cout << "Variance Uniform Hemisphere: " << var_uniform << std::endl;
    std::cout << "Variance Cosine-Weighted Hemisphere: " << var_cosine << std::endl;

    EXPECT_LT(var_cosine, var_uniform);
    EXPECT_LT(var_cosine, var_uniform * 0.7); // cosine-weighted should reduce variance noticeably
}

TEST(Radiometric, UniformDiskIntegral_Constant)
{
    math::RandomGenerator<double, 2> rng2d(456);
    UniformDiskDomain<double> disk;
    int sample_count = 100000;
    auto res = integrate<double>(disk, [](const math::Point<double, 2>& p) {
        return 1.0;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, math::pi_v<double>, 0.01);
}

TEST(Radiometric, ParallelogramAreaIntegral)
{
    math::RandomGenerator<double, 2> rng2d(321);
    UniformParallelogramAreaDomain<double> para{
        math::Point<double, 3>(-1.0, 4.0, -1.0),
        math::Vector<double, 3>(2.0, 0.0, 0.0),
        math::Vector<double, 3>(0.0, 0.0, 2.0)
    };
    auto shading_p = math::Point<double, 3>(0.0, 0.0, 0.0);
    auto shading_p_normal = math::Normal3(0.0, 1.0, 0.0);
    int sample_count = 100000;
    auto res = integrate<double>(para, [&shading_p, &shading_p_normal](const SurfaceInfo<double>& surface) {
        auto [p, normal] = surface;
        auto n = normal.to_vector();
        auto pn = shading_p_normal.to_vector();
        auto wi = (p - shading_p).normalized();
        auto cos_p = pn.dot(wi);
        auto cos_x = n.dot(-wi);
        auto r2 = (p - shading_p).length_squared();
        auto L = 1.0;
        return L * cos_p * cos_x / r2;
    }, sample_count, rng2d);
    EXPECT_NEAR(res, 0.2308367977, 0.01);
}

TEST(PathIntegratorObserver, ProgressCallbacksInRangeAndMonotonic)
{
    TempDir temp_dir("pbpt_integrator_observer");
    const auto scene_xml_path = temp_dir.path / "tiny_scene.xml";
    const auto output_exr_path = temp_dir.path / "tiny_scene.exr";
    (void)write_tiny_cbox_scene_xml(scene_xml_path);

    auto scene = pbpt::loader::load_scene<double>(scene_xml_path.string());
    pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);

    std::vector<float> progress_values;
    pbpt::integrator::RenderObserver observer{};
    observer.on_progress = [&](float progress) {
        progress_values.push_back(progress);
    };
    observer.is_cancel_requested = []() {
        return false;
    };

    integrator.render(scene, 1, output_exr_path.string(), false, observer);

    ASSERT_FALSE(progress_values.empty());
    for (std::size_t i = 0; i < progress_values.size(); ++i) {
        EXPECT_GE(progress_values[i], 0.0f);
        EXPECT_LE(progress_values[i], 1.0f);
        if (i > 0) {
            EXPECT_GE(progress_values[i], progress_values[i - 1]);
        }
    }
    EXPECT_NEAR(progress_values.back(), 1.0f, 1e-5f);
    EXPECT_TRUE(std::filesystem::exists(output_exr_path));
    EXPECT_GT(std::filesystem::file_size(output_exr_path), 0);
}

TEST(PathIntegratorObserver, CancelStopsRenderAndSkipsOutputWrite)
{
    TempDir temp_dir("pbpt_integrator_cancel");
    const auto scene_xml_path = temp_dir.path / "tiny_scene.xml";
    const auto output_exr_path = temp_dir.path / "tiny_scene_canceled.exr";
    (void)write_tiny_cbox_scene_xml(scene_xml_path);

    auto scene = pbpt::loader::load_scene<double>(scene_xml_path.string());
    pbpt::integrator::PathIntegrator<double, 4> integrator(-1, 0.9);

    std::atomic<bool> cancel_requested{false};
    std::size_t progress_call_count = 0;

    pbpt::integrator::RenderObserver observer{};
    observer.on_progress = [&](float progress) {
        (void)progress;
        ++progress_call_count;
        cancel_requested.store(true);
    };
    observer.is_cancel_requested = [&]() {
        return cancel_requested.load();
    };

    EXPECT_THROW(
        integrator.render(scene, 1, output_exr_path.string(), false, observer),
        pbpt::integrator::RenderCanceled
    );
    EXPECT_GT(progress_call_count, 0);
    EXPECT_FALSE(std::filesystem::exists(output_exr_path));
}


}
