#include <filesystem>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/loader/scene_loader.hpp"
#include "pbpt/material/plugin/material/material_type.hpp"

namespace {

std::filesystem::path find_repo_root() {
    auto path = std::filesystem::current_path();
    while (!path.empty()) {
        if (std::filesystem::exists(path / "CMakeLists.txt") && std::filesystem::exists(path / "asset")) {
            return path;
        }
        const auto parent = path.parent_path();
        if (parent == path) {
            break;
        }
        path = parent;
    }
    throw std::runtime_error("Failed to find repository root from current path.");
}

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name)
        : path(std::filesystem::temp_directory_path() / ("pbpt_roundtrip_" + name)) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

template <typename T>
const pbpt::camera::ThinLensPerspectiveCamera<T>& get_thin_lens_camera(const pbpt::scene::Scene<T>& scene) {
    const auto* camera = std::get_if<pbpt::camera::ThinLensPerspectiveCamera<T>>(&scene.camera);
    if (!camera) {
        throw std::runtime_error("Scene camera is not ThinLensPerspectiveCamera.");
    }
    return *camera;
}

template <typename T>
void expect_camera_equal(const pbpt::scene::Scene<T>& lhs, const pbpt::scene::Scene<T>& rhs) {
    const auto& a = get_thin_lens_camera(lhs);
    const auto& b = get_thin_lens_camera(rhs);

    EXPECT_NEAR(a.fov_degrees(), b.fov_degrees(), 1e-6);
    EXPECT_EQ(a.fov_axis(), b.fov_axis());
    EXPECT_NEAR(a.near_clip(), b.near_clip(), 1e-6);
    EXPECT_NEAR(a.far_clip(), b.far_clip(), 1e-3);
    EXPECT_NEAR(a.focal_distance(), b.focal_distance(), 1e-6);
    EXPECT_EQ(a.film_resolution().x(), b.film_resolution().x());
    EXPECT_EQ(a.film_resolution().y(), b.film_resolution().y());
}

template <typename T>
void expect_emission_spectrum_equal(const pbpt::scene::Scene<T>& lhs, const pbpt::scene::Scene<T>& rhs,
                                    const std::string& name) {
    const auto& s0 = lhs.resources.reflectance_spectrum_library.get(name);
    const auto& s1 = rhs.resources.reflectance_spectrum_library.get(name);
    const auto& p0 = s0.points();
    const auto& p1 = s1.points();
    ASSERT_EQ(p0.size(), p1.size());
    for (std::size_t i = 0; i < p0.size(); ++i) {
        EXPECT_NEAR(p0[i].first, p1[i].first, 1e-6);
        EXPECT_NEAR(p0[i].second, p1[i].second, 1e-6);
    }
}

template <typename T>
void expect_piecewise_spectrum_equal(const pbpt::radiometry::PiecewiseLinearSpectrumDistribution<T>& a,
                                     const pbpt::radiometry::PiecewiseLinearSpectrumDistribution<T>& b,
                                     T eps = T(1e-6)) {
    const auto& pa = a.points();
    const auto& pb = b.points();
    ASSERT_EQ(pa.size(), pb.size());
    for (std::size_t i = 0; i < pa.size(); ++i) {
        EXPECT_NEAR(pa[i].first, pb[i].first, eps);
        EXPECT_NEAR(pa[i].second, pb[i].second, eps);
    }
}

template <typename T>
void expect_material_equal(const pbpt::material::AnyMaterial<T>& m0, const pbpt::material::AnyMaterial<T>& m1) {
    ASSERT_EQ(m0.index(), m1.index());
    std::visit(
        [&](const auto& a) {
            using M = std::decay_t<decltype(a)>;
            const auto& b = std::get<M>(m1);
            if constexpr (std::is_same_v<M, pbpt::material::DielectricMaterial<T>> ||
                          std::is_same_v<M, pbpt::material::DielectricRoughMaterial<T>>) {
                EXPECT_NEAR(a.eta(), b.eta(), 1e-6);
                EXPECT_NEAR(a.microfacet_model().alpha_x(), b.microfacet_model().alpha_x(), 1e-6);
                EXPECT_NEAR(a.microfacet_model().alpha_y(), b.microfacet_model().alpha_y(), 1e-6);
            } else if constexpr (std::is_same_v<M, pbpt::material::DielectricSpecularMaterial<T>>) {
                EXPECT_NEAR(a.eta(), b.eta(), 1e-6);
            } else if constexpr (std::is_same_v<M, pbpt::material::ConductorMaterial<T>> ||
                                 std::is_same_v<M, pbpt::material::ConductorRoughMaterial<T>>) {
                expect_piecewise_spectrum_equal(a.eta_dist(), b.eta_dist());
                expect_piecewise_spectrum_equal(a.k_dist(), b.k_dist());
                EXPECT_NEAR(a.microfacet_model().alpha_x(), b.microfacet_model().alpha_x(), 1e-6);
                EXPECT_NEAR(a.microfacet_model().alpha_y(), b.microfacet_model().alpha_y(), 1e-6);
            } else if constexpr (std::is_same_v<M, pbpt::material::ConductorSpecularMaterial<T>>) {
                expect_piecewise_spectrum_equal(a.eta_dist(), b.eta_dist());
                expect_piecewise_spectrum_equal(a.k_dist(), b.k_dist());
            }
        },
        m0);
}

template <typename T>
pbpt::scene::Scene<T> roundtrip_scene(const std::filesystem::path& input_scene, const TempDir& temp_dir,
                                      const std::string& output_name) {
    const auto loaded = pbpt::loader::load_scene<T>(input_scene.string());
    const auto out_path = temp_dir.path / output_name;
    pbpt::loader::write_scene(loaded, out_path.string());
    return pbpt::loader::load_scene<T>(out_path.string());
}

}  // namespace

TEST(SceneRoundTrip, DiffuseScene) {
    const auto repo = find_repo_root();
    const auto scene_path = repo / "asset/scene/cbox/cbox.xml";

    TempDir temp_dir("diffuse");
    const auto original = pbpt::loader::load_scene<double>(scene_path.string());
    const auto reloaded = roundtrip_scene<double>(scene_path, temp_dir, "cbox_roundtrip.xml");

    EXPECT_EQ(original.resources.mesh_library.size(), reloaded.resources.mesh_library.size());
    EXPECT_EQ(original.resources.any_material_library.size(), reloaded.resources.any_material_library.size());
    expect_camera_equal(original, reloaded);

    ASSERT_TRUE(original.resources.reflectance_spectrum_library.name_to_id().contains("cbox_luminaire_emission"));
    ASSERT_TRUE(reloaded.resources.reflectance_spectrum_library.name_to_id().contains("cbox_luminaire_emission"));
    expect_emission_spectrum_equal(original, reloaded, "cbox_luminaire_emission");
}

TEST(SceneRoundTrip, TextureScene) {
    const auto repo = find_repo_root();
    const auto scene_path = repo / "asset/scene/cbox/cbox_checkerboard_texture.xml";

    TempDir temp_dir("texture");
    const auto original = pbpt::loader::load_scene<double>(scene_path.string());
    const auto reloaded = roundtrip_scene<double>(scene_path, temp_dir, "cbox_checkerboard_roundtrip.xml");

    EXPECT_EQ(original.resources.reflectance_texture_library.size(), reloaded.resources.reflectance_texture_library.size());

    for (const auto& [name, id0] : original.resources.reflectance_texture_library.name_to_id()) {
        ASSERT_TRUE(reloaded.resources.reflectance_texture_library.name_to_id().contains(name));
        const int id1 = reloaded.resources.reflectance_texture_library.name_to_id().at(name);
        const auto& t0 = original.resources.reflectance_texture_library.get(id0);
        const auto& t1 = reloaded.resources.reflectance_texture_library.get(id1);
        EXPECT_EQ(t0.index(), t1.index());
    }

    expect_camera_equal(original, reloaded);
}

TEST(SceneRoundTrip, MicrofacetScene) {
    const auto repo = find_repo_root();
    const auto scene_path = repo / "asset/scene/cbox/cbox_microfacet.xml";

    TempDir temp_dir("microfacet");
    const auto original = pbpt::loader::load_scene<double>(scene_path.string());
    const auto reloaded = roundtrip_scene<double>(scene_path, temp_dir, "cbox_microfacet_roundtrip.xml");

    EXPECT_EQ(original.resources.any_material_library.size(), reloaded.resources.any_material_library.size());

    for (const auto& [name, id0] : original.resources.any_material_library.name_to_id()) {
        ASSERT_TRUE(reloaded.resources.any_material_library.name_to_id().contains(name));
        const int id1 = reloaded.resources.any_material_library.name_to_id().at(name);
        const auto& m0 = original.resources.any_material_library.get(id0);
        const auto& m1 = reloaded.resources.any_material_library.get(id1);
        expect_material_equal(m0, m1);
    }

    expect_camera_equal(original, reloaded);
}
