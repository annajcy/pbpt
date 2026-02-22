#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <variant>

#include <gtest/gtest.h>

#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name) : path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

void write_text_file(const std::filesystem::path& path, const std::string& content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    out << content;
}

std::filesystem::path write_basic_obj(const TempDir& dir) {
    const auto path = dir.path / "tri.obj";
    write_text_file(path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    return path;
}

}  // namespace

TEST(SceneLoaderReflectanceRgbTest, LoadsDiffuseRgbReflectanceAsLambertian) {
    TempDir temp_dir("pbpt_scene_loader_reflectance_rgb");
    (void)write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_rgb_reflectance.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_rgb">
    <rgb name="reflectance" value="0.2 0.4 0.6"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_rgb"/>
  </shape>
</scene>)XML");

    const auto result = pbpt::serde::load_scene<double>(xml_path.string());

    ASSERT_TRUE(result.scene.resources.any_material_library.name_to_id().contains("mat_rgb"));
    const auto& material = result.scene.resources.any_material_library.get("mat_rgb");
    EXPECT_TRUE((std::holds_alternative<pbpt::material::LambertianMaterial<double>>(material)));

    const auto& reflectance = result.scene.resources.reflectance_spectrum_library.get("mat_rgb_reflectance");
    const double s360 = reflectance.at(360.0);
    const double s600 = reflectance.at(600.0);
    const double s830 = reflectance.at(830.0);
    EXPECT_GE(s360, 0.0);
    EXPECT_GE(s600, 0.0);
    EXPECT_GE(s830, 0.0);
    EXPECT_LE(s360, 1.0);
    EXPECT_LE(s600, 1.0);
    EXPECT_LE(s830, 1.0);
    EXPECT_GT(std::abs(s360 - s830), 1e-6);
}

TEST(SceneLoaderReflectanceRgbTest, DiffuseMissingReflectanceUsesConstant07Spectrum) {
    TempDir temp_dir("pbpt_scene_loader_reflectance_default");
    (void)write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_default_reflectance.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_default"/>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_default"/>
  </shape>
</scene>)XML");

    const auto result = pbpt::serde::load_scene<double>(xml_path.string());
    const auto& reflectance = result.scene.resources.reflectance_spectrum_library.get("mat_default_reflectance");
    EXPECT_NEAR(reflectance.at(400.0), 0.7, 1e-9);
    EXPECT_NEAR(reflectance.at(700.0), 0.7, 1e-9);
}
