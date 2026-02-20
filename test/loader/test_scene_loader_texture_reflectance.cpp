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

void write_basic_obj(const std::filesystem::path& path) {
    write_text_file(path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nf 1/1 2/2 3/3\n");
}

}  // namespace

TEST(SceneLoaderTextureReflectanceTest, LoadsDiffuseReflectanceTextureReference) {
    TempDir temp_dir("pbpt_scene_loader_texture_reflectance");
    write_basic_obj(temp_dir.path / "tri.obj");

    const auto xml_path = temp_dir.path / "scene_texture_reflectance.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <texture type="checkerboard" id="tex_checker">
    <rgb name="color0" value="0.2"/>
    <rgb name="color1" value="0.8"/>
    <float name="uscale" value="4"/>
    <float name="vscale" value="4"/>
  </texture>
  <bsdf type="diffuse" id="mat_tex">
    <ref name="reflectance" id="tex_checker"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_tex"/>
  </shape>
</scene>)XML");

    const auto result = pbpt::serde::load_scene<double>(xml_path.string());
    ASSERT_TRUE(result.scene.resources.reflectance_texture_library.name_to_id().contains("tex_checker"));
    ASSERT_TRUE(result.scene.resources.any_material_library.name_to_id().contains("mat_tex"));
    const auto& material = result.scene.resources.any_material_library.get("mat_tex");
    EXPECT_TRUE((std::holds_alternative<pbpt::material::LambertianMaterial<double>>(material)));
}

TEST(SceneLoaderTextureReflectanceTest, ThrowsOnMissingReflectanceTextureReference) {
    TempDir temp_dir("pbpt_scene_loader_texture_reflectance_missing");
    write_basic_obj(temp_dir.path / "tri.obj");

    const auto xml_path = temp_dir.path / "scene_texture_reflectance_missing.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_tex">
    <ref name="reflectance" id="missing_tex"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_tex"/>
  </shape>
</scene>)XML");

    EXPECT_THROW((void)pbpt::serde::load_scene<double>(xml_path.string()), std::runtime_error);
}
