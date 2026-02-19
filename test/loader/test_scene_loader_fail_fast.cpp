#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "pbpt/loader/scene_loader.hpp"

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

void write_basic_obj(const TempDir& dir) {
    const auto path = dir.path / "tri.obj";
    write_text_file(path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
}

}  // namespace

TEST(SceneLoaderFailFastTest, ThrowsOnDuplicateBsdfId) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_duplicate_bsdf");
    write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_duplicate_bsdf.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <rgb name="reflectance" value="0.8,0.8,0.8"/>
  </bsdf>
  <bsdf type="diffuse" id="mat_white">
    <rgb name="reflectance" value="0.2,0.2,0.2"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML");

    EXPECT_THROW((void)pbpt::loader::load_scene<double>(xml_path.string()), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsOnUnknownShapeMaterialReference) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_unknown_ref");
    write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_unknown_material_ref.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <rgb name="reflectance" value="0.8,0.8,0.8"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_missing"/>
  </shape>
</scene>)XML");

    EXPECT_THROW((void)pbpt::loader::load_scene<double>(xml_path.string()), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsWhenMeshHasNoMaterialAssignment) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_no_material_assignment");
    write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_no_material_assignment.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <rgb name="reflectance" value="0.8,0.8,0.8"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
  </shape>
</scene>)XML");

    EXPECT_THROW((void)pbpt::loader::load_scene<double>(xml_path.string()), std::runtime_error);
}
