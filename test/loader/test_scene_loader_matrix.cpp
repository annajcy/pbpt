#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "pbpt/loader/scene_loader.hpp"

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name)
        : path(std::filesystem::temp_directory_path() / name) {
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

} // namespace

TEST(SceneLoaderMatrixTest, LoadsShapeMatrixTransform) {
    TempDir temp_dir("pbpt_scene_loader_matrix_shape");
    (void)write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_shape_matrix.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="path">
    <integer name="maxDepth" value="-1"/>
  </integrator>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <transform name="toWorld">
      <matrix value="1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1"/>
    </transform>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <transform name="toWorld">
      <matrix value="1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1"/>
    </transform>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    auto scene = pbpt::loader::load_scene<double>(xml_path.string());

    ASSERT_EQ(scene.resources.mesh_library.size(), 1u);
    const auto& mesh = scene.resources.mesh_library.get("tri");
    const auto& positions = mesh.positions();
    ASSERT_FALSE(positions.empty());
    EXPECT_NEAR(positions[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(positions[0].y(), 2.0, 1e-9);
    EXPECT_NEAR(positions[0].z(), 3.0, 1e-9);
}

TEST(SceneLoaderMatrixTest, LoadsSensorToWorldMatrix) {
    TempDir temp_dir("pbpt_scene_loader_matrix_sensor");
    (void)write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_sensor_matrix.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="path">
    <integer name="maxDepth" value="-1"/>
  </integrator>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <transform name="toWorld">
      <matrix value="1,0,0,5, 0,1,0,6, 0,0,1,7, 0,0,0,1"/>
    </transform>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    auto scene = pbpt::loader::load_scene<double>(xml_path.string());
    const auto camera_to_world = scene.render_transform.camera_to_world().matrix();
    EXPECT_NEAR(camera_to_world.at(0, 3), 5.0, 1e-9);
    EXPECT_NEAR(camera_to_world.at(1, 3), 6.0, 1e-9);
    EXPECT_NEAR(camera_to_world.at(2, 3), 7.0, 1e-9);
}

TEST(SceneLoaderMatrixTest, ThrowsOnInvalidMatrixValue) {
    TempDir temp_dir("pbpt_scene_loader_matrix_invalid");
    (void)write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_invalid_matrix.xml";
    write_text_file(
        xml_path,
        R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <transform name="toWorld">
      <matrix value="1,0,0,1"/>
    </transform>
  </sensor>
  <bsdf type="diffuse" id="mat_white">
    <spectrum name="reflectance" value="400:0.7, 500:0.7, 600:0.7, 700:0.7"/>
  </bsdf>
  <shape type="obj">
    <string name="filename" value="tri.obj"/>
    <ref id="mat_white"/>
  </shape>
</scene>)XML"
    );

    EXPECT_THROW(
        (void)pbpt::loader::load_scene<double>(xml_path.string()),
        std::runtime_error
    );
}
