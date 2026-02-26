#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "pbpt/serde/scene_loader.hpp"

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
<scene version="3.0.0">
  <integrator type="simple_path"/>
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

    EXPECT_THROW((void)pbpt::serde::load_scene<double>(xml_path.string()), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsOnUnknownShapeMaterialReference) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_unknown_ref");
    write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_unknown_material_ref.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
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

    EXPECT_THROW((void)pbpt::serde::load_scene<double>(xml_path.string()), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsWhenMeshHasNoMaterialAssignment) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_no_material_assignment");
    write_basic_obj(temp_dir);

    const auto xml_path = temp_dir.path / "scene_no_material_assignment.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
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

    EXPECT_THROW((void)pbpt::serde::load_scene<double>(xml_path.string()), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsOnUnsupportedType) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_unsupported_type");

    const auto xml_path = temp_dir.path / "scene_unsupported_type.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <texture type="noise" id="tex_noise">
  </texture>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        EXPECT_NE(std::string(err.what()).find("noise"), std::string::npos);
        EXPECT_NE(std::string(err.what()).find("tex_noise"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnTextureMissingId) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_texture_missing_id");

    const auto xml_path = temp_dir.path / "scene_texture_missing_id.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
  <texture type="bitmap">
    <string name="filename" value="missing.exr"/>
  </texture>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("id"), std::string::npos);
        EXPECT_NE(message.find("bitmap"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnInvalidSamplerSampleCountZero) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_sampler_zero");

    const auto xml_path = temp_dir.path / "scene_sampler_zero.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <sampler type="independent">
      <integer name="sample_count" value="0"/>
    </sampler>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("sample_count"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnInvalidSamplerSampleCountNegative) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_sampler_negative");

    const auto xml_path = temp_dir.path / "scene_sampler_negative.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <sampler type="independent">
      <integer name="sample_count" value="-1"/>
    </sampler>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("sample_count"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsWhenShapeInstanceReferencesMissingMesh) {
    pbpt::scene::RenderResources<double> resources{};
    pbpt::scene::ShapeInstanceRecord<double> bad_record{};
    bad_record.shape_id = "shape_missing_mesh";
    bad_record.shape_type = "obj";
    bad_record.mesh_name = "mesh_missing";
    bad_record.material_ref_name = "mat_dummy";
    resources.shape_instances.push_back(std::move(bad_record));

    EXPECT_THROW((void)pbpt::serde::build_primitives_from_resources<double>(resources), std::runtime_error);
}

TEST(SceneLoaderFailFastTest, ThrowsOnLegacySceneVersion) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_legacy_version");
    const auto xml_path = temp_dir.path / "scene_legacy_version.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="0.4.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("version"), std::string::npos);
        EXPECT_NE(message.find("3.0.0"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnLegacyFieldNames) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_legacy_fields");
    const auto xml_path = temp_dir.path / "scene_legacy_fields.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path">
    <integer name="max_depth" value="2"/>
  </integrator>
  <sensor type="perspective">
    <string name="fovAxis" value="smaller"/>
    <transform name="to_world">
      <lookat origin="0, 0, 5" target="0, 0, 0" up="0, 1, 0"/>
    </transform>
    <float name="fov" value="45"/>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("fovAxis"), std::string::npos);
        EXPECT_NE(message.find("fov_axis"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnLegacyLookAtTag) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_legacy_lookat");
    const auto xml_path = temp_dir.path / "scene_legacy_lookat_tag.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <transform name="to_world">
      <lookAt origin="0, 0, 5" target="0, 0, 0" up="0, 1, 0"/>
    </transform>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("lookAt"), std::string::npos);
        EXPECT_NE(message.find("lookat"), std::string::npos);
    }
}

TEST(SceneLoaderFailFastTest, ThrowsOnLegacyPluginNames) {
    TempDir temp_dir("pbpt_scene_loader_fail_fast_legacy_plugin");
    const auto xml_path = temp_dir.path / "scene_legacy_plugin.xml";
    write_text_file(xml_path,
                    R"XML(<?xml version="1.0" encoding="utf-8"?>
<scene version="3.0.0">
  <integrator type="simple_path"/>
  <sensor type="perspective">
    <float name="fov" value="45"/>
    <sampler type="ldsampler">
      <integer name="sample_count" value="1"/>
    </sampler>
  </sensor>
</scene>)XML");

    try {
        pbpt::serde::load_scene<double>(xml_path.string());
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("ldsampler"), std::string::npos);
        EXPECT_NE(message.find("independent"), std::string::npos);
    }
}
