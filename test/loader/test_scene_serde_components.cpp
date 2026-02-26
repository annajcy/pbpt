#include <filesystem>
#include <string>
#include <tuple>

#include <gtest/gtest.h>
#include <pugixml.hpp>

#include "pbpt/serde/value/impl/render_transform.hpp"
#include "pbpt/serde/domain/impl/camera.hpp"
#include "pbpt/serde/value/impl/transform.hpp"
#include "pbpt/serde/domain/impl/integrator.hpp"
#include "pbpt/serde/domain/impl/sampler.hpp"
#include "pbpt/serde/value/impl/microfacet_model.hpp"
#include "pbpt/serde/domain/impl/material.hpp"
#include "pbpt/serde/value/impl/piecewise_spectrum.hpp"
#include "pbpt/serde/value/impl/rgb.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/domain/typelist.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/serde/value/impl/wrap_mode.hpp"
#include "pbpt/serde/domain/impl/texture.hpp"
#include "pbpt/serde/domain/impl/shape.hpp"

namespace {

template <typename T>
void expect_matrix_equal(const pbpt::math::Matrix<T, 4, 4>& a, const pbpt::math::Matrix<T, 4, 4>& b, T eps = T(1e-9)) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(a.at(row, col), b.at(row, col), eps);
        }
    }
}

pbpt::serde::ValueCodecReadEnv<double> make_read_env(const std::filesystem::path& base_path,
                                                     const pbpt::scene::RenderResources<double>& resources) {
    return pbpt::serde::ValueCodecReadEnv<double>{resources, base_path};
}

pbpt::serde::ValueCodecWriteEnv<double> make_write_env(const pbpt::scene::RenderResources<double>& resources) {
    const auto cwd = std::filesystem::current_path();
    return pbpt::serde::ValueCodecWriteEnv<double>{resources, cwd, cwd / "meshes", cwd / "textures"};
}

}  // namespace

TEST(SceneSerdeComponents, TransformSerdeRoundTripMatrixText) {
    pugi::xml_document doc;
    auto transform = doc.append_child("transform");
    transform.append_child("matrix").append_attribute("value") = "1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1";

    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);
    const auto write_env = make_write_env(resources);

    const auto parsed =
        pbpt::serde::ValueCodec<double, pbpt::geometry::Transform<double>>::parse_node(transform, read_env);
    const auto text = pbpt::serde::ValueCodec<double, pbpt::geometry::Transform<double>>::write_text(parsed, write_env);

    pugi::xml_document doc2;
    auto transform2 = doc2.append_child("transform");
    transform2.append_child("matrix").append_attribute("value") = text.c_str();
    const auto reparsed =
        pbpt::serde::ValueCodec<double, pbpt::geometry::Transform<double>>::parse_node(transform2, read_env);

    expect_matrix_equal(parsed.matrix(), reparsed.matrix());
}

TEST(SceneSerdeComponents, TransformSerdeThrowsOnInvalidMatrix) {
    pugi::xml_document doc;
    auto transform = doc.append_child("transform");
    transform.append_child("matrix").append_attribute("value") = "1,0,0,1";

    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_invalid_transform = [&] {
        (void)pbpt::serde::ValueCodec<double, pbpt::geometry::Transform<double>>::parse_node(transform, read_env);
    };
    EXPECT_THROW(parse_invalid_transform(), std::runtime_error);
}

TEST(SceneSerdeComponents, RenderTransformSerdeThrowsOnMixedLookAtAndMatrix) {
    pugi::xml_document doc;
    auto transform = doc.append_child("transform");
    auto look_at = transform.append_child("lookat");
    look_at.append_attribute("origin") = "0,0,5";
    look_at.append_attribute("target") = "0,0,0";
    look_at.append_attribute("up") = "0,1,0";
    transform.append_child("matrix").append_attribute("value") = "1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1";

    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_invalid_render_transform = [&] {
        (void)pbpt::serde::ValueCodec<double, pbpt::camera::RenderTransform<double>>::parse_node(transform, read_env);
    };
    EXPECT_THROW(parse_invalid_render_transform(), std::runtime_error);
}

TEST(SceneSerdeComponents, SpectrumSerdeRoundTripInlineValue) {
    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);
    const auto write_env = make_write_env(resources);

    const auto parsed =
        pbpt::serde::ValueCodec<double, pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>>::parse_text(
            "400:0.2, 500:0.3, 600:0.4", read_env);
    const auto text =
        pbpt::serde::ValueCodec<double, pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>>::write_text(
            parsed, write_env);
    const auto reparsed = pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>::from_string(text);

    const auto& p0 = parsed.points();
    const auto& p1 = reparsed.points();
    ASSERT_EQ(p0.size(), p1.size());
    for (std::size_t i = 0; i < p0.size(); ++i) {
        EXPECT_NEAR(p0[i].first, p1[i].first, 1e-9);
        EXPECT_NEAR(p0[i].second, p1[i].second, 1e-9);
    }
}

TEST(SceneSerdeComponents, SpectrumSerdeThrowsOnMissingCsv) {
    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_missing_spectrum_csv = [&] {
        (void)
            pbpt::serde::ValueCodec<double, pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>>::parse_text(
                "file:this_file_should_not_exist.csv", read_env);
    };
    EXPECT_THROW(parse_missing_spectrum_csv(), std::runtime_error);
}

TEST(SceneSerdeComponents, RgbSerdeThrowsOnInvalidText) {
    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_invalid_rgb = [&] {
        (void)pbpt::serde::ValueCodec<double, pbpt::radiometry::RGB<double>>::parse_text("abc", read_env);
    };
    EXPECT_THROW(parse_invalid_rgb(), std::runtime_error);
}

TEST(SceneSerdeComponents, WrapModeSerdeThrowsOnInvalidText) {
    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_invalid_wrap = [&] {
        (void)pbpt::serde::ValueCodec<double, pbpt::texture::WrapMode>::parse_text("mirror", read_env);
    };
    EXPECT_THROW(parse_invalid_wrap(), std::runtime_error);
}

TEST(SceneSerdeComponents, MicrofacetSerdeThrowsOnMixedAlphaAndRoughness) {
    pugi::xml_document doc;
    auto bsdf = doc.append_child("bsdf");
    bsdf.append_child("float").append_attribute("name") = "alpha";
    bsdf.child("float").append_attribute("value") = "0.2";
    auto roughness = bsdf.append_child("float");
    roughness.append_attribute("name") = "roughness";
    roughness.append_attribute("value") = "0.4";

    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    auto parse_mixed_microfacet = [&] {
        (void)pbpt::serde::ValueCodec<double, pbpt::material::MicrofacetModel<double>>::parse_node(bsdf, read_env);
    };
    EXPECT_THROW(parse_mixed_microfacet(), std::runtime_error);
}

TEST(SceneSerdeComponents, MicrofacetSerdeDefaultsToAlphaPointOne) {
    pugi::xml_document doc;
    auto bsdf = doc.append_child("bsdf");

    pbpt::scene::RenderResources<double> resources{};
    const auto read_env = make_read_env(std::filesystem::current_path(), resources);

    const auto model =
        pbpt::serde::ValueCodec<double, pbpt::material::MicrofacetModel<double>>::parse_node(bsdf, read_env);
    EXPECT_NEAR(model.alpha_x(), 0.1, 1e-9);
    EXPECT_NEAR(model.alpha_y(), 0.1, 1e-9);
}

namespace {

template <typename T>
struct DummyDupSerdeA {
    static constexpr std::string_view xml_type = "dup";
};

template <typename T>
struct DummyDupSerdeB {
    static constexpr std::string_view xml_type = "dup";
};

using DuplicateXmlTypeList = std::tuple<DummyDupSerdeA<float>, DummyDupSerdeB<float>>;

struct MissingCodecType {};

static_assert(pbpt::serde::has_unique_xml_types<pbpt::serde::TextureSerdeList<float>>());
static_assert(!pbpt::serde::has_unique_xml_types<DuplicateXmlTypeList>());

static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::geometry::Transform<double>>);
static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::camera::RenderTransform<double>>);
static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>>);
static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::radiometry::RGB<double>>);
static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::texture::WrapMode>);
static_assert(pbpt::serde::ValueCodecConcept<double, pbpt::material::MicrofacetModel<double>>);
static_assert(!pbpt::serde::ValueCodecConcept<double, MissingCodecType>);

static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::BitmapTextureSerde<float>>);
static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::DiffuseMaterialSerde<float>>);
static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::ObjShapeSerde<float>>);
static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::PerspectiveCameraSerde<float>>);
static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::SimplePathIntegratorSerde<float>>);
static_assert(pbpt::serde::SerdeConcept<float, pbpt::serde::LdsSamplerSerde<float>>);

static_assert(pbpt::serde::ValueSerdeConcept<float, pbpt::serde::BitmapTextureSerde<float>>);
static_assert(pbpt::serde::ValueSerdeConcept<float, pbpt::serde::DiffuseMaterialSerde<float>>);
static_assert(pbpt::serde::ShapeSerdeConcept<float, pbpt::serde::ObjShapeSerde<float>>);
static_assert(pbpt::serde::CameraSerdeConcept<float, pbpt::serde::PerspectiveCameraSerde<float>>);
static_assert(pbpt::serde::IntegratorSerdeConcept<float, pbpt::serde::SimplePathIntegratorSerde<float>>);
static_assert(pbpt::serde::SamplerSerdeConcept<float, pbpt::serde::LdsSamplerSerde<float>>);

}  // namespace

TEST(SceneSerdeComponents, TypeListUniqueXmlTypeContract) {
    EXPECT_TRUE((pbpt::serde::has_unique_xml_types<pbpt::serde::TextureSerdeList<float>>()));
    EXPECT_FALSE((pbpt::serde::has_unique_xml_types<DuplicateXmlTypeList>()));
}
