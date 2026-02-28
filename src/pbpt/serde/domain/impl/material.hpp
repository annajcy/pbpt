#pragma once

#include <string>
#include <string_view>
#include <stdexcept>
#include <pugixml.hpp>

#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/serde/value/value_codec_dispatch.hpp"
#include "pbpt/serde/value/impl/piecewise_spectrum.hpp"
#include "pbpt/serde/value/impl/rgb.hpp"
#include "pbpt/serde/value/impl/microfacet_model.hpp"
#include "pbpt/radiometry/constant/metal_ior_spectrum.hpp"

#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/material/plugin/material/dielectric_material.hpp"
#include "pbpt/material/plugin/material/dielectric_specular_material.hpp"
#include "pbpt/material/plugin/material/dielectric_rough_material.hpp"
#include "pbpt/material/plugin/material/conductor_material.hpp"
#include "pbpt/material/plugin/material/conductor_specular_material.hpp"
#include "pbpt/material/plugin/material/conductor_rough_material.hpp"

namespace pbpt::serde {

template <typename T>
inline T parse_dielectric_eta_from_node(const pugi::xml_node& node, const ValueCodecReadEnv<T>& read_env) {
    if (const auto eta = parse_child_value<T, T>(node, "float", "eta", read_env)) {
        return *eta;
    }

    const T int_ior = parse_child_value<T, T>(node, "float", "int_ior", read_env).value_or(T(1.5));
    const T ext_ior = parse_child_value<T, T>(node, "float", "ext_ior", read_env).value_or(T(1.0));
    if (ext_ior <= T(0)) {
        throw std::runtime_error("invalid ext_ior for dielectric: must be > 0");
    }
    return int_ior / ext_ior;
}

template <typename T>
struct DiffuseMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "diffuse";
    using value_type = material::LambertianMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        auto spectrum_value = find_child_value(node, "spectrum", "reflectance");
        auto rgb_value = find_child_value(node, "rgb", "reflectance");
        auto reflectance_ref = parse_named_ref(node, "reflectance");

        if (reflectance_ref) {
            if (ctx.result.scene.resources.reflectance_texture_library.name_to_id().contains(*reflectance_ref)) {
                int tex_id = ctx.result.scene.resources.reflectance_texture_library.name_to_id().at(*reflectance_ref);
                return material::LambertianMaterial<T>(tex_id,
                                                       ctx.result.scene.resources.reflectance_texture_library.get(*reflectance_ref));
            }
            throw std::runtime_error("Unknown reflectance texture reference: " + *reflectance_ref);
        }

        std::string id = node.attribute("id").value();
        std::string spec_name = id + "_reflectance";
        radiometry::PiecewiseLinearSpectrumDistribution<T> spectrum = make_constant_piecewise_spectrum<T>(T(0.7));
        if (spectrum_value) {
            spectrum = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(
                *spectrum_value, read_env);
        } else if (rgb_value) {
            const auto rgb = ValueCodec<T, radiometry::RGB<T>>::parse_text(*rgb_value, read_env);
            spectrum = srgb_rgb_to_piecewise<T>(rgb);
        }
        ctx.result.scene.resources.reflectance_spectrum_library.add_item(spec_name, std::move(spectrum));

        return material::LambertianMaterial<T>(ctx.result.scene.resources.reflectance_spectrum_library.get(spec_name));
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir, ctx.config.write_microfacet_dist};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();

        const auto& source = mat.reflectance_source();
        std::visit(
            [&](const auto& reflectance) {
                using SourceT = std::decay_t<decltype(reflectance)>;
                if constexpr (std::is_same_v<SourceT, radiometry::PiecewiseLinearSpectrumDistribution<T>>) {
                    auto reflectance_node = node.append_child("spectrum");
                    reflectance_node.append_attribute("name") = "reflectance";
                    const auto text = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(
                        reflectance, write_env);
                    reflectance_node.append_attribute("value") = text.c_str();
                } else {
                    int tex_id = mat.texture_id();
                    if (!ctx.result.scene.resources.reflectance_texture_library.id_to_name().contains(tex_id)) {
                        throw std::runtime_error("Lambertian texture id not found in library: " + id);
                    }
                    const auto& tex_name = ctx.result.scene.resources.reflectance_texture_library.id_to_name().at(tex_id);
                    auto reflectance_ref = node.append_child("ref");
                    reflectance_ref.append_attribute("name") = "reflectance";
                    reflectance_ref.append_attribute("id") = tex_name.c_str();
                }
            },
            source);
    }
};

template <typename T>
struct DielectricMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "dielectric";
    using value_type = material::DielectricMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        T eta = parse_dielectric_eta_from_node<T>(node, read_env);
        auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
        return material::DielectricMaterial<T>(eta, microfacet_model);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto int_ior_node = node.append_child("float");
        int_ior_node.append_attribute("name") = "int_ior";
        int_ior_node.append_attribute("value") = mat.eta();
        auto ext_ior_node = node.append_child("float");
        ext_ior_node.append_attribute("name") = "ext_ior";
        ext_ior_node.append_attribute("value") = T(1);
    }
};

template <typename T>
struct DielectricSpecularMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "dielectric";
    using value_type = material::DielectricSpecularMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        T eta = parse_dielectric_eta_from_node<T>(node, read_env);
        return material::DielectricSpecularMaterial<T>(eta);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto int_ior_node = node.append_child("float");
        int_ior_node.append_attribute("name") = "int_ior";
        int_ior_node.append_attribute("value") = mat.eta();
        auto ext_ior_node = node.append_child("float");
        ext_ior_node.append_attribute("name") = "ext_ior";
        ext_ior_node.append_attribute("value") = T(1);
    }
};

template <typename T>
struct DielectricRoughMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "roughdielectric";
    using value_type = material::DielectricRoughMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        T eta = parse_dielectric_eta_from_node<T>(node, read_env);
        auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
        return material::DielectricRoughMaterial<T>(eta, microfacet_model);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{
            ctx.result.scene.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir, ctx.config.write_microfacet_dist};
        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto int_ior_node = node.append_child("float");
        int_ior_node.append_attribute("name") = "int_ior";
        int_ior_node.append_attribute("value") = mat.eta();
        auto ext_ior_node = node.append_child("float");
        ext_ior_node.append_attribute("name") = "ext_ior";
        ext_ior_node.append_attribute("value") = T(1);
        ValueCodec<T, material::MicrofacetModel<T>>::write_node(mat.microfacet_model(), node, write_env);
    }
};

template <typename T>
struct ConductorMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "conductor";
    using value_type = material::ConductorMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        if (const auto material_name = find_child_value(node, "string", "material")) {
            const auto eta_k = radiometry::constant::get_metal_eta_k_spectrum<T>(*material_name, ctx.base_path);
            auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
            return material::ConductorMaterial<T>(eta_k.eta, eta_k.k, microfacet_model);
        }

        auto eta_value = find_child_value(node, "spectrum", "eta");
        auto k_value = find_child_value(node, "spectrum", "k");

        radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = make_constant_piecewise_spectrum<T>(T(1));
        radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = make_constant_piecewise_spectrum<T>(T(0));

        if (eta_value)
            eta_dist = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*eta_value,
                                                                                                              read_env);
        else if (auto eta_scalar = parse_child_value<T, T>(node, "float", "eta", read_env))
            eta_dist = make_constant_piecewise_spectrum<T>(*eta_scalar);

        if (k_value)
            k_dist =
                ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*k_value, read_env);
        else if (auto k_scalar = parse_child_value<T, T>(node, "float", "k", read_env))
            k_dist = make_constant_piecewise_spectrum<T>(*k_scalar);

        auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
        return material::ConductorMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir, ctx.config.write_microfacet_dist};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto eta_node = node.append_child("spectrum");
        eta_node.append_attribute("name") = "eta";
        const auto eta_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.eta_dist(), write_env);
        eta_node.append_attribute("value") = eta_text.c_str();
        auto k_node = node.append_child("spectrum");
        k_node.append_attribute("name") = "k";
        const auto k_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.k_dist(), write_env);
        k_node.append_attribute("value") = k_text.c_str();
        ValueCodec<T, material::MicrofacetModel<T>>::write_node(mat.microfacet_model(), node, write_env);
    }
};

template <typename T>
struct ConductorSpecularMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "conductor";
    using value_type = material::ConductorSpecularMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        if (const auto material_name = find_child_value(node, "string", "material")) {
            const auto eta_k = radiometry::constant::get_metal_eta_k_spectrum<T>(*material_name, ctx.base_path);
            return material::ConductorSpecularMaterial<T>(eta_k.eta, eta_k.k);
        }

        auto eta_value = find_child_value(node, "spectrum", "eta");
        auto k_value = find_child_value(node, "spectrum", "k");

        radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = make_constant_piecewise_spectrum<T>(T(1));
        radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = make_constant_piecewise_spectrum<T>(T(0));

        if (eta_value)
            eta_dist = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*eta_value,
                                                                                                              read_env);
        else if (auto eta_scalar = parse_child_value<T, T>(node, "float", "eta", read_env))
            eta_dist = make_constant_piecewise_spectrum<T>(*eta_scalar);

        if (k_value)
            k_dist =
                ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*k_value, read_env);
        else if (auto k_scalar = parse_child_value<T, T>(node, "float", "k", read_env))
            k_dist = make_constant_piecewise_spectrum<T>(*k_scalar);

        return material::ConductorSpecularMaterial<T>(std::move(eta_dist), std::move(k_dist));
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir, ctx.config.write_microfacet_dist};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto eta_node = node.append_child("spectrum");
        eta_node.append_attribute("name") = "eta";
        const auto eta_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.eta_dist(), write_env);
        eta_node.append_attribute("value") = eta_text.c_str();
        auto k_node = node.append_child("spectrum");
        k_node.append_attribute("name") = "k";
        const auto k_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.k_dist(), write_env);
        k_node.append_attribute("value") = k_text.c_str();
    }
};

template <typename T>
struct ConductorRoughMaterialSerde {
    static constexpr std::string_view domain = "material";
    static constexpr std::string_view xml_type = "roughconductor";
    using value_type = material::ConductorRoughMaterial<T>;
    using load_result = value_type;
    using write_target = IdValueWriteTarget<value_type>;

    static value_type load(const pugi::xml_node& node, LoadContext<T>& ctx) {
        const ValueCodecReadEnv<T> read_env{ctx.result.scene.resources, ctx.base_path, ctx.config.load_microfacet_dist};

        if (const auto material_name = find_child_value(node, "string", "material")) {
            const auto eta_k = radiometry::constant::get_metal_eta_k_spectrum<T>(*material_name, ctx.base_path);
            auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
            return material::ConductorRoughMaterial<T>(eta_k.eta, eta_k.k, microfacet_model);
        }

        auto eta_value = find_child_value(node, "spectrum", "eta");
        auto k_value = find_child_value(node, "spectrum", "k");

        radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = make_constant_piecewise_spectrum<T>(T(1));
        radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = make_constant_piecewise_spectrum<T>(T(0));

        if (eta_value)
            eta_dist = ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*eta_value,
                                                                                                              read_env);
        else if (auto eta_scalar = parse_child_value<T, T>(node, "float", "eta", read_env))
            eta_dist = make_constant_piecewise_spectrum<T>(*eta_scalar);

        if (k_value)
            k_dist =
                ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::parse_text(*k_value, read_env);
        else if (auto k_scalar = parse_child_value<T, T>(node, "float", "k", read_env))
            k_dist = make_constant_piecewise_spectrum<T>(*k_scalar);

        auto microfacet_model = ValueCodec<T, material::MicrofacetModel<T>>::parse_node(node, read_env);
        return material::ConductorRoughMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
    }

    static void write(const write_target& target, pugi::xml_node& node, WriteContext<T>& ctx) {
        const auto& mat = target.value;
        const std::string id(target.id);
        const ValueCodecWriteEnv<T> write_env{ctx.result.scene.resources, ctx.scene_dir, ctx.mesh_dir, ctx.texture_dir, ctx.config.write_microfacet_dist};

        node.append_attribute("id") = id.c_str();
        node.append_attribute("type") = xml_type.data();
        auto eta_node = node.append_child("spectrum");
        eta_node.append_attribute("name") = "eta";
        const auto eta_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.eta_dist(), write_env);
        eta_node.append_attribute("value") = eta_text.c_str();
        auto k_node = node.append_child("spectrum");
        k_node.append_attribute("name") = "k";
        const auto k_text =
            ValueCodec<T, radiometry::PiecewiseLinearSpectrumDistribution<T>>::write_text(mat.k_dist(), write_env);
        k_node.append_attribute("value") = k_text.c_str();
        ValueCodec<T, material::MicrofacetModel<T>>::write_node(mat.microfacet_model(), node, write_env);
    }
};

static_assert(MaterialSerdeConcept<float, DiffuseMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, DielectricMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, DielectricSpecularMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, DielectricRoughMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, ConductorMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, ConductorSpecularMaterialSerde<float>>);
static_assert(MaterialSerdeConcept<float, ConductorRoughMaterialSerde<float>>);

}  // namespace pbpt::serde
