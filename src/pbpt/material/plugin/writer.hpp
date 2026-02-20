#pragma once

#include <stdexcept>
#include <type_traits>

#include <pugixml.hpp>

#include "pbpt/loader/writer_context.hpp"

namespace pbpt::loader {

template <typename T>
void write_bsdf_nodes(pugi::xml_node& root, WriterContext<T>& ctx) {
    for (const auto& [material_name, material_id] : ctx.resources.any_material_library.name_to_id()) {
        const auto& any_material = ctx.resources.any_material_library.get(material_id);

        auto bsdf = root.append_child("bsdf");
        bsdf.append_attribute("id") = material_name.c_str();

        std::visit(
            [&](const auto& material_variant) {
                using MaterialT = std::decay_t<decltype(material_variant)>;
                if constexpr (std::is_same_v<MaterialT, material::LambertianMaterial<T>>) {
                    bsdf.append_attribute("type") = "diffuse";

                    const auto& source = material_variant.reflectance_source();
                    std::visit(
                        [&](const auto& reflectance) {
                            using SourceT = std::decay_t<decltype(reflectance)>;
                            if constexpr (std::is_same_v<SourceT, radiometry::PiecewiseLinearSpectrumDistribution<T>>) {
                                auto reflectance_node = bsdf.append_child("spectrum");
                                reflectance_node.append_attribute("name") = "reflectance";
                                const auto text = serialize_piecewise_spectrum(reflectance);
                                reflectance_node.append_attribute("value") = text.c_str();
                            } else {
                                int tex_id = material_variant.texture_id();
                                if (!ctx.resources.reflectance_texture_library.id_to_name().contains(tex_id)) {
                                    throw std::runtime_error("Lambertian texture id not found in library: " +
                                                             material_name);
                                }
                                const auto& tex_name =
                                    ctx.resources.reflectance_texture_library.id_to_name().at(tex_id);
                                auto reflectance_ref = bsdf.append_child("ref");
                                reflectance_ref.append_attribute("name") = "reflectance";
                                reflectance_ref.append_attribute("id") = tex_name.c_str();
                            }
                        },
                        source);
                } else if constexpr (std::is_same_v<MaterialT, material::DielectricMaterial<T>>) {
                    bsdf.append_attribute("type") = "dielectric";

                    auto eta = bsdf.append_child("float");
                    eta.append_attribute("name") = "eta";
                    eta.append_attribute("value") = material_variant.eta();

                    auto alpha_x = bsdf.append_child("float");
                    alpha_x.append_attribute("name") = "alpha_x";
                    alpha_x.append_attribute("value") = material_variant.microfacet_model().alpha_x();

                    auto alpha_y = bsdf.append_child("float");
                    alpha_y.append_attribute("name") = "alpha_y";
                    alpha_y.append_attribute("value") = material_variant.microfacet_model().alpha_y();
                } else if constexpr (std::is_same_v<MaterialT, material::DielectricSpecularMaterial<T>>) {
                    bsdf.append_attribute("type") = "dielectric_specular";

                    auto eta = bsdf.append_child("float");
                    eta.append_attribute("name") = "eta";
                    eta.append_attribute("value") = material_variant.eta();
                } else if constexpr (std::is_same_v<MaterialT, material::DielectricRoughMaterial<T>>) {
                    bsdf.append_attribute("type") = "dielectric_rough";

                    auto eta = bsdf.append_child("float");
                    eta.append_attribute("name") = "eta";
                    eta.append_attribute("value") = material_variant.eta();

                    auto alpha_x = bsdf.append_child("float");
                    alpha_x.append_attribute("name") = "alpha_x";
                    alpha_x.append_attribute("value") = material_variant.microfacet_model().alpha_x();

                    auto alpha_y = bsdf.append_child("float");
                    alpha_y.append_attribute("name") = "alpha_y";
                    alpha_y.append_attribute("value") = material_variant.microfacet_model().alpha_y();
                } else if constexpr (std::is_same_v<MaterialT, material::ConductorMaterial<T>>) {
                    bsdf.append_attribute("type") = "conductor";

                    auto eta = bsdf.append_child("spectrum");
                    eta.append_attribute("name") = "eta";
                    const auto eta_text = serialize_piecewise_spectrum(material_variant.eta_dist());
                    eta.append_attribute("value") = eta_text.c_str();

                    auto k = bsdf.append_child("spectrum");
                    k.append_attribute("name") = "k";
                    const auto k_text = serialize_piecewise_spectrum(material_variant.k_dist());
                    k.append_attribute("value") = k_text.c_str();

                    auto alpha_x = bsdf.append_child("float");
                    alpha_x.append_attribute("name") = "alpha_x";
                    alpha_x.append_attribute("value") = material_variant.microfacet_model().alpha_x();

                    auto alpha_y = bsdf.append_child("float");
                    alpha_y.append_attribute("name") = "alpha_y";
                    alpha_y.append_attribute("value") = material_variant.microfacet_model().alpha_y();
                } else if constexpr (std::is_same_v<MaterialT, material::ConductorSpecularMaterial<T>>) {
                    bsdf.append_attribute("type") = "conductor_specular";

                    auto eta = bsdf.append_child("spectrum");
                    eta.append_attribute("name") = "eta";
                    const auto eta_text = serialize_piecewise_spectrum(material_variant.eta_dist());
                    eta.append_attribute("value") = eta_text.c_str();

                    auto k = bsdf.append_child("spectrum");
                    k.append_attribute("name") = "k";
                    const auto k_text = serialize_piecewise_spectrum(material_variant.k_dist());
                    k.append_attribute("value") = k_text.c_str();
                } else if constexpr (std::is_same_v<MaterialT, material::ConductorRoughMaterial<T>>) {
                    bsdf.append_attribute("type") = "conductor_rough";

                    auto eta = bsdf.append_child("spectrum");
                    eta.append_attribute("name") = "eta";
                    const auto eta_text = serialize_piecewise_spectrum(material_variant.eta_dist());
                    eta.append_attribute("value") = eta_text.c_str();

                    auto k = bsdf.append_child("spectrum");
                    k.append_attribute("name") = "k";
                    const auto k_text = serialize_piecewise_spectrum(material_variant.k_dist());
                    k.append_attribute("value") = k_text.c_str();

                    auto alpha_x = bsdf.append_child("float");
                    alpha_x.append_attribute("name") = "alpha_x";
                    alpha_x.append_attribute("value") = material_variant.microfacet_model().alpha_x();

                    auto alpha_y = bsdf.append_child("float");
                    alpha_y.append_attribute("name") = "alpha_y";
                    alpha_y.append_attribute("value") = material_variant.microfacet_model().alpha_y();
                } else {
                    throw std::runtime_error("Unsupported material type for serialization: " + material_name);
                }
            },
            any_material);

        ctx.material_name_by_id[material_id] = material_name;
    }
}

}  // namespace pbpt::loader
