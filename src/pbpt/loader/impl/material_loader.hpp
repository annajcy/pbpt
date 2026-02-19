#pragma once

#include "pbpt/loader/component_loader.hpp"
#include "pbpt/loader/parser_utils.hpp"

#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/material/plugin/material/dielectric_material.hpp"
#include "pbpt/material/plugin/material/dielectric_specular_material.hpp"
#include "pbpt/material/plugin/material/dielectric_rough_material.hpp"
#include "pbpt/material/plugin/material/conductor_material.hpp"
#include "pbpt/material/plugin/material/conductor_specular_material.hpp"
#include "pbpt/material/plugin/material/conductor_rough_material.hpp"

namespace pbpt::loader {

template <typename T>
void register_material_loaders() {
    using Registry = MaterialLoaderRegistry<T>;

    Registry::register_loader(
        "diffuse", [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
            auto spectrum_value = find_child_value(node, "spectrum", "reflectance");
            auto rgb_value = find_child_value(node, "rgb", "reflectance");
            auto reflectance_ref = find_child_reference_id(node, "reflectance");

            if (reflectance_ref) {
                if (ctx.resources.reflectance_texture_library.name_to_id().contains(*reflectance_ref)) {
                    return material::LambertianMaterial<T>(
                        ctx.resources.reflectance_texture_library.get(*reflectance_ref));
                }
                throw std::runtime_error("Unknown reflectance texture reference: " + *reflectance_ref);
            }

            std::string id = node.attribute("id").value();
            std::string spec_name = id + "_reflectance";
            radiometry::PiecewiseLinearSpectrumDistribution<T> spectrum = constant_spectrum<T>(T(0.7));
            if (spectrum_value) {
                spectrum = parse_piecewise_spectrum_value<T>(*spectrum_value, ctx);
            } else if (rgb_value) {
                spectrum = srgb_rgb_to_piecewise<T>(parse_rgb_triplet<T>(*rgb_value));
            }
            ctx.resources.reflectance_spectrum_library.add_item(spec_name, std::move(spectrum));

            return material::LambertianMaterial<T>(ctx.resources.reflectance_spectrum_library.get(spec_name));
        });

    Registry::register_loader("dielectric",
                              [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
                                  auto eta_opt = find_float_property<T>(node, "eta");
                                  if (!eta_opt) {
                                      eta_opt = find_float_property<T>(node, "intIOR");
                                  }
                                  T eta = eta_opt.value_or(T(1.5));
                                  auto microfacet_model = parse_microfacet_model<T>(node);
                                  return material::DielectricMaterial<T>(eta, microfacet_model);
                              });

    Registry::register_loader("dielectric_specular",
                              [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
                                  auto eta_opt = find_float_property<T>(node, "eta");
                                  if (!eta_opt) {
                                      eta_opt = find_float_property<T>(node, "intIOR");
                                  }
                                  T eta = eta_opt.value_or(T(1.5));
                                  return material::DielectricSpecularMaterial<T>(eta);
                              });

    Registry::register_loader("dielectric_rough",
                              [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
                                  auto eta_opt = find_float_property<T>(node, "eta");
                                  if (!eta_opt) {
                                      eta_opt = find_float_property<T>(node, "intIOR");
                                  }
                                  T eta = eta_opt.value_or(T(1.5));
                                  auto microfacet_model = parse_microfacet_model<T>(node);
                                  return material::DielectricRoughMaterial<T>(eta, microfacet_model);
                              });

    Registry::register_loader(
        "conductor", [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
            auto eta_value = find_child_value(node, "spectrum", "eta");
            if (!eta_value) {
                eta_value = find_child_value(node, "string", "eta");
            }
            auto k_value = find_child_value(node, "spectrum", "k");
            if (!k_value) {
                k_value = find_child_value(node, "string", "k");
            }

            radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = constant_spectrum<T>(T(1));
            radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = constant_spectrum<T>(T(0));

            if (eta_value) {
                eta_dist = parse_piecewise_spectrum_value<T>(*eta_value, ctx);
            } else if (auto eta_scalar = find_float_property<T>(node, "eta")) {
                eta_dist = constant_spectrum<T>(*eta_scalar);
            }

            if (k_value) {
                k_dist = parse_piecewise_spectrum_value<T>(*k_value, ctx);
            } else if (auto k_scalar = find_float_property<T>(node, "k")) {
                k_dist = constant_spectrum<T>(*k_scalar);
            }

            auto microfacet_model = parse_microfacet_model<T>(node);
            return material::ConductorMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
        });

    Registry::register_loader(
        "conductor_specular", [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
            auto eta_value = find_child_value(node, "spectrum", "eta");
            if (!eta_value) {
                eta_value = find_child_value(node, "string", "eta");
            }
            auto k_value = find_child_value(node, "spectrum", "k");
            if (!k_value) {
                k_value = find_child_value(node, "string", "k");
            }

            radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = constant_spectrum<T>(T(1));
            radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = constant_spectrum<T>(T(0));

            if (eta_value) {
                eta_dist = parse_piecewise_spectrum_value<T>(*eta_value, ctx);
            } else if (auto eta_scalar = find_float_property<T>(node, "eta")) {
                eta_dist = constant_spectrum<T>(*eta_scalar);
            }

            if (k_value) {
                k_dist = parse_piecewise_spectrum_value<T>(*k_value, ctx);
            } else if (auto k_scalar = find_float_property<T>(node, "k")) {
                k_dist = constant_spectrum<T>(*k_scalar);
            }

            return material::ConductorSpecularMaterial<T>(std::move(eta_dist), std::move(k_dist));
        });

    Registry::register_loader(
        "conductor_rough", [](const pugi::xml_node& node, LoaderContext<T>& ctx) -> material::AnyMaterial<T> {
            auto eta_value = find_child_value(node, "spectrum", "eta");
            if (!eta_value) {
                eta_value = find_child_value(node, "string", "eta");
            }
            auto k_value = find_child_value(node, "spectrum", "k");
            if (!k_value) {
                k_value = find_child_value(node, "string", "k");
            }

            radiometry::PiecewiseLinearSpectrumDistribution<T> eta_dist = constant_spectrum<T>(T(1));
            radiometry::PiecewiseLinearSpectrumDistribution<T> k_dist = constant_spectrum<T>(T(0));

            if (eta_value) {
                eta_dist = parse_piecewise_spectrum_value<T>(*eta_value, ctx);
            } else if (auto eta_scalar = find_float_property<T>(node, "eta")) {
                eta_dist = constant_spectrum<T>(*eta_scalar);
            }

            if (k_value) {
                k_dist = parse_piecewise_spectrum_value<T>(*k_value, ctx);
            } else if (auto k_scalar = find_float_property<T>(node, "k")) {
                k_dist = constant_spectrum<T>(*k_scalar);
            }

            auto microfacet_model = parse_microfacet_model<T>(node);
            return material::ConductorRoughMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
        });
}

}  // namespace pbpt::loader
