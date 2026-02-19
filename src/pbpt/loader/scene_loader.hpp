#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"
#include "pbpt/loader/basic_parser.hpp"
#include "pbpt/loader/transform_loader.hpp"
#include "pbpt/loader/loader_context.hpp"

// Component headers
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/material/plugin/material/dielectric_material.hpp"
#include "pbpt/material/plugin/material/dielectric_specular_material.hpp"
#include "pbpt/material/plugin/material/dielectric_rough_material.hpp"
#include "pbpt/material/plugin/material/conductor_material.hpp"
#include "pbpt/material/plugin/material/conductor_specular_material.hpp"
#include "pbpt/material/plugin/material/conductor_rough_material.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"  // Contains Triangle and TriangleMesh
#include "pbpt/light/plugin/light/area_light.hpp"
#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/radiometry/constant/lambda.hpp"
#include "pbpt/radiometry/constant/xyz_spectrum.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/texture/plugin/texture/bitmap_texture.hpp"

namespace pbpt::loader {

inline bool ends_with(const std::string& str, const std::string& suffix) {
    if (str.size() < suffix.size())
        return false;
    return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

inline bool starts_with(const std::string& str, const std::string& prefix) {
    if (str.size() < prefix.size())
        return false;
    return std::equal(prefix.begin(), prefix.end(), str.begin());
}

inline std::optional<std::string> find_child_value(const pugi::xml_node& node, const std::string& tag,
                                                   const std::string& name) {
    for (auto child : node.children(tag.c_str())) {
        if (std::string(child.attribute("name").value()) == name) {
            const char* value = child.attribute("value").value();
            if (value && value[0] != '\0') {
                return std::string(value);
            }
        }
    }
    return std::nullopt;
}

inline std::optional<std::string> find_child_reference_id(const pugi::xml_node& node, const std::string& name) {
    for (auto child : node.children("ref")) {
        if (std::string(child.attribute("name").value()) == name) {
            const char* id = child.attribute("id").value();
            if (id && id[0] != '\0') {
                return std::string(id);
            }
        }
    }
    return std::nullopt;
}

template <typename T>
inline std::optional<T> find_float_property(const pugi::xml_node& node, const std::string& name) {
    for (auto child : node.children("float")) {
        if (std::string(child.attribute("name").value()) == name) {
            return parse_value<T>(child.attribute("value").value());
        }
    }
    return std::nullopt;
}

template <typename T>
inline T roughness_to_alpha(T roughness) {
    return std::sqrt(std::max(T(0), roughness));
}

template <typename T>
inline material::MicrofacetModel<T> parse_microfacet_model(const pugi::xml_node& node) {
    T alpha_x = T(0.1);
    T alpha_y = T(0.1);
    bool alpha_found = false;

    if (auto alpha = find_float_property<T>(node, "alpha")) {
        alpha_x = alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_x")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_y")) {
        alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alphaU")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alphaV")) {
        alpha_y = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_u")) {
        alpha_x = *alpha;
        alpha_found = true;
    }
    if (auto alpha = find_float_property<T>(node, "alpha_v")) {
        alpha_y = *alpha;
        alpha_found = true;
    }

    if (!alpha_found) {
        if (auto rough = find_float_property<T>(node, "roughness")) {
            alpha_x = alpha_y = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughness_x")) {
            alpha_x = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughness_y")) {
            alpha_y = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughnessU")) {
            alpha_x = roughness_to_alpha<T>(*rough);
        }
        if (auto rough = find_float_property<T>(node, "roughnessV")) {
            alpha_y = roughness_to_alpha<T>(*rough);
        }
    }

    return material::MicrofacetModel<T>(alpha_x, alpha_y);
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> constant_spectrum(T value) {
    return radiometry::PiecewiseLinearSpectrumDistribution<T>(
        {{radiometry::constant::lambda_min<T>, value}, {radiometry::constant::lambda_max<T>, value}});
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> load_piecewise_spectrum_from_csv(
    const std::string& abs_path) {
    std::ifstream fin(abs_path);
    if (!fin) {
        throw std::runtime_error("Cannot open spectrum CSV: " + abs_path);
    }

    std::vector<std::pair<T, T>> points;
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        T lambda = T(0);
        T value = T(0);
        if (!(iss >> lambda >> value))
            continue;
        points.emplace_back(lambda, value);
    }
    if (points.empty()) {
        throw std::runtime_error("Spectrum CSV contains no data: " + abs_path);
    }
    return radiometry::PiecewiseLinearSpectrumDistribution<T>(points);
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> parse_piecewise_spectrum_value(const std::string& value,
                                                                                         const LoaderContext<T>& ctx) {
    if (starts_with(value, "file:")) {
        auto rel_path = value.substr(5);
        return load_piecewise_spectrum_from_csv<T>(ctx.resolve_path(rel_path));
    }
    if (ends_with(value, ".csv")) {
        return load_piecewise_spectrum_from_csv<T>(ctx.resolve_path(value));
    }
    return radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(value);
}

template <typename T>
inline radiometry::RGB<T> parse_rgb_triplet(const std::string& value) {
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');

    std::istringstream iss(normalized);
    T r = T(0);
    T g = T(0);
    T b = T(0);
    if (!(iss >> r)) {
        throw std::runtime_error("Invalid rgb reflectance value: " + value);
    }
    if (!(iss >> g))
        g = r;
    if (!(iss >> b))
        b = r;
    return radiometry::RGB<T>(r, g, b);
}

inline texture::WrapMode parse_wrap_mode(const std::string& value) {
    std::string mode = value;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "clamp") {
        return texture::WrapMode::Clamp;
    }
    return texture::WrapMode::Repeat;
}

template <typename T>
inline radiometry::PiecewiseLinearSpectrumDistribution<T> srgb_rgb_to_piecewise(const radiometry::RGB<T>& rgb) {
    const auto rsp = radiometry::lookup_srgb_to_rsp(rgb);
    const auto albedo_spectrum =
        radiometry::RGBAlbedoSpectrumDistribution<T, radiometry::RGBSigmoidPolynomialNormalized>(rsp);

    std::vector<std::pair<T, T>> points{};
    points.reserve(830 - 360 + 1);
    for (int lambda = 360; lambda <= 830; ++lambda) {
        points.emplace_back(static_cast<T>(lambda), albedo_spectrum.at(static_cast<T>(lambda)));
    }

    return radiometry::PiecewiseLinearSpectrumDistribution<T>(points);
}

template <typename T>
void parse_texture_node(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    const std::string type = node.attribute("type").value();
    const std::string id = node.attribute("id").value();
    if (id.empty()) {
        return;
    }

    if (type == "bitmap") {
        auto filename = find_child_value(node, "string", "filename");
        if (!filename) {
            throw std::runtime_error("bitmap texture missing filename: " + id);
        }

        auto wrap_u_str = find_child_value(node, "string", "wrapModeU").value_or("repeat");
        auto wrap_v_str = find_child_value(node, "string", "wrapModeV").value_or("repeat");
        auto wrap_u = parse_wrap_mode(wrap_u_str);
        auto wrap_v = parse_wrap_mode(wrap_v_str);

        // use RSPSpectrumTexture for bitmap textures to allow arbitrary RGB to spectrum conversion via the lookup table
        // auto tex = texture::RSPSpectrumTexture<T>(ctx.resolve_path(*filename), wrap_u, wrap_v);
        // ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));

        // use regular RGB texture and convert to spectrum at evaluation time, which is more flexible and allows for procedural RGB textures in the future
        auto tex = texture::BitmapTexture<T>(ctx.resolve_path(*filename), wrap_u, wrap_v);
        ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));
    } else if (type == "checkerboard") {
        auto color0 = find_child_value(node, "rgb", "color0").value_or("0");
        auto color1 = find_child_value(node, "rgb", "color1").value_or("1");
        auto uscale = find_float_property<T>(node, "uscale").value_or(T(1));
        auto vscale = find_float_property<T>(node, "vscale").value_or(T(1));
        auto uoffset = find_float_property<T>(node, "uoffset").value_or(T(0));
        auto voffset = find_float_property<T>(node, "voffset").value_or(T(0));

        auto tex = texture::CheckerboardTexture<T>(parse_rgb_triplet<T>(color0), parse_rgb_triplet<T>(color1), uscale,
                                                   vscale, uoffset, voffset);
        ctx.resources.reflectance_texture_library.add_item(id, std::move(tex));
    }
}

template <typename T>
void parse_bsdf(const pugi::xml_node& node, LoaderContext<T>& ctx) {
    std::string type = node.attribute("type").value();
    std::string id = node.attribute("id").value();

    if (type == "diffuse") {
        auto spectrum_value = find_child_value(node, "spectrum", "reflectance");
        auto rgb_value = find_child_value(node, "rgb", "reflectance");
        auto reflectance_ref = find_child_reference_id(node, "reflectance");

        if (reflectance_ref) {
            if (ctx.resources.reflectance_texture_library.name_to_id().contains(*reflectance_ref)) {
                auto mat =
                    material::LambertianMaterial<T>(ctx.resources.reflectance_texture_library.get(*reflectance_ref));
                ctx.resources.any_material_library.add_item(id, std::move(mat));
                return;
            }
            throw std::runtime_error("Unknown reflectance texture reference: " + *reflectance_ref);
        }

        std::string spec_name = id + "_reflectance";
        radiometry::PiecewiseLinearSpectrumDistribution<T> spectrum = constant_spectrum<T>(T(0.7));
        if (spectrum_value) {
            spectrum = parse_piecewise_spectrum_value<T>(*spectrum_value, ctx);
        } else if (rgb_value) {
            spectrum = srgb_rgb_to_piecewise<T>(parse_rgb_triplet<T>(*rgb_value));
        }
        ctx.resources.reflectance_spectrum_library.add_item(spec_name, std::move(spectrum));

        auto mat = material::LambertianMaterial<T>(ctx.resources.reflectance_spectrum_library.get(spec_name));
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "dielectric") {
        auto eta_opt = find_float_property<T>(node, "eta");
        if (!eta_opt) {
            eta_opt = find_float_property<T>(node, "intIOR");
        }
        T eta = eta_opt.value_or(T(1.5));
        auto microfacet_model = parse_microfacet_model<T>(node);
        auto mat = material::DielectricMaterial<T>(eta, microfacet_model);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "dielectric_specular") {
        auto eta_opt = find_float_property<T>(node, "eta");
        if (!eta_opt) {
            eta_opt = find_float_property<T>(node, "intIOR");
        }
        T eta = eta_opt.value_or(T(1.5));
        auto mat = material::DielectricSpecularMaterial<T>(eta);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "dielectric_rough") {
        auto eta_opt = find_float_property<T>(node, "eta");
        if (!eta_opt) {
            eta_opt = find_float_property<T>(node, "intIOR");
        }
        T eta = eta_opt.value_or(T(1.5));
        auto microfacet_model = parse_microfacet_model<T>(node);
        auto mat = material::DielectricRoughMaterial<T>(eta, microfacet_model);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "conductor") {
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
        auto mat = material::ConductorMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "conductor_specular") {
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

        auto mat = material::ConductorSpecularMaterial<T>(std::move(eta_dist), std::move(k_dist));
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    } else if (type == "conductor_rough") {
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
        auto mat = material::ConductorRoughMaterial<T>(std::move(eta_dist), std::move(k_dist), microfacet_model);
        ctx.resources.any_material_library.add_item(id, std::move(mat));
    }
}

template <typename T>
void parse_shape(const pugi::xml_node& node, LoaderContext<T>& ctx,
                 const camera::RenderTransform<T>& render_transform) {
    std::string type = node.attribute("type").value();
    if (type == "obj") {
        std::string filename = "";
        // Find filename property
        for (auto child : node.children("string")) {
            if (std::string(child.attribute("name").value()) == "filename") {
                filename = child.attribute("value").value();
            }
        }
        if (filename.empty())
            return;

        std::string abs_path = ctx.resolve_path(filename);

        auto transform_node = node.child("transform");
        geometry::Transform<T> obj_to_world = geometry::Transform<T>::identity();
        if (transform_node) {
            obj_to_world = load_transform<T>(transform_node);
        }

        auto to_render = render_transform.object_to_render_from_object_to_world(obj_to_world);

        // Generate name from filename (e.g. "meshes/cbox_floor.obj" -> "cbox_floor")
        std::filesystem::path mesh_path(filename);
        std::string name = mesh_path.stem().string();

        ctx.resources.mesh_library.add_item(name, shape::TriangleMesh<T>(to_render, abs_path, false));

        // Ref material
        auto ref_node = node.child("ref");
        if (ref_node) {
            std::string mat_id = ref_node.attribute("id").value();
            auto& lib = ctx.resources.any_material_library;
            if (mat_id.empty()) {
                throw std::runtime_error("shape mesh reference has empty material id: " + name);
            }
            if (!lib.name_to_id().contains(mat_id)) {
                throw std::runtime_error("Unknown material reference: " + mat_id + " for mesh " + name);
            }
            ctx.resources.mesh_material_map[name] = lib.name_to_id().at(mat_id);
        }

        // Emitter
        auto emitter_node = node.child("emitter");
        if (emitter_node) {
            std::string radiance_str;
            for (auto child : emitter_node.children("spectrum")) {
                if (std::string(child.attribute("name").value()) == "radiance") {
                    radiance_str = child.attribute("value").value();
                }
            }

            std::string spec_name = name + "_emission";
            ctx.resources.reflectance_spectrum_library.add_item(
                spec_name, radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(radiance_str));

            const auto& emission_spec = ctx.resources.reflectance_spectrum_library.get(spec_name);
            auto light_spectrum_dist =
                radiometry::StandardEmissionSpectrum<T>(emission_spec, radiometry::constant::CIE_D65_ilum<T>);

            const auto& mesh = ctx.resources.mesh_library.get(name);
            for (int i = 0; i < mesh.triangle_count(); ++i) {
                std::string light_name = std::format("{}_{}", name, i);
                auto al = light::AreaLight<T, shape::Triangle<T>, decltype(light_spectrum_dist)>(
                    shape::Triangle<T>(mesh, i), light_spectrum_dist, light::AreaLightSamplingDomain::Shape);
                int lid = ctx.resources.any_light_library.add_item(light_name, std::move(al));
                ctx.resources.mesh_light_map[scene::make_mesh_triangle_key(name, i)] =
                    lid;  // This logic needs to match Aggregate construction
            }
        }
    }
}

template <typename T>
scene::Scene<T> load_scene(const std::string& filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        throw std::runtime_error(std::string("XML Load Error: ") + result.description());
    }

    scene::Scene<T> scene;
    pugi::xml_node root = doc.child("scene");
    LoaderContext<T> ctx(scene.resources, std::filesystem::path(filename).parent_path());

    // 1. Textures
    for (auto node : root.children("texture")) {
        parse_texture_node(node, ctx);
    }

    // 2. BSDFs
    for (auto node : root.children("bsdf")) {
        parse_bsdf(node, ctx);
    }

    // 3. Sensor
    auto sensor_node = root.child("sensor");
    if (sensor_node) {
        // Transform
        auto transform_node = sensor_node.child("transform");
        if (transform_node) {
            scene.render_transform = load_render_transform<T>(transform_node);
        }

        float fov = parse_property<float>(sensor_node, "fov");
        float focus_dist = parse_property<float>(sensor_node, "focusDistance");
        float near_clip = -parse_property<float>(sensor_node, "nearClip", 0.1f);
        float far_clip = -parse_property<float>(sensor_node, "farClip", 10000.0f);

        int width = 512, height = 512;
        auto film_node = sensor_node.child("film");
        if (film_node) {
            width = parse_property<int>(film_node, "width", width);
            height = parse_property<int>(film_node, "height", height);
        }

        scene.camera = camera::ThinLensPerspectiveCamera<T>(math::Vector<int, 2>(width, height), fov, "smaller",
                                                            near_clip, far_clip, T(focus_dist));

        // Pixel Filter
        scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5));  // Hardcoded for now or parse

        // Film
        scene.film = std::visit(
            [&](auto& cam) {
                auto pixel_sensor = camera::PixelSensor<T, radiometry::constant::CIED65SpectrumType<T>,
                                                        radiometry::constant::CIED65SpectrumType<T>,
                                                        radiometry::constant::XYZSpectrumType<T>>(
                    radiometry::constant::CIE_D65_ilum<T>, radiometry::constant::CIE_D65_ilum<T>,
                    radiometry::constant::sRGB<T>,
                    radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                        radiometry::constant::CIE_X<T>, radiometry::constant::CIE_Y<T>, radiometry::constant::CIE_Z<T>),
                    T{1.0});
                // Construct HDRFilm with this sensor
                return camera::AnyFilm<T>(camera::HDRFilm<T, decltype(pixel_sensor)>(cam, pixel_sensor));
            },
            scene.camera);
    }

    // 4. Shapes
    for (auto node : root.children("shape")) {
        parse_shape(node, ctx, scene.render_transform);
    }

    // 5. Aggregate
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);
        if (!scene.resources.mesh_material_map.contains(mesh_name)) {
            throw std::runtime_error("Mesh has no material assignment: " + mesh_name);
        }

        int material_id = scene.resources.mesh_material_map.at(mesh_name);
        if (!scene.resources.any_material_library.id_to_name().contains(material_id)) {
            throw std::runtime_error("Mesh material id is invalid for mesh: " + mesh_name);
        }

        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            auto light_key = scene::make_mesh_triangle_key(mesh_name, i);
            if (scene.resources.mesh_light_map.contains(light_key)) {
                light_id = scene.resources.mesh_light_map.at(light_key);
                if (!scene.resources.any_light_library.id_to_name().contains(light_id)) {
                    throw std::runtime_error(
                        std::format("Mesh light id is invalid for key: {}_{}", mesh_name, i));
                }
            }
            primitives.push_back(shape::Primitive<T>(shape::Triangle<T>(mesh, i), material_id, light_id));
        }
    }
    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));

    return scene;
}

}  // namespace pbpt::loader
