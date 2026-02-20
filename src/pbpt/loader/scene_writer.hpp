#pragma once

/**
 * @file scene_writer.hpp
 * @brief Scene serialization: write_scene() and supporting write_sensor_node().
 *
 * Include this header (or the convenience umbrella scene_loader.hpp) to
 * serialize a Scene<T> back to disk as a Mitsuba-compatible XML file.
 */

#include <filesystem>
#include <stdexcept>
#include <variant>

#include <pugixml.hpp>

#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/loader/writer_context.hpp"
#include "pbpt/texture/plugin/writer.hpp"
#include "pbpt/material/plugin/writer.hpp"
#include "pbpt/shape/plugin/writer.hpp"

namespace pbpt::loader {

namespace detail {

/**
 * @brief Write the `<sensor>` XML block for a ThinLensPerspectiveCamera.
 *
 * Reads FOV, clip planes, focal distance and film resolution directly from
 * the camera object embedded in Scene<T>.  The render-to-world transform is
 * taken from @p scene.render_transform.
 */
template <typename T>
void write_sensor_node(pugi::xml_node& root, const scene::Scene<T>& scene) {
    auto sensor = root.append_child("sensor");
    sensor.append_attribute("type") = "perspective";

    const auto* cam = std::get_if<camera::ThinLensPerspectiveCamera<T>>(&scene.camera);
    if (!cam) {
        throw std::runtime_error("write_scene currently only supports ThinLensPerspectiveCamera.");
    }

    // FOV
    auto fov_node = sensor.append_child("float");
    fov_node.append_attribute("name") = "fov";
    fov_node.append_attribute("value") = cam->fov_degrees();

    // FOV axis (enum → string)
    const auto fov_axis_str = camera::fov_axis_to_string(cam->fov_axis());
    auto fov_axis_node = sensor.append_child("string");
    fov_axis_node.append_attribute("name") = "fovAxis";
    fov_axis_node.append_attribute("value") = fov_axis_str.c_str();

    // Clip planes (stored negative internally)
    auto near_node = sensor.append_child("float");
    near_node.append_attribute("name") = "nearClip";
    near_node.append_attribute("value") = -cam->near_clip();

    auto far_node = sensor.append_child("float");
    far_node.append_attribute("name") = "farClip";
    far_node.append_attribute("value") = -cam->far_clip();

    // Focal distance
    auto fd_node = sensor.append_child("float");
    fd_node.append_attribute("name") = "focusDistance";
    fd_node.append_attribute("value") = cam->focal_distance();

    // Camera-to-world transform
    auto transform = sensor.append_child("transform");
    transform.append_attribute("name") = "toWorld";
    const auto mat_text = detail::serialize_transform_row_major(scene.render_transform.camera_to_world());
    transform.append_child("matrix").append_attribute("value") = mat_text.c_str();

    // Sampler (hardcoded — scene XML still carries it)
    auto sampler = sensor.append_child("sampler");
    sampler.append_attribute("type") = "ldsampler";
    auto scount = sampler.append_child("integer");
    scount.append_attribute("name") = "sampleCount";
    scount.append_attribute("value") = "4";

    // Film resolution from embedded film
    const auto res = cam->film_resolution();
    auto film = sensor.append_child("film");
    film.append_attribute("type") = "hdrfilm";
    auto w = film.append_child("integer");
    w.append_attribute("name") = "width";
    w.append_attribute("value") = res.x();
    auto h = film.append_child("integer");
    h.append_attribute("name") = "height";
    h.append_attribute("value") = res.y();
}

}  // namespace detail

/**
 * @brief Serialize @p scene to a Mitsuba-compatible XML file at @p filename.
 *
 * Supporting assets (meshes, textures) are written into sub-directories of
 * the XML's parent directory (`meshes/` and `textures/`).
 *
 * @param scene    The scene to serialize.
 * @param filename Absolute or relative path of the output `.xml` file.
 */
template <typename T>
void write_scene(const scene::Scene<T>& scene, const std::string& filename) {
    if (filename.empty()) {
        throw std::invalid_argument("write_scene: filename must not be empty.");
    }

    const auto xml_path = std::filesystem::path(filename);
    const auto scene_dir = xml_path.parent_path();
    const auto mesh_dir = scene_dir / "meshes";
    const auto texture_dir = scene_dir / "textures";

    std::filesystem::create_directories(mesh_dir);
    std::filesystem::create_directories(texture_dir);

    WriterContext<T> ctx{scene.resources, scene_dir, mesh_dir, texture_dir};

    pugi::xml_document doc;
    auto root = doc.append_child("scene");
    root.append_attribute("version") = "0.5.0";

    root.append_child("integrator").append_attribute("type") = "path";

    detail::write_sensor_node(root, scene);
    write_texture_nodes(root, ctx);
    write_bsdf_nodes(root, ctx);
    write_shape_nodes(root, ctx);

    std::filesystem::create_directories(scene_dir);
    if (!doc.save_file(xml_path.string().c_str(), "  ")) {
        throw std::runtime_error("write_scene: failed to save XML to: " + xml_path.string());
    }
}

}  // namespace pbpt::loader
