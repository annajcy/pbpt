#pragma once

#include "scene.hpp"

#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/light/plugin/light/area_light.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"

namespace pbpt::scene {

template <typename T>
inline Scene<T> create_cbox_scene(
    const std::string& scene_path = "/Users/jinceyang/Desktop/codebase/graphics/pbpt/asset/scene/cbox") {
    Scene<T> scene;

    // 1. Setup Camera System
    scene.camera =
        camera::ThinLensPerspectiveCamera<T>(math::Vector<int, 2>(512, 512), 39.307, "smaller", -10, -2800, T(1000.0));

    scene.render_transform =
        camera::RenderTransform<T>::look_at(math::Point<T, 3>(278, 273, -800), math::Point<T, 3>(278, 273, -799),
                                            math::Vector<T, 3>(0, 1, 0), camera::RenderSpace::World);

    scene.pixel_filter = camera::GaussianFilter<T>(T(1.5), T(0.5));

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
            return camera::HDRFilm<T, decltype(pixel_sensor)>(cam, pixel_sensor);
        },
        scene.camera);

    // 2. Load Meshes & Spectra
    scene.resources.mesh_library.add_item(
        "cbox_floor", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                 geometry::Transform<double>::identity()),
                                             scene_path + "/meshes/cbox_floor.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_ceiling", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                   geometry::Transform<double>::identity()),
                                               scene_path + "/meshes/cbox_ceiling.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_back", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                geometry::Transform<double>::identity()),
                                            scene_path + "/meshes/cbox_back.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_greenwall", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                     geometry::Transform<double>::identity()),
                                                 scene_path + "/meshes/cbox_greenwall.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_redwall", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                   geometry::Transform<double>::identity()),
                                               scene_path + "/meshes/cbox_redwall.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_smallbox", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                    geometry::Transform<double>::identity()),
                                                scene_path + "/meshes/cbox_smallbox.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_largebox", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                    geometry::Transform<double>::identity()),
                                                scene_path + "/meshes/cbox_largebox.obj", false));
    scene.resources.mesh_library.add_item(
        "cbox_luminaire", shape::TriangleMesh<T>(scene.render_transform.object_to_render_from_object_to_world(
                                                     geometry::Transform<double>::translate(Vector<T, 3>(0, -0.5, 0))),
                                                 scene_path + "/meshes/cbox_luminaire.obj", false));

    // 3. Setup Spectra
    scene.resources.reflectance_spectrum_library.add_item(
        "box", radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(
                   "400:0.343, 404:0.445, 408:0.551, 412:0.624, 416:0.665, 420:0.687, 424:0.708, 428:0.723, 432:0.715, "
                   "436:0.71, 440:0.745, 444:0.758, 448:0.739, 452:0.767, 456:0.777, 460:0.765, 464:0.751, 468:0.745, "
                   "472:0.748, 476:0.729, 480:0.745, 484:0.757, 488:0.753, 492:0.75, 496:0.746, 500:0.747, 504:0.735, "
                   "508:0.732, 512:0.739, 516:0.734, 520:0.725, 524:0.721, 528:0.733, 532:0.725, 536:0.732, 540:0.743, "
                   "544:0.744, 548:0.748, 552:0.728, 556:0.716, 560:0.733, 564:0.726, 568:0.713, 572:0.74, 576:0.754, "
                   "580:0.764, 584:0.752, 588:0.736, 592:0.734, 596:0.741, 600:0.74, 604:0.732, 608:0.745, 612:0.755, "
                   "616:0.751, 620:0.744, 624:0.731, 628:0.733, 632:0.744, 636:0.731, 640:0.712, 644:0.708, 648:0.729, "
                   "652:0.73, 656:0.727, 660:0.707, 664:0.703, 668:0.729, 672:0.75, 676:0.76, 680:0.751, 684:0.739, "
                   "688:0.724, 692:0.73, 696:0.74, 700:0.737"));
    scene.resources.reflectance_spectrum_library.add_item(
        "white",
        radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(
            "400:0.343, 404:0.445, 408:0.551, 412:0.624, 416:0.665, 420:0.687, 424:0.708, 428:0.723, 432:0.715, "
            "436:0.71, 440:0.745, 444:0.758, 448:0.739, 452:0.767, 456:0.777, 460:0.765, 464:0.751, 468:0.745, "
            "472:0.748, 476:0.729, 480:0.745, 484:0.757, 488:0.753, 492:0.75, 496:0.746, 500:0.747, 504:0.735, "
            "508:0.732, 512:0.739, 516:0.734, 520:0.725, 524:0.721, 528:0.733, 532:0.725, 536:0.732, 540:0.743, "
            "544:0.744, 548:0.748, 552:0.728, 556:0.716, 560:0.733, 564:0.726, 568:0.713, 572:0.74, 576:0.754, "
            "580:0.764, 584:0.752, 588:0.736, 592:0.734, 596:0.741, 600:0.74, 604:0.732, 608:0.745, 612:0.755, "
            "616:0.751, 620:0.744, 624:0.731, 628:0.733, 632:0.744, 636:0.731, 640:0.712, 644:0.708, 648:0.729, "
            "652:0.73, 656:0.727, 660:0.707, 664:0.703, 668:0.729, 672:0.75, 676:0.76, 680:0.751, 684:0.739, "
            "688:0.724, 692:0.73, 696:0.74, 700:0.737"));
    scene.resources.reflectance_spectrum_library.add_item(
        "red", radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(
                   "400:0.04, 404:0.046, 408:0.048, 412:0.053, 416:0.049, 420:0.05, 424:0.053, 428:0.055, 432:0.057, "
                   "436:0.056, 440:0.059, 444:0.057, 448:0.061, 452:0.061, 456:0.06, 460:0.062, 464:0.062, 468:0.062, "
                   "472:0.061, 476:0.062, 480:0.06, 484:0.059, 488:0.057, 492:0.058, 496:0.058, 500:0.058, 504:0.056, "
                   "508:0.055, 512:0.056, 516:0.059, 520:0.057, 524:0.055, 528:0.059, 532:0.059, 536:0.058, 540:0.059, "
                   "544:0.061, 548:0.061, 552:0.063, 556:0.063, 560:0.067, 564:0.068, 568:0.072, 572:0.08, 576:0.09, "
                   "580:0.099, 584:0.124, 588:0.154, 592:0.192, 596:0.255, 600:0.287, 604:0.349, 608:0.402, 612:0.443, "
                   "616:0.487, 620:0.513, 624:0.558, 628:0.584, 632:0.62, 636:0.606, 640:0.609, 644:0.651, 648:0.612, "
                   "652:0.61, 656:0.65, 660:0.638, 664:0.627, 668:0.62, 672:0.63, 676:0.628, 680:0.642, 684:0.639, "
                   "688:0.657, 692:0.639, 696:0.635, 700:0.642"));
    scene.resources.reflectance_spectrum_library.add_item(
        "green",
        radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string(
            "400:0.092, 404:0.096, 408:0.098, 412:0.097, 416:0.098, 420:0.095, 424:0.095, 428:0.097, 432:0.095, "
            "436:0.094, 440:0.097, 444:0.098, 448:0.096, 452:0.101, 456:0.103, 460:0.104, 464:0.107, 468:0.109, "
            "472:0.112, 476:0.115, 480:0.125, 484:0.14, 488:0.16, 492:0.187, 496:0.229, 500:0.285, 504:0.343, "
            "508:0.39, 512:0.435, 516:0.464, 520:0.472, 524:0.476, 528:0.481, 532:0.462, 536:0.447, 540:0.441, "
            "544:0.426, 548:0.406, 552:0.373, 556:0.347, 560:0.337, 564:0.314, 568:0.285, 572:0.277, 576:0.266, "
            "580:0.25, 584:0.23, 588:0.207, 592:0.186, 596:0.171, 600:0.16, 604:0.148, 608:0.141, 612:0.136, 616:0.13, "
            "620:0.126, 624:0.123, 628:0.121, 632:0.122, 636:0.119, 640:0.114, 644:0.115, 648:0.117, 652:0.117, "
            "656:0.118, 660:0.12, 664:0.122, 668:0.128, 672:0.132, 676:0.139, 680:0.144, 684:0.146, 688:0.15, "
            "692:0.152, 696:0.157, 700:0.159"));
    scene.resources.reflectance_spectrum_library.add_item(
        "light",
        radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.78, 500:0.78, 600:0.78, 700:0.78"));
    scene.resources.reflectance_spectrum_library.add_item(
        "emitter", radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0, 500:8, 600:15.6, 700:18.4"));

    // 4. Setup Materials
    // White/Box material (floor, ceiling, back wall, boxes)
    int white_id = scene.resources.any_material_library.add_item(
        "white", material::LambertianMaterial<T>(scene.resources.reflectance_spectrum_library.get("white")));
    // Red material (right wall)
    int red_id = scene.resources.any_material_library.add_item(
        "red", material::LambertianMaterial<T>(scene.resources.reflectance_spectrum_library.get("red")));
    // Green material (left wall)
    int green_id = scene.resources.any_material_library.add_item(
        "green", material::LambertianMaterial<T>(scene.resources.reflectance_spectrum_library.get("green")));
    // light source material
    int light_id = scene.resources.any_material_library.add_item(
        "light", material::LambertianMaterial<T>(scene.resources.reflectance_spectrum_library.get("light")));

    // Map meshes to materials
    scene.resources.mesh_material_map["cbox_floor"] = white_id;
    scene.resources.mesh_material_map["cbox_ceiling"] = white_id;
    scene.resources.mesh_material_map["cbox_back"] = white_id;
    scene.resources.mesh_material_map["cbox_smallbox"] = white_id;
    scene.resources.mesh_material_map["cbox_largebox"] = white_id;
    scene.resources.mesh_material_map["cbox_redwall"] = red_id;
    scene.resources.mesh_material_map["cbox_greenwall"] = green_id;
    scene.resources.mesh_material_map["cbox_luminaire"] = light_id;

    // 5. Setup Lights
    auto light_spectrum = radiometry::StandardEmissionSpectrum<T>(
        scene.resources.reflectance_spectrum_library.get("emitter"), radiometry::constant::CIE_D65_ilum<T>);

    auto light_mesh = scene.resources.mesh_library.get("cbox_luminaire");  // mesh_library returns reference
    for (int i = 0; i < light_mesh.triangle_count(); ++i) {
        int new_light_id = scene.resources.any_light_library.add_item(
            std::format("cbox_luminaire_{}", i),
            light::AreaLight<T, shape::Triangle<T>, decltype(light_spectrum)>(
                shape::Triangle<T>(light_mesh, i), light_spectrum, light::AreaLightSamplingDomain::Shape));
        scene.resources.mesh_light_map[make_mesh_triangle_key("cbox_luminaire", i)] = new_light_id;
    }

    // 6. Setup Aggregate
    std::vector<shape::Primitive<T>> primitives;
    for (const auto& [mesh_name, mesh_id] : scene.resources.mesh_library.name_to_id()) {
        const auto& mesh = scene.resources.mesh_library.get(mesh_id);
        int material_id = scene.resources.mesh_material_map.at(mesh_name);
        for (int i = 0; i < mesh.triangle_count(); ++i) {
            int light_id = -1;
            // Note: mesh_name is the key in mesh_library (e.g. "cbox_floor")
            // light_map keys are MeshTriangleKey{mesh_name, triangle_index}
            auto key = make_mesh_triangle_key(mesh_name, i);
            if (scene.resources.mesh_light_map.contains(key)) {
                light_id = scene.resources.mesh_light_map.at(key);
            }
            primitives.push_back(shape::Primitive<T>(shape::Triangle<T>(mesh, i), material_id, light_id));
        }
    }

    scene.aggregate = aggregate::EmbreeAggregate<T>(std::move(primitives));
    return scene;
}

}  // namespace pbpt::scene
