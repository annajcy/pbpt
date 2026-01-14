#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "aggregate/aggregate.hpp"
#include "camera/camera.hpp"
#include "camera/camera_system.hpp"
#include "camera/film.hpp"
#include "camera/pixel_filter.hpp"
#include "camera/pixel_sensor.hpp"
#include "camera/render_transform.hpp"
#include "light/area_light.hpp"
#include "math/vector.hpp"
#include "radiometry/constant/illuminant_spectrum.hpp"
#include "radiometry/constant/standard_color_spaces.hpp"
#include "radiometry/constant/xyz_spectrum.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"
#include "shape/primitive.hpp"
#include "shape/triangle.hpp"
#include "material/material.hpp"

namespace pbpt::scene {

class CornellBoxScene {
    using T = double;
    static constexpr int SpectrumSampleCount = 4;

private:
    int m_ssp = 4;
    int m_max_depth = -1;

    camera::CameraSystem<T, 
        camera::ThinLensPerspectiveCamera<T>, 
        camera::HDRFilm<T, 
            camera::PixelSensor<T, 
                radiometry::constant::CIED65SpectrumType<T>, 
                radiometry::constant::CIED65SpectrumType<T>, 
                radiometry::constant::XYZSpectrumType<T>
            >
        >, 
        camera::GaussianFilter<T>
    > m_camera_system;

    std::unordered_map<std::string, shape::TriangleMesh<T>> m_mesh_map;
    aggregate::LinearAggregate<T> m_aggregate;
    
    std::unordered_map<std::string, int> m_material_map;
    material::MaterialLibrary<T> m_material_library;

    std::unordered_map<std::string, radiometry::PiecewiseLinearSpectrumDistribution<T>> m_spectrum_map;

    std::vector<light::AreaLight<T, 
        shape::Triangle<T>, 
        radiometry::MultipliedSpectrumDistribution<T, 
            radiometry::PiecewiseLinearSpectrumDistribution<T>, 
            radiometry::constant::CIED65SpectrumType<T>>>
    > m_area_lights;

private:

    void make_spectrum() {
        m_spectrum_map.insert({
            "box",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.343, 404:0.445, 408:0.551, 412:0.624, 416:0.665, 420:0.687, 424:0.708, 428:0.723, 432:0.715, 436:0.71, 440:0.745, 444:0.758, 448:0.739, 452:0.767, 456:0.777, 460:0.765, 464:0.751, 468:0.745, 472:0.748, 476:0.729, 480:0.745, 484:0.757, 488:0.753, 492:0.75, 496:0.746, 500:0.747, 504:0.735, 508:0.732, 512:0.739, 516:0.734, 520:0.725, 524:0.721, 528:0.733, 532:0.725, 536:0.732, 540:0.743, 544:0.744, 548:0.748, 552:0.728, 556:0.716, 560:0.733, 564:0.726, 568:0.713, 572:0.74, 576:0.754, 580:0.764, 584:0.752, 588:0.736, 592:0.734, 596:0.741, 600:0.74, 604:0.732, 608:0.745, 612:0.755, 616:0.751, 620:0.744, 624:0.731, 628:0.733, 632:0.744, 636:0.731, 640:0.712, 644:0.708, 648:0.729, 652:0.73, 656:0.727, 660:0.707, 664:0.703, 668:0.729, 672:0.75, 676:0.76, 680:0.751, 684:0.739, 688:0.724, 692:0.73, 696:0.74, 700:0.737")
        });
        
        m_spectrum_map.insert({
            "white",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.343, 404:0.445, 408:0.551, 412:0.624, 416:0.665, 420:0.687, 424:0.708, 428:0.723, 432:0.715, 436:0.71, 440:0.745, 444:0.758, 448:0.739, 452:0.767, 456:0.777, 460:0.765, 464:0.751, 468:0.745, 472:0.748, 476:0.729, 480:0.745, 484:0.757, 488:0.753, 492:0.75, 496:0.746, 500:0.747, 504:0.735, 508:0.732, 512:0.739, 516:0.734, 520:0.725, 524:0.721, 528:0.733, 532:0.725, 536:0.732, 540:0.743, 544:0.744, 548:0.748, 552:0.728, 556:0.716, 560:0.733, 564:0.726, 568:0.713, 572:0.74, 576:0.754, 580:0.764, 584:0.752, 588:0.736, 592:0.734, 596:0.741, 600:0.74, 604:0.732, 608:0.745, 612:0.755, 616:0.751, 620:0.744, 624:0.731, 628:0.733, 632:0.744, 636:0.731, 640:0.712, 644:0.708, 648:0.729, 652:0.73, 656:0.727, 660:0.707, 664:0.703, 668:0.729, 672:0.75, 676:0.76, 680:0.751, 684:0.739, 688:0.724, 692:0.73, 696:0.74, 700:0.737")
        });
        
        m_spectrum_map.insert({
            "red",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.04, 404:0.046, 408:0.048, 412:0.053, 416:0.049, 420:0.05, 424:0.053, 428:0.055, 432:0.057, 436:0.056, 440:0.059, 444:0.057, 448:0.061, 452:0.061, 456:0.06, 460:0.062, 464:0.062, 468:0.062, 472:0.061, 476:0.062, 480:0.06, 484:0.059, 488:0.057, 492:0.058, 496:0.058, 500:0.058, 504:0.056, 508:0.055, 512:0.056, 516:0.059, 520:0.057, 524:0.055, 528:0.059, 532:0.059, 536:0.058, 540:0.059, 544:0.061, 548:0.061, 552:0.063, 556:0.063, 560:0.067, 564:0.068, 568:0.072, 572:0.08, 576:0.09, 580:0.099, 584:0.124, 588:0.154, 592:0.192, 596:0.255, 600:0.287, 604:0.349, 608:0.402, 612:0.443, 616:0.487, 620:0.513, 624:0.558, 628:0.584, 632:0.62, 636:0.606, 640:0.609, 644:0.651, 648:0.612, 652:0.61, 656:0.65, 660:0.638, 664:0.627, 668:0.62, 672:0.63, 676:0.628, 680:0.642, 684:0.639, 688:0.657, 692:0.639, 696:0.635, 700:0.642")
        });
        
        m_spectrum_map.insert({
            "green",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.092, 404:0.096, 408:0.098, 412:0.097, 416:0.098, 420:0.095, 424:0.095, 428:0.097, 432:0.095, 436:0.094, 440:0.097, 444:0.098, 448:0.096, 452:0.101, 456:0.103, 460:0.104, 464:0.107, 468:0.109, 472:0.112, 476:0.115, 480:0.125, 484:0.14, 488:0.16, 492:0.187, 496:0.229, 500:0.285, 504:0.343, 508:0.39, 512:0.435, 516:0.464, 520:0.472, 524:0.476, 528:0.481, 532:0.462, 536:0.447, 540:0.441, 544:0.426, 548:0.406, 552:0.373, 556:0.347, 560:0.337, 564:0.314, 568:0.285, 572:0.277, 576:0.266, 580:0.25, 584:0.23, 588:0.207, 592:0.186, 596:0.171, 600:0.16, 604:0.148, 608:0.141, 612:0.136, 616:0.13, 620:0.126, 624:0.123, 628:0.121, 632:0.122, 636:0.119, 640:0.114, 644:0.115, 648:0.117, 652:0.117, 656:0.118, 660:0.12, 664:0.122, 668:0.128, 672:0.132, 676:0.139, 680:0.144, 684:0.146, 688:0.15, 692:0.152, 696:0.157, 700:0.159")
        });
        
        m_spectrum_map.insert({
            "light",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0.78, 500:0.78, 600:0.78, 700:0.78")
        });

        m_spectrum_map.insert({
            "emitter",
            radiometry::PiecewiseLinearSpectrumDistribution<T>::from_string("400:0, 500:8, 600:15.6, 700:18.4")
        });
    }

    void make_materials() {
        // White/Box material (floor, ceiling, back wall, boxes)
        int white_id = m_material_library.add_material(
            material::LambertianMaterial<T>(m_spectrum_map.at("white"))
        );
        m_material_map["cbox_floor"] = white_id;
        m_material_map["cbox_ceiling"] = white_id;
        m_material_map["cbox_back"] = white_id;
        m_material_map["cbox_smallbox"] = white_id;
        m_material_map["cbox_largebox"] = white_id;
        
        // Red material (right wall)
        int red_id = m_material_library.add_material(
            material::LambertianMaterial<T>(m_spectrum_map.at("red"))
        );
        m_material_map["cbox_redwall"] = red_id;
        
        // Green material (left wall) 
        int green_id = m_material_library.add_material(
            material::LambertianMaterial<T>(m_spectrum_map.at("green"))
        );
        m_material_map["cbox_greenwall"] = green_id;

        // light source material
        int light_id = m_material_library.add_material(
            material::LambertianMaterial<T>(m_spectrum_map.at("light"))
        );
        m_material_map["cbox_luminaire"] = light_id;
    }

    void make_light_sources() {
        // Light source material
        auto light_spectrum = radiometry::MultipliedSpectrumDistribution<T, 
            radiometry::PiecewiseLinearSpectrumDistribution<T>, 
            radiometry::constant::CIED65SpectrumType<T>>(
                m_spectrum_map.at("emitter"),
                radiometry::constant::CIE_D65_ilum<T>
            );

        // Light mesh
        auto& light_mesh = m_mesh_map.at("cbox_luminaire");

        for (int i = 0; i < light_mesh.triangle_count(); ++i) {
            m_area_lights.emplace_back(
            light::AreaLight<T, 
                shape::Triangle<T>, 
                decltype(light_spectrum)
            >(
                shape::Triangle<T>(light_mesh, i),
                light_spectrum,
                light::AreaLightSamplingDomain::SolidAngle)
            );
        }
    }

    void make_camera_system() {
        auto camera = camera::ThinLensPerspectiveCamera<T>(
            camera::CameraProjection<T>::create_perspective_projection_by_fov(
                math::Vector<int, 2>(512, 512), 
                39.307, "smaller", 
                10, 2800),
            T(1.0), // lens radius
            T(1000.0)
        );

        auto render_transform = camera::RenderTransform<T>::look_at(
            math::Point<T, 3>(278, 273, -800), 
            math::Point<T, 3>(278, 273, -799), 
            math::Vector<T, 3>(0, 1, 0),
            camera::RenderSpace::World
        );

        auto pixel_filter = camera::GaussianFilter<T>(T(0.5), T(0.5));

        auto pixel_sensor = camera::PixelSensor<T, 
            radiometry::constant::CIED65SpectrumType<T>, 
            radiometry::constant::CIED65SpectrumType<T>, 
            radiometry::constant::XYZSpectrumType<T>
        >(
            radiometry::constant::CIE_D65_ilum<T>,
            radiometry::constant::CIE_D65_ilum<T>,
            radiometry::constant::sRGB<T>,
            radiometry::ResponseSpectrum<radiometry::constant::XYZSpectrumType<T>>(
                radiometry::constant::CIE_X<T>,
                radiometry::constant::CIE_Y<T>,
                radiometry::constant::CIE_Z<T>
            ),
            T{1.0}
        );

        auto film = camera::HDRFilm<T, decltype(pixel_sensor)>(
            camera.film_resolution(),
            math::Vector<T, 2>(
                static_cast<T>(camera.film_resolution().x()),
                static_cast<T>(camera.film_resolution().y())
            ),
            pixel_sensor
        );

        m_camera_system = camera::CameraSystem<T, 
            decltype(camera), decltype(film), decltype(pixel_filter)>
        (
            camera,
            film,
            pixel_filter,
            render_transform
        );
    }

    void make_mesh() {
        auto render_transform = m_camera_system.render_transform();
        m_mesh_map.insert({   
            "cbox_floor", 
            shape::TriangleMesh<T>(
            render_transform.object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_floor.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_ceiling", 
            shape::TriangleMesh<T>(
            render_transform.object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_ceiling.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_back", 
            shape::TriangleMesh<T>(
            render_transform.object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_back.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_greenwall", 
            shape::TriangleMesh<T>(
            render_transform.object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_greenwall.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_redwall", 
            shape::TriangleMesh<T>(
            m_camera_system.render_transform().object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_redwall.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_smallbox", 
            shape::TriangleMesh<T>(
            m_camera_system.render_transform().object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_smallbox.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_largebox", 
            shape::TriangleMesh<T>(
            m_camera_system.render_transform().object_to_render_from_object_to_world(geometry::Transform<double>::identity()),
            "meshes/cbox_largebox.obj",
            false) 
        });
        m_mesh_map.insert({   
            "cbox_luminaire", 
            shape::TriangleMesh<T>(
            m_camera_system.render_transform().object_to_render_from_object_to_world(geometry::Transform<double>::translate(Vector<T, 3>(0, -0.5, 0))),
            "meshes/cbox_luminaire.obj",
            false) 
        });
    }

    void make_aggregate() {
        std::vector<shape::Primitive<T>> primitives;
        for (const auto& [mesh_name, mesh] : m_mesh_map) {
            int material_id = m_material_map.at(mesh_name);
            for (int i = 0; i < mesh.triangle_count(); ++i) {
                primitives.push_back(
                    shape::Primitive<T>(
                        shape::Triangle<T>(mesh, i),
                        material_id
                    )
                );
            }
        }

        m_aggregate = aggregate::LinearAggregate<T>(primitives);
    }
    
public:
    CornellBoxScene() {
        make_camera_system();
        make_spectrum();
        make_materials();
        make_mesh();
        make_light_sources();
        make_aggregate();
    }

    // Accessors
    auto& camera_system() { return m_camera_system; }
    const auto& camera_system() const { return m_camera_system; }
    
    auto& aggregate() { return m_aggregate; }
    const auto& aggregate() const { return m_aggregate; }
    
    auto& area_lights() { return m_area_lights; }
    const auto& area_lights() const { return m_area_lights; }
    
    auto& material_library() { return m_material_library; }
    const auto& material_library() const { return m_material_library; }
    
    int ssp() const { return m_ssp; }
    int max_depth() const { return m_max_depth; }
};

};