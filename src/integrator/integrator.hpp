#pragma once

#include <iostream>
#include <string>
#include <variant>

#include "radiometry/sampled_spectrum.hpp"
#include "camera/camera.hpp"

#include "math/point.hpp"
#include "sampler/2d.hpp" 
#include "utils/image_io.hpp"
#include "utils/progress_bar.hpp"
#include "scene/scene.hpp"

namespace pbpt::integrator {

template<typename Derived, typename T, int N, typename Sampler>
class Integrator {
public:
    Integrator() = default;

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    void render(pbpt::scene::Scene<T>& scene, int spp = 4, std::string output_path = "output.exr", bool is_trace_ray_differential = false) {
        std::visit([&](const auto& camera, auto& film, const auto& pixel_filter, const auto& aggregate) {
            scene::SceneContext context{
                camera, film, pixel_filter, aggregate, 
                scene.render_transform, scene.resources
            };

            this->render_loop(
                context,
                spp,
                output_path,
                is_trace_ray_differential
            );
        }, scene.camera, scene.film, scene.pixel_filter, scene.aggregate);
    }

protected:
    template<typename SceneContextT>
    void render_loop(
        const SceneContextT& context,
        int spp, const std::string& output_path,
        bool is_trace_ray_differential
    ) {
        auto resolution = context.film.resolution();
        std::cout << "Starting render: " << resolution.x() << "x" << resolution.y() << " pixels, SPP=" << spp << std::endl;
        const std::size_t total_pixels = static_cast<std::size_t>(resolution.x()) * static_cast<std::size_t>(resolution.y());
        utils::ProgressBar progress_bar(total_pixels, 40, "Rendering");
        progress_bar.start(std::cout);
        
        for (int y = 0; y < resolution.y(); ++y) {
            for (int x = 0; x < resolution.x(); ++x) {
                Sampler sampler; 
                for (int s = 0; s < spp; ++s) {
                    // Generate camera ray
                    const math::Point<int, 2> pixel(x, y);
                    const auto film_uv = sampler.next_2d();
                    const auto filtered_sample = context.pixel_filter.sample_film_position(pixel, film_uv);
                    const math::Point<T, 2> lens_uv = sampler.next_2d();
                    const auto lens_position = pbpt::sampler::sample_uniform_disk_concentric(lens_uv, T(10.0)); 
                    
                    auto sample = camera::CameraSample<T>::create_thinlens_sample(
                        filtered_sample.film_position,
                        lens_position
                    );
                    
                    // Sample wavelengths
                    auto wavelength_sample = radiometry::sample_visible_wavelengths_stratified<T, N>(sampler.next_1d());
                    auto wavelength_pdf = radiometry::sample_visible_wavelengths_pdf(wavelength_sample);

                    radiometry::SampledSpectrum<T, N> Li;
                    if (is_trace_ray_differential) {
                        auto ray_diff = context.camera.generate_differential_ray(sample);
                        ray_diff = context.render_transform.camera_to_render().transform_ray_differential(ray_diff);
                        Li = this->Li_ray_differential(context, ray_diff, wavelength_sample, sampler);
                    } else {
                        auto ray = context.camera.generate_ray(sample);
                        ray = context.render_transform.camera_to_render().transform_ray_main(ray);
                        Li = this->Li_ray(context, ray, wavelength_sample, sampler);
                    }

                    // Accumulate the result to film
                    context.film.template add_sample<N>(pixel, Li, wavelength_sample, wavelength_pdf, filtered_sample.weight);
                }
            }
            progress_bar.update(std::cout, resolution.x());
        }
        progress_bar.finish(std::cout);
        // Develop and save the final image
        auto image = context.film.develop();
        pbpt::utils::write_hdr_image(output_path, image);
    }

private:
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_ray(
        const SceneContextT& context,
        const geometry::Ray<T, 3>& ray, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        Sampler& sampler
    ) {
        return as_derived().Li_ray_impl(
            context, 
            ray, 
            wavelength_sample, 
            sampler
        );
    }

    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_ray_differential(
        const SceneContextT& context,
        const geometry::RayDifferential<T, 3>& ray_diff, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        Sampler& sampler
    ) {
        return as_derived().Li_ray_differential_impl(
            context, 
            ray_diff, 
            wavelength_sample, 
            sampler
        );
    }
};

}
