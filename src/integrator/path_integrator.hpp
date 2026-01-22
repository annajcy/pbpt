#pragma once

#include "integrator/integrator.hpp"
#include "lds/independent.hpp"

#include <cstddef>
#include <variant>
#include <cmath>
#include "geometry/ray.hpp"
#include "material/bsdf.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/primitive.hpp"

namespace pbpt::integrator {

template<typename T, int N, typename Sampler = pbpt::lds::IndependentSampler<T>>
class PathIntegrator : public Integrator<PathIntegrator<T, N, Sampler>, T, N, Sampler> {
    friend class Integrator<PathIntegrator<T, N, Sampler>, T, N, Sampler>;
private:
    unsigned m_max_depth = -1;
    T m_rr_threshold = T(0.9);

public:
    PathIntegrator(unsigned max_depth, T rr_threshold) 
    : m_max_depth(max_depth), m_rr_threshold(rr_threshold) {}
    
    unsigned max_depth() const { return m_max_depth; }
    T rr_threshold() const { return m_rr_threshold; }

private:
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_impl(
        const SceneContextT& context,
        const geometry::Ray<T, 3>& ray, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        Sampler& sampler
    ) {
        return this->Li_dfs(
            context,
            ray,
            wavelength_sample,
            0,
            sampler
        );
    }

private:
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_dfs(
        const SceneContextT& context,
        const geometry::Ray<T, 3>& ray, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        auto hit_opt = context.aggregate.intersect(ray);

        if (hit_opt) {
            return this->hit_shader(
                context,
                hit_opt.value(), 
                wavelength_sample, 
                depth,
                sampler);
        } else {
            return this->miss_shader(wavelength_sample);
        }
    }
    
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> hit_shader(
        const SceneContextT& context,
        const shape::PrimitiveIntersectionRecord<T>& prim_intersection_rec,
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        auto material = context.resources.any_material_library.get(prim_intersection_rec.material_id);
        material::BSDF<T, N> bsdf;
        std::visit([&](const auto& mat) {
            bsdf = mat.template compute_bsdf<N>(
                prim_intersection_rec.intersection.interaction,
                prim_intersection_rec.intersection.shading,
                wavelength_sample
            );
        }, material);

        radiometry::SampledSpectrum<T, N> result = radiometry::SampledSpectrum<T, N>::filled(0);

        // hit light source
        if (prim_intersection_rec.light_id != -1) {
            const auto& any_light = context.resources.any_light_library.get(prim_intersection_rec.light_id);
            std::visit([&](const auto& light){
                 result += light.emission_spectrum(
                    wavelength_sample, 
                    prim_intersection_rec.intersection.interaction.point(),
                    prim_intersection_rec.intersection.interaction.n(),
                    prim_intersection_rec.intersection.interaction.wo()
                );
            }, any_light);
           
            return result;
        }

        // not hit light source
        auto bsdf_sample_record = bsdf.sample_f(
            wavelength_sample, 
            prim_intersection_rec.intersection.interaction.wo(),
            sampler.next_2d()
        );

        if (!bsdf_sample_record.is_valid || bsdf_sample_record.pdf <= 0) {
            return result; 
        }

        if (depth == m_max_depth) {
            return result;
        }

        if (depth > 3) {
            // Russian roulette
            if (sampler.next_1d() > m_rr_threshold) { 
               return result;
            }
            // re-scaling: / p_rr
            result += bsdf_sample_record.f * std::abs(bsdf_sample_record.wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record.pdf / m_rr_threshold * this->Li_dfs(
                context,
                prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record.wi),
                wavelength_sample,
                depth + 1,
                sampler
            );

            return result;
        } else {
            result += bsdf_sample_record.f * std::abs(bsdf_sample_record.wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record.pdf * this->Li_dfs(
                context,
                prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record.wi),
                wavelength_sample,
                depth + 1,
                sampler
            );
            return result;
        }
    }

    radiometry::SampledSpectrum<T, N> miss_shader(
        const radiometry::SampledWavelength<T, N>& wavelength_sample
    ) const {
        return radiometry::SampledSpectrum<T, N>::filled(0.0);
    }
};

}
