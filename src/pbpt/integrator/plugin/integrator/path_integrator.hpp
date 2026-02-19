#pragma once

#include "pbpt/integrator/integrator.hpp"
#include "pbpt/lds/plugin/lds/independent.hpp"

#include <cstddef>
#include <variant>
#include <cmath>
#include <optional>
#include "pbpt/geometry/ray.hpp"
#include "pbpt/material/bsdf.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/shape/primitive.hpp"

namespace pbpt::integrator {

template<typename T, int N, typename Sampler = pbpt::lds::IndependentSampler<T>>
class PathIntegrator : public Integrator<PathIntegrator<T, N, Sampler>, T, N, Sampler> {
    friend class Integrator<PathIntegrator<T, N, Sampler>, T, N, Sampler>;
private:
    unsigned m_max_depth = -1;
    T m_rr_threshold = T(0.9);

public:
    PathIntegrator(unsigned max_depth = -1, T rr_threshold = T(0.9)) 
    : m_max_depth(max_depth), m_rr_threshold(rr_threshold) {}
    
    unsigned max_depth() const { return m_max_depth; }
    T rr_threshold() const { return m_rr_threshold; }

private:
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_ray_impl(
        const SceneContextT& context,
        const geometry::Ray<T, 3>& ray, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        Sampler& sampler
    ) {
        return this->Li_dfs_ray(
            context,
            ray,
            wavelength_sample,
            0,
            sampler
        );
    }

    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_ray_differential_impl(
        const SceneContextT& context,
        const geometry::RayDifferential<T, 3>& ray_diff, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        Sampler& sampler
    ) {
        return this->Li_dfs_ray_differential(
            context,
            ray_diff,
            wavelength_sample,
            0,
            sampler
        );
    }

private:
    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_dfs_ray(
        const SceneContextT& context,
        const geometry::Ray<T, 3>& ray, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        auto hit_opt = context.aggregate.intersect_ray(ray);

        if (hit_opt) {
            return this->hit_shader(
                context,
                ray,
                hit_opt.value(), 
                wavelength_sample, 
                depth,
                sampler);
        } else {
            return this->miss_shader(wavelength_sample);
        }
    }

    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> Li_dfs_ray_differential(
        const SceneContextT& context,
        const geometry::RayDifferential<T, 3>& ray_diff, 
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        auto hit_opt = context.aggregate.intersect_ray_differential(ray_diff);

        if (hit_opt) {
            return this->hit_shader_differential(
                context,
                ray_diff,
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
        const geometry::Ray<T, 3>& ray,
        const shape::PrimitiveIntersectionRecord<T>& prim_intersection_rec,
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        (void)ray;
        auto material = context.resources.any_material_library.get(prim_intersection_rec.material_id);
        material::BSDF<T, N> bsdf = std::visit([&](const auto& mat) {
            return mat.template compute_bsdf<N>(
                prim_intersection_rec.intersection.interaction,
                prim_intersection_rec.intersection.shading,
                wavelength_sample,
                std::nullopt
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
            sampler.next_1d(),
            sampler.next_2d(),
            material::TransportMode::Radiance
        );

        if (!bsdf_sample_record || bsdf_sample_record->pdf <= 0) {
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
            result += bsdf_sample_record->f * std::abs(bsdf_sample_record->wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record->pdf / m_rr_threshold * this->Li_dfs_ray(
                context,
                prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record->wi),
                wavelength_sample,
                depth + 1,
                sampler
            );

            return result;
        } else {
            result += bsdf_sample_record->f * std::abs(bsdf_sample_record->wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record->pdf * this->Li_dfs_ray(
                context,
                prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record->wi),
                wavelength_sample,
                depth + 1,
                sampler
            );
            return result;
        }
    }

    template<typename SceneContextT>
    radiometry::SampledSpectrum<T, N> hit_shader_differential(
        const SceneContextT& context,
        const geometry::RayDifferential<T, 3>& ray_diff,
        const shape::PrimitiveIntersectionRecord<T>& prim_intersection_rec,
        const radiometry::SampledWavelength<T, N>& wavelength_sample,
        int depth,
        Sampler& sampler
    ) {
        auto material = context.resources.any_material_library.get(prim_intersection_rec.material_id);
        material::BSDF<T, N> bsdf = std::visit([&](const auto& mat) {
            return mat.template compute_bsdf<N>(
                prim_intersection_rec.intersection.interaction,
                prim_intersection_rec.intersection.shading,
                wavelength_sample,
                prim_intersection_rec.intersection.differentials
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

        auto bsdf_sample_record = bsdf.sample_f(
            wavelength_sample, 
            prim_intersection_rec.intersection.interaction.wo(),
            sampler.next_1d(),
            sampler.next_2d(),
            material::TransportMode::Radiance
        );

        if (!bsdf_sample_record || bsdf_sample_record->pdf <= 0) {
            return result; 
        }

        if (depth == m_max_depth) {
            return result;
        }

        auto trace_next = [&]() {
            const bool has_hit_differentials = prim_intersection_rec.intersection.differentials.has_value();
            if (has_hit_differentials) {
                auto diffs_offset_opt = material::make_ray_differential_offset(
                    bsdf_sample_record->sampled_flags,
                    ray_diff,
                    prim_intersection_rec.intersection.interaction,
                    prim_intersection_rec.intersection.shading,
                    *prim_intersection_rec.intersection.differentials,
                    bsdf_sample_record->wi,
                    bsdf_sample_record->eta
                );
                if (diffs_offset_opt) {
                    auto ray_next_diff = prim_intersection_rec.intersection.interaction.spawn_ray_differential(
                        bsdf_sample_record->wi, *diffs_offset_opt
                    );
                    return this->Li_dfs_ray_differential(
                        context, ray_next_diff, wavelength_sample, depth + 1, sampler
                    );
                }
            }

            return this->Li_dfs_ray(
                context,
                prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record->wi),
                wavelength_sample,
                depth + 1,
                sampler
            );
        };

        if (depth > 3) {
            // Russian roulette
            if (sampler.next_1d() > m_rr_threshold) { 
               return result;
            }
            // re-scaling: / p_rr
            result += bsdf_sample_record->f * std::abs(bsdf_sample_record->wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record->pdf / m_rr_threshold * trace_next();

            return result;
        } else {
            result += bsdf_sample_record->f * std::abs(bsdf_sample_record->wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector())) 
            / bsdf_sample_record->pdf * trace_next();
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
