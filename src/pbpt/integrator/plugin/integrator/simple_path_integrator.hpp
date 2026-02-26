#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <variant>

#include "pbpt/geometry/ray.hpp"
#include "pbpt/integrator/concepts.hpp"
#include "pbpt/integrator/integrator.hpp"
#include "pbpt/lds/plugin/lds/independent.hpp"
#include "pbpt/material/bsdf.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/shape/primitive.hpp"

namespace pbpt::integrator {

template <typename T, int N>
class SimplePathIntegrator : public Integrator<SimplePathIntegrator<T, N>, T, N> {
    friend class Integrator<SimplePathIntegrator<T, N>, T, N>;

private:
    unsigned m_max_depth = -1;
    T m_rr = T(0.9);

public:
    SimplePathIntegrator(unsigned max_depth = -1, T rr = T(0.9)) : m_max_depth(max_depth), m_rr(rr) {}

    unsigned max_depth() const { return m_max_depth; }
    void set_max_depth(unsigned depth) { m_max_depth = depth; }

    T rr_threshold() const { return m_rr; }
    void set_rr_threshold(T rr) { m_rr = rr; }

private:
    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_ray_impl(const SceneContextT& context, const geometry::Ray<T, 3>& ray,
                                                  const radiometry::SampledWavelength<T, N>& wavelength_sample,
                                                  SamplerT& sampler) {
        return this->Li_loop_ray(context, ray, wavelength_sample, sampler);
    }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_ray_differential_impl(
        const SceneContextT& context, const geometry::RayDifferential<T, 3>& ray_diff,
        const radiometry::SampledWavelength<T, N>& wavelength_sample, SamplerT& sampler) {
        return this->Li_loop_ray_differential(context, ray_diff, wavelength_sample, sampler);
    }

private:
    bool is_reached_max_depth(int depth) const {
        return m_max_depth != std::numeric_limits<unsigned>::max() && static_cast<unsigned>(depth) >= m_max_depth;
    }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_loop_ray(const SceneContextT& context, geometry::Ray<T, 3> ray,
                                                  const radiometry::SampledWavelength<T, N>& wavelength_sample,
                                                  SamplerT& sampler) {
        radiometry::SampledSpectrum<T, N> radiance = radiometry::SampledSpectrum<T, N>::filled(0);
        radiometry::SampledSpectrum<T, N> throughput = radiometry::SampledSpectrum<T, N>::filled(1);

        for (int depth = 0; !is_reached_max_depth(depth); ++depth) {
            // Intersect ray with the scene
            std::optional<shape::PrimitiveIntersectionRecord<T>> hit_opt = context.aggregate.intersect_ray(ray);
            if (!hit_opt) {
                break;
            }

            const auto& hit = hit_opt.value();
            // Compute BSDF at the intersection point
            const auto& material = context.resources.any_material_library.get(hit.material_id);
            material::BSDF<T, N> bsdf = std::visit(
                [&](const auto& mat) {
                    return mat.template compute_bsdf<N>(hit.intersection.interaction, hit.intersection.shading,
                                                        wavelength_sample, std::nullopt);
                },
                material);

            // If the ray hits a light source, evaluate the emitted radiance and add it to the accumulated radiance
            if (hit.light_id != -1) {
                const auto& any_light = context.resources.any_light_library.get(hit.light_id);
                std::visit(
                    [&](const auto& light) {
                        radiance +=
                            throughput * light.Le(wavelength_sample, hit.intersection.interaction.point(),
                                                  hit.intersection.interaction.n(), hit.intersection.interaction.wo());
                    },
                    any_light);
            }

            // Sample the BSDF to get the next ray direction and update the throughput
            auto bsdf_sample_record_opt =
                bsdf.sample_f(wavelength_sample, hit.intersection.interaction.wo(), sampler.next_1d(),
                              sampler.next_2d(), material::TransportMode::Radiance);

            // If the BSDF sampling fails (e.g., due to total internal reflection), terminate the path
            if (!bsdf_sample_record_opt) {
                break;
            }
            auto& bsdf_sample_record = bsdf_sample_record_opt.value();

            // If the sampled direction has zero contribution, terminate the path
            if (bsdf_sample_record.pdf <= 0) {
                break;
            }
            const T cos_term = std::abs(bsdf_sample_record.wi.dot(hit.intersection.interaction.n().to_vector()));
            throughput = throughput * bsdf_sample_record.f * (cos_term / bsdf_sample_record.pdf);

            if (depth > 3) {
                if (sampler.next_1d() > m_rr) {
                    break;
                }
                throughput = throughput / m_rr;
            }

            ray = hit.intersection.interaction.spawn_ray(bsdf_sample_record.wi);
        }

        return radiance;
    }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_loop_ray_differential(
        const SceneContextT& context, geometry::RayDifferential<T, 3> ray_diff,
        const radiometry::SampledWavelength<T, N>& wavelength_sample, SamplerT& sampler) {
        radiometry::SampledSpectrum<T, N> radiance = radiometry::SampledSpectrum<T, N>::filled(0);
        radiometry::SampledSpectrum<T, N> throughput = radiometry::SampledSpectrum<T, N>::filled(1);

        bool trace_differential = true;
        geometry::Ray<T, 3> ray = ray_diff.main_ray();

        for (int depth = 0; !is_reached_max_depth(depth); ++depth) {
            auto hit_opt = trace_differential ? context.aggregate.intersect_ray_differential(ray_diff)
                                              : context.aggregate.intersect_ray(ray);
            if (!hit_opt) {
                break;
            }

            const auto& prim_intersection_rec = hit_opt.value();
            const auto& material = context.resources.any_material_library.get(prim_intersection_rec.material_id);

            const auto si_differentials =
                trace_differential ? prim_intersection_rec.intersection.differentials : std::nullopt;

            material::BSDF<T, N> bsdf = std::visit(
                [&](const auto& mat) {
                    return mat.template compute_bsdf<N>(prim_intersection_rec.intersection.interaction,
                                                        prim_intersection_rec.intersection.shading, wavelength_sample,
                                                        si_differentials);
                },
                material);

            if (prim_intersection_rec.light_id != -1) {
                const auto& any_light = context.resources.any_light_library.get(prim_intersection_rec.light_id);
                std::visit(
                    [&](const auto& light) {
                        radiance += throughput * light.Le(wavelength_sample,
                                                          prim_intersection_rec.intersection.interaction.point(),
                                                          prim_intersection_rec.intersection.interaction.n(),
                                                          prim_intersection_rec.intersection.interaction.wo());
                    },
                    any_light);
            }

            auto bsdf_sample_record_opt =
                bsdf.sample_f(wavelength_sample, prim_intersection_rec.intersection.interaction.wo(), sampler.next_1d(),
                              sampler.next_2d(), material::TransportMode::Radiance);

            if (!bsdf_sample_record_opt) {
                break;
            }

            auto& bsdf_sample_record = bsdf_sample_record_opt.value();
            if (bsdf_sample_record.pdf <= 0) {
                break;
            }

            const T cos_term =
                std::abs(bsdf_sample_record.wi.dot(prim_intersection_rec.intersection.interaction.n().to_vector()));
            throughput = throughput * bsdf_sample_record.f * (cos_term / bsdf_sample_record.pdf);

            if (depth > 3) {
                if (sampler.next_1d() > m_rr) {
                    break;
                }
                throughput = throughput / m_rr;
            }

            bool has_next_differential = false;
            if (trace_differential && prim_intersection_rec.intersection.differentials.has_value()) {
                auto diffs_offset_opt = material::make_ray_differential_offset(
                    bsdf_sample_record.sampled_flags, ray_diff, prim_intersection_rec.intersection.interaction,
                    prim_intersection_rec.intersection.shading, *prim_intersection_rec.intersection.differentials,
                    bsdf_sample_record.wi, bsdf_sample_record.eta);
                if (diffs_offset_opt) {
                    ray_diff = prim_intersection_rec.intersection.interaction.spawn_ray_differential(
                        bsdf_sample_record.wi, *diffs_offset_opt);
                    ray = ray_diff.main_ray();
                    has_next_differential = true;
                }
            }

            if (!has_next_differential) {
                ray = prim_intersection_rec.intersection.interaction.spawn_ray(bsdf_sample_record.wi);
                trace_differential = false;
            }
        }

        return radiance;
    }

    radiometry::SampledSpectrum<T, N> miss_shader(const radiometry::SampledWavelength<T, N>& wavelength_sample) const {
        return radiometry::SampledSpectrum<T, N>::filled(0.0);
    }
};

}  // namespace pbpt::integrator
