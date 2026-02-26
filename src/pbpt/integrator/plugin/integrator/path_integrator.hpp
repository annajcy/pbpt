#pragma once

#include <string_view>
#include <optional>
#include <random>
#include <iostream>

#include "pbpt/integrator/integrator.hpp"
#include "pbpt/math/utils.hpp"
#include "pbpt/scene/scene.hpp"
#include "pbpt/light_sampler/light_sampler.hpp"

namespace pbpt::integrator {

/**
 * @brief Advanced Path Tracer with Multiple Importance Sampling (MIS) and Next Event Estimation (NEE).
 *
 * @tparam T Scalar type.
 * @tparam N Number of spectral channels.
 */
template <typename T, int N>
class PathIntegrator : public Integrator<PathIntegrator<T, N>, T, N> {
    friend class Integrator<PathIntegrator<T, N>, T, N>;

private:
    int m_max_depth;
    T m_rr_threshold;

public:
    PathIntegrator(int max_depth, T rr_threshold = T(0.9)) : m_max_depth(max_depth), m_rr_threshold(rr_threshold) {}

    int max_depth() const { return m_max_depth; }
    T rr_threshold() const { return m_rr_threshold; }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_ray_impl(const SceneContextT& context, const geometry::Ray<T, 3>& ray,
                                                  const radiometry::SampledWavelength<T, N>& wavelength_sample,
                                                  SamplerT& sampler) const {
        return this->Li_loop_ray(context, ray, wavelength_sample, sampler);
    }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_ray_differential_impl(
        const SceneContextT& context, const geometry::RayDifferential<T, 3>& ray_diff,
        const radiometry::SampledWavelength<T, N>& wavelength_sample, SamplerT& sampler) const {
        return this->Li_loop_ray_differential(context, ray_diff, wavelength_sample, sampler);
    }

private:
    bool is_reached_max_depth(int depth) const { return m_max_depth != -1 && depth >= m_max_depth; }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_loop_ray(const SceneContextT& context, geometry::Ray<T, 3> ray,
                                                  const radiometry::SampledWavelength<T, N>& wavelength_sample,
                                                  SamplerT& sampler) const {
        radiometry::SampledSpectrum<T, N> L = radiometry::SampledSpectrum<T, N>::filled(T(0));
        radiometry::SampledSpectrum<T, N> beta = radiometry::SampledSpectrum<T, N>::filled(T(1));

        int bounces = 0;
        T prev_bsdf_pdf = T(0);
        std::optional<geometry::SurfaceInteraction<T>> prev_si = std::nullopt;

        while (true) {
            auto hit_opt = context.aggregate.intersect_ray(ray);

            if (!hit_opt.has_value()) {
                // Evaluated infinite lights with MIS (omitted for now as PBPT infinite lights aren't active)
                break;
            }

            const auto& hit = hit_opt.value();
            const auto& si = hit.intersection.interaction;

            // 1. Direct Hit / Emission
            if (hit.light_id != -1) {
                const auto& any_light = context.resources.any_light_library.get(hit.light_id);
                auto Le = std::visit(
                    [&](const auto& light) {
                        return light.template Le<N>(wavelength_sample, si.point(), si.n(), si.wo());
                    },
                    any_light);

                if (bounces == 0 || prev_bsdf_pdf == T(0)) {
                    L += beta * Le;
                } else if (prev_si.has_value()) {
                    // MIS weight for BSDF matching the light hit
                    T pdf_light_pick =
                        std::visit([&](const auto& ls) { return ls.pdf(hit.light_id); }, context.light_sampler);

                    T pdf_light_dir =
                        std::visit([&](const auto& light) { return light.sample_light_pdf(*prev_si, ray.direction()); },
                                   any_light);

                    T pdf_light = pdf_light_pick * pdf_light_dir;
                    T weight_bsdf = math::power_heuristic(1, prev_bsdf_pdf, 1, pdf_light);
                    L += beta * Le * weight_bsdf;
                }
            }

            if (is_reached_max_depth(bounces)) {
                break;
            }

            // Compute BSDF
            const auto& material = context.resources.any_material_library.get(hit.material_id);
            material::BSDF<T, N> bsdf = std::visit(
                [&](const auto& mat) {
                    return mat.template compute_bsdf<N>(si, hit.intersection.shading, wavelength_sample, std::nullopt);
                },
                material);

            // 2. Next Event Estimation (NEE) - Direct Light Sampling
            auto light_sample_res =
                std::visit([&](auto& ls) { return ls.sample(sampler.next_1d()); }, context.light_sampler);

            if (light_sample_res.selection_pdf > T(0)) {
                const auto* light = light_sample_res.light;
                auto light_sample = std::visit(
                    [&](const auto& l) { return l.template sample_light<N>(wavelength_sample, si, sampler.next_2d()); },
                    *light);

                if (light_sample.has_value() && light_sample->pdf > T(0)) {
                    const auto& ls = light_sample.value();
                    auto bsdf_f = bsdf.f(wavelength_sample, si.wo(), ls.wi, material::TransportMode::Radiance);

                    // Exclude delta distributions that evaluate to zero
                    if (!bsdf_f.is_all_zero()) {
                        T pdf_bsdf = bsdf.pdf(si.wo(), ls.wi, material::TransportMode::Radiance);
                        if (pdf_bsdf > T(0) && ls.visibility_tester.is_unoccluded(context.aggregate)) {
                            T cos_theta = std::abs(si.n().to_vector().dot(ls.wi));
                            T weight_light =
                                math::power_heuristic(1, ls.pdf * light_sample_res.selection_pdf, 1, pdf_bsdf);
                            L += beta * bsdf_f * ls.radiance * cos_theta * weight_light /
                                 (ls.pdf * light_sample_res.selection_pdf);
                        }
                    }
                }
            }

            // 3. BSDF Sampling (Indirect Ray Generation)
            auto bsdf_sample_opt = bsdf.sample_f(wavelength_sample, si.wo(), sampler.next_1d(), sampler.next_2d(),
                                                 material::TransportMode::Radiance);

            if (!bsdf_sample_opt.has_value() || bsdf_sample_opt->pdf == T(0)) {
                break;
            }

            const auto& bsdf_sample = bsdf_sample_opt.value();
            T cos_theta = std::abs(si.n().to_vector().dot(bsdf_sample.wi));
            beta = beta * bsdf_sample.f * cos_theta / bsdf_sample.pdf;

            if (beta.is_all_zero()) {
                break;
            }

            prev_bsdf_pdf = bsdf_sample.pdf;
            prev_si = si;
            ray = si.spawn_ray(bsdf_sample.wi);

            // 4. Russian Roulette (terminate paths effectively)
            if (bounces > 3) {
                if (sampler.next_1d() > m_rr_threshold) {
                    break;
                }
                beta = beta / m_rr_threshold;
            }

            bounces++;
        }
        return L;
    }

    template <typename SamplerT, typename SceneContextT>
        requires PathTraceContextConcept<SceneContextT, T, N>
    radiometry::SampledSpectrum<T, N> Li_loop_ray_differential(
        const SceneContextT& context, geometry::RayDifferential<T, 3> ray_diff,
        const radiometry::SampledWavelength<T, N>& wavelength_sample, SamplerT& sampler) const {
        return Li_loop_ray(context, ray_diff.main_ray(), wavelength_sample, sampler);
    }

    radiometry::SampledSpectrum<T, N> miss_shader(const radiometry::SampledWavelength<T, N>&) const {
        return radiometry::SampledSpectrum<T, N>::filled(0.0);
    }
};

}  // namespace pbpt::integrator
