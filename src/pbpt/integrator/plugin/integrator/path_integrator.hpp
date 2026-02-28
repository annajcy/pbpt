#pragma once

#include <string_view>
#include <optional>
#include <random>
#include <iostream>
#include <atomic>

#include "pbpt/integrator/integrator.hpp"
#include "pbpt/material/bxdf.hpp"
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

        //TODO: ADD MIS and NEE
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
