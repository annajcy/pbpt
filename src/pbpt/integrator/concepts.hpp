#pragma once

#include <type_traits>
#include <utility>

#include "pbpt/aggregate/concepts.hpp"
#include "pbpt/camera/concepts.hpp"
#include "pbpt/lds/concepts.hpp"
#include "pbpt/scene/concepts.hpp"

namespace pbpt::integrator {

template <typename SamplerT, typename T>
concept IntegratorSamplerConcept = lds::Sampler1D2DConcept<SamplerT, T>;

template <typename SceneContextT, typename T, int N>
concept RenderLoopContextConcept =
    requires(const SceneContextT& context) {
        context.camera;
        context.film;
        context.pixel_filter;
        context.render_transform;
        context.resources;
    } &&
    camera::CameraRayGeneratorConcept<std::remove_cvref_t<decltype(std::declval<const SceneContextT&>().camera)>, T> &&
    camera::PixelFilterSampleConcept<std::remove_cvref_t<decltype(std::declval<const SceneContextT&>().pixel_filter)>,
                                     T> &&
    camera::FilmAccumulationConcept<std::remove_cvref_t<decltype(std::declval<SceneContextT&>().film)>, T, N> &&
    camera::RenderTransformRayConcept<
        std::remove_cvref_t<decltype(std::declval<const SceneContextT&>().render_transform)>, T> &&
    scene::SceneResourcesConcept<std::remove_cvref_t<decltype(std::declval<const SceneContextT&>().resources)>>;

template <typename SceneContextT, typename T, int N>
concept PathTraceContextConcept =
    RenderLoopContextConcept<SceneContextT, T, N> && requires(const SceneContextT& context) { context.aggregate; } &&
    aggregate::AggregateIntersectConcept<std::remove_cvref_t<decltype(std::declval<const SceneContextT&>().aggregate)>,
                                         T> &&
    scene::LightSamplerAccessConcept<SceneContextT>;

}  // namespace pbpt::integrator
