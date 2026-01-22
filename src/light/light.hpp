#pragma once

#include <optional>
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "math/normal.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "visibility_tester.hpp"

namespace pbpt::light {
/**
 * @brief Result of sampling a light source.
 *
 * Contains the emitted radiance along the sampled direction,
 * a visibility tester to check for occlusion, the sampled
 * direction wi, and the PDF of the sample wrt solid angle. 
 * Note that wi and pdf are defined in the perspective of the shading point (the ref point).
 */
template<typename T, int N, typename NormalInteractionType>
struct LightSampleResult {
    radiometry::SampledSpectrum<T, N> radiance;
    VisibilityTester<T, NormalInteractionType> visibility_tester;
    math::Vector<T, 3> wi;
    T pdf;
};

/**
 * @brief Base class for light sources using CRTP.
 *
 * Derived light classes should implement the following methods:
 * - render_to_light_transform_impl()
 * - light_to_render_transform_impl()
 * - sample_light_impl()
 * - sample_light_pdf_impl()
 * - emission_spectrum_impl()
 * - is_delta_light_impl()
 *
 * @tparam T      Scalar type (e.g. float or double).
 * @tparam Derived Concrete light type.
 */
template<typename T, typename Derived>
class Light {
public:
    geometry::Transform<T> render_to_light_transform() const {
        return as_derived().render_to_light_transform_impl();
    }

    geometry::Transform<T> light_to_render_transform() const {
        return as_derived().light_to_render_transform_impl();
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    template<int N, typename NormalInteractionType>
    std::optional<LightSampleResult<T, N, NormalInteractionType>> sample_light(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const NormalInteractionType& ref_point,
        const math::Point<T, 2>& u_sample
    ) {
        return as_derived().sample_light_impl(
            sampled_wavelengths,
            ref_point,
            u_sample
        );
    }

    /// PDF for sampling the light from a reference point. 
    // wi is the direction from the light to the ref point. 
    // pointing outwards from the light to the ref point.
    template<typename NormalInteractionType>
    T sample_light_pdf(
        const NormalInteractionType& ref_point,
        const math::Vector<T, 3>& wi
    ) const {
        return as_derived().sample_light_pdf_impl(
            ref_point,
            wi
        );
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> emission_spectrum(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Point<T, 3>& point,
        const math::Normal<T, 3>& normal,
        const math::Vector<T, 3>& wo
    ) const {
        return as_derived().emission_spectrum_impl(
            sampled_wavelengths,
            point,
            normal,
            wo
        );
    }

    bool is_delta_light() const {
        return as_derived().is_delta_light_impl();
    }
};

};
