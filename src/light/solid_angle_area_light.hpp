#pragma once

#include <cmath>
#include <optional>

#include "light/light.hpp"

namespace pbpt::light {

template<typename T, typename ShapeType, typename PowerSpectrumType>
class SolidAngleAreaLight : public Light<T, SolidAngleAreaLight<T, ShapeType, PowerSpectrumType>> {
    friend class Light<T, SolidAngleAreaLight<T, ShapeType, PowerSpectrumType>>;

private:
    const ShapeType m_shape;
    PowerSpectrumType m_power_spectrum;

public:
    SolidAngleAreaLight(
        const ShapeType& shape,
        const PowerSpectrumType& power_spectrum
    ) : m_shape(shape), m_power_spectrum(power_spectrum) {}

    const ShapeType& shape() const {
        return m_shape;
    }

private:
    geometry::Transform<T> render_to_light_transform_impl() const {
        return m_shape.render_to_object_transform();
    }

    geometry::Transform<T> light_to_render_transform_impl() const {
        return m_shape.object_to_render_transform();
    }

    template<int N, typename NormalInteractionType>
    std::optional<LightSampleResult<T, N, NormalInteractionType>> sample_light_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const NormalInteractionType& ref_point,
        const math::Point<T, 2>& u_sample
    ) {
        shape::ShapeSample<T> shape_sample = m_shape.sample_on_solid_angle(ref_point.point(), u_sample);

        auto ref_to_light = shape_sample.point - ref_point.point();
        T distance_squared = ref_to_light.length_squared();
        if (distance_squared <= T(0) || shape_sample.pdf <= T(0)) {
            return std::nullopt;
        }

        T distance = std::sqrt(distance_squared);
        math::Vector<T, 3> wi = ref_to_light / distance;

        T cos_theta_light = shape_sample.normal.dot(-wi);
        if (cos_theta_light <= T(0)) {
            return std::nullopt;
        }

        VisibilityTester<T, NormalInteractionType> visibility_tester(
            ref_point,
            shape_sample.point
        );

        radiometry::SampledSpectrum<T, N> radiance = m_power_spectrum.sample(sampled_wavelengths);
        return std::make_optional(LightSampleResult<T, N, NormalInteractionType>{
            radiance,
            visibility_tester,
            wi,
            shape_sample.pdf
        });
    }

    template<typename NormalInteractionType>
    T sample_light_pdf_impl(
        const NormalInteractionType& ref_point,
        const math::Vector<T, 3>& wi
    ) const {
        auto ray = ref_point.spawn_ray(wi);
        auto intersection_opt = m_shape.intersect(ray);
        if (!intersection_opt.has_value()) {
            return T(0);
        }

        const auto& interaction = intersection_opt->interaction;
        T cos_theta_light = interaction.n().dot(-wi);
        if (cos_theta_light <= T(0)) {
            return T(0);
        }

        return m_shape.sample_on_solid_angle_pdf(ref_point.point(), interaction.point());
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> emission_spectrum_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Point<T, 3>& point,
        const math::Vector<T, 3>& w_exit
    ) const {
        (void)point;
        (void)w_exit;
        return m_power_spectrum.sample(sampled_wavelengths);
    }

    bool is_delta_light_impl() const {
        return false;
    }
};

} // namespace pbpt::light
