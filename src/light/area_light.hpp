#pragma once

#include <cmath>
#include <optional>
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/shape.hpp"
#include "light.hpp"

namespace pbpt::light {

enum class AreaLightSamplingDomain {
    Shape,
    SolidAngle
};

template<typename T, typename ShapeType, typename PowerSpectrumType>
class AreaLight : public Light<T, AreaLight<T, ShapeType, PowerSpectrumType>> {
    friend class Light<T, AreaLight<T, ShapeType, PowerSpectrumType>>;
private:
    const ShapeType m_shape;
    PowerSpectrumType m_power_spectrum;
    AreaLightSamplingDomain m_sampling_domain;

public:
    AreaLight(
        const ShapeType& shape,
        const PowerSpectrumType& power_spectrum,
        AreaLightSamplingDomain sampling_domain = AreaLightSamplingDomain::Shape
    ) : m_shape(shape),
        m_power_spectrum(power_spectrum),
        m_sampling_domain(sampling_domain) {}

    const ShapeType& shape() const {
        return m_shape;
    }

    AreaLightSamplingDomain sampling_domain() const {
        return m_sampling_domain;
    }

    template<int N, typename NormalInteractionType>
    std::optional<LightSampleResult<T, N, NormalInteractionType>> sample_light_on_shape(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const NormalInteractionType& ref_point,
        const math::Point<T, 2>& u_sample
    ) {
        shape::ShapeSample<T> shape_sample = m_shape.sample_on_shape(u_sample);

        VisibilityTester<T, NormalInteractionType> visibility_tester(
            ref_point,
            shape_sample.point
        );

        math::Vector<T, 3> ref_to_light = (shape_sample.point - ref_point.point());
        T distance_squared = ref_to_light.length_squared();
        T distance = std::sqrt(distance_squared);
        math::Vector<T, 3> wi = ref_to_light / distance;

        T cos_theta_light = shape_sample.normal.dot(-wi);
        if (cos_theta_light <= T(0)) {
            return std::nullopt;
        }

        radiometry::SampledSpectrum<T, N> radiance = m_power_spectrum.sample(sampled_wavelengths);

        T pdf_area = shape_sample.pdf;
        T pdf_solid_angle = pdf_area * distance_squared / cos_theta_light;
        return std::make_optional(LightSampleResult<T, N, NormalInteractionType>{
            radiance,
            visibility_tester,
            wi,
            pdf_solid_angle
        });
    }

    template<int N, typename NormalInteractionType>
    std::optional<LightSampleResult<T, N, NormalInteractionType>> sample_light_on_solid_angle(
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
    T sample_light_on_shape_pdf(
        const NormalInteractionType& ref_point,
        const math::Vector<T, 3>& wi
    ) const {
        auto ray = ref_point.spawn_ray(wi);
        auto intersection_opt = m_shape.intersect(ray);
        if (!intersection_opt.has_value()) {
            return T(0);
        }
        auto intersection = intersection_opt.value();

        T distance_squared = (ref_point.point() - intersection.interaction.point()).length_squared();
        T cos_theta_light = intersection.interaction.n().dot(-wi);
        if (cos_theta_light <= T(0)) {
            return T(0);
        }
        T pdf_area = m_shape.sample_on_shape_pdf(intersection.interaction.point());
        return distance_squared * pdf_area / cos_theta_light;
    }

    template<typename NormalInteractionType>
    T sample_light_on_solid_angle_pdf(
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
        if (m_sampling_domain == AreaLightSamplingDomain::SolidAngle) {
            return sample_light_on_solid_angle(sampled_wavelengths, ref_point, u_sample);
        }
        return sample_light_on_shape(sampled_wavelengths, ref_point, u_sample);
    }

    template<typename NormalInteractionType>
    T sample_light_pdf_impl(
        const NormalInteractionType& ref_point,
        const math::Vector<T, 3>& wi
    ) const {
        if (m_sampling_domain == AreaLightSamplingDomain::SolidAngle) {
            return sample_light_on_solid_angle_pdf(ref_point, wi);
        }
        return sample_light_on_shape_pdf(ref_point, wi);
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

}  // namespace pbpt::light
