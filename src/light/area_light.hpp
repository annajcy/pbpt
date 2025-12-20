#pragma once

#include <optional>
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/shape.hpp"
#include "light.hpp"

namespace pbpt::light {

template<typename T, typename ShapeType, typename PowerSpectrumType>
class AreaLight : public Light<T, AreaLight<T, ShapeType, PowerSpectrumType>> {
    friend class Light<T, AreaLight<T, ShapeType, PowerSpectrumType>>;
private:
    const ShapeType m_shape;
    PowerSpectrumType m_power_spectrum;

public:
    AreaLight(
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
        // Sample a point on the light source
        shape::ShapeSample<T> shape_sample = m_shape.sample_on_shape(u_sample);

        VisibilityTester<T, NormalInteractionType> visibility_tester(
            ref_point,
            shape_sample.point
        );

        
        math::Vector<T, 3> ref_to_light = (shape_sample.point - ref_point.point());
        T distance_squared = ref_to_light.length_squared();
        T distance = std::sqrt(distance_squared);
        math::Vector<T, 3> wi = ref_to_light / distance;

        // Compute cosine term at light source
        T cos_theta_light = shape_sample.normal.dot(-wi);

        // If light is facing away, return no contribution
        if (cos_theta_light <= T(0)) {
            return std::nullopt;    
        }

        // Compute emitted radiance along wi.
        // For now the power spectrum is assumed to be spatially/directionally uniform.
        radiometry::SampledSpectrum<T, N> radiance = m_power_spectrum.sample(sampled_wavelengths);

        // Compute PDF wrt solid angle at ref point
        T pdf_area = shape_sample.pdf;
        T pdf_solid_angle = pdf_area * distance_squared / cos_theta_light;
        return std::make_optional(LightSampleResult<T, N, NormalInteractionType>{
            radiance,
            visibility_tester,
            wi,
            pdf_solid_angle
        });
    }

    template<typename NormalInteractionType>
    T sample_light_pdf_impl(
        const NormalInteractionType& ref_point,
        const math::Vector<T, 3>& wi // ref to light
    ) const {
       
        auto ray = ref_point.spawn_ray(wi);
        auto intersection_opt = m_shape.intersect(ray);
        if (!intersection_opt.has_value()) {
            return T(0);
        }
        auto intersection = intersection_opt.value();

        // Compute PDF wrt solid angle at ref point
        T distance_squared = (ref_point.point() - intersection.interaction.point()).length_squared();
        T cos_theta_light = intersection.interaction.n().dot(-wi);
        if (cos_theta_light <= T(0)) {
            return T(0);
        }
        T pdf_area = m_shape.sample_on_shape_pdf(intersection.interaction.point());
        T pdf_solid_angle = distance_squared * pdf_area / cos_theta_light;
        return pdf_solid_angle;
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> emission_spectrum_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Point<T, 3>& point,
        const math::Vector<T, 3>& w_exit
    ) const {
        return m_power_spectrum.sample(sampled_wavelengths);
    }

    bool is_delta_light_impl() const {
        return false;
    }

};

};
