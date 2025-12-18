#pragma once

#include <optional>
#include "geometry/transform.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "shape/shape.hpp"

namespace pbpt::light {

template<typename T, typename NormalInteractionType>
class VisibilityTester {
private:
    /// Points in render space, between which visibility is tested.
    /// from m_src_i to m_dst_p

    NormalInteractionType m_src_i;
    math::Point<T, 3> m_dst_p;

public:

    VisibilityTester(
        const NormalInteractionType& src_interaction, 
        const math::Point<T, 3>& dst_point
    ) : m_src_i(src_interaction), m_dst_p(dst_point) {}
   
    template<typename Aggregate>
    bool is_unoccluded(const Aggregate& aggregate) const {
        auto ray = m_src_i.spawn_ray_to(m_dst_p);
        return !aggregate.is_intersected(ray);        
    }

    const NormalInteractionType& src_interaction() const {
        return m_src_i;
    }

    const math::Point<T, 3>& dst_point() const {
        return m_dst_p;
    }
};

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
        const math::Vector<T, 3>& wi_light
    ) const {
        return as_derived().sample_light_pdf_impl(
            ref_point,
            wi_light
        );
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> emission_spectrum(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths
    ) const {
        return as_derived().emission_spectrum_impl(
            sampled_wavelengths
        );
    }

    bool is_delta_light() const {
        return as_derived().is_delta_light_impl();
    }
};

template<typename T, typename ShapeType, typename PowerSpectrumType>
class AreaLight : public Light<T, AreaLight<T, ShapeType, PowerSpectrumType>> {
    friend class Light<T, AreaLight<T, ShapeType, PowerSpectrumType>>;
private:
    const shape::TransformedShape<T, ShapeType>& m_transformed_shape;
    PowerSpectrumType m_power_spectrum;

public:
    AreaLight(
        const shape::TransformedShape<T, ShapeType>& transformed_shape,
        const PowerSpectrumType& power_spectrum
    ) : m_transformed_shape(transformed_shape), m_power_spectrum(power_spectrum) {}

    const shape::TransformedShape<T, ShapeType>& transform_shape() const {
        return m_transformed_shape;
    }

private:
    geometry::Transform<T> render_to_light_transform_impl() const {
        return m_transformed_shape.render_to_object_transform();
    }

    geometry::Transform<T> light_to_render_transform_impl() const {
        return m_transformed_shape.object_to_render_transform();
    }


    template<int N, typename NormalInteractionType>
    std::optional<LightSampleResult<T, N, NormalInteractionType>> sample_light_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const NormalInteractionType& ref_point,
        const math::Point<T, 2>& u_sample
    ) {
        // Sample a point on the light source
        shape::ShapeSample<T> shape_sample = m_transformed_shape.sample_on_shape(u_sample);

        VisibilityTester<T, NormalInteractionType> visibility_tester(
            ref_point,
            shape_sample.point
        );

        // Compute emitted radiance along wi
        math::Vector<T, 3> ref_to_light = (shape_sample.point - ref_point.point());
        T distance_squared = ref_to_light.length_squared();
        T distance = std::sqrt(distance_squared);
        math::Vector<T, 3> wi = ref_to_light / distance;
        T cos_theta_light = shape_sample.normal.dot(-wi);

        if (cos_theta_light <= T(0)) {
            return std::nullopt;    
        }

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
        auto intersection_opt = m_transformed_shape.intersect(ray);
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
        T pdf_area = m_transformed_shape.sample_on_shape_pdf(intersection.interaction.point());
        T pdf_solid_angle = distance_squared * pdf_area / cos_theta_light;
        return pdf_solid_angle;
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> emission_spectrum_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths
    ) const {
        return m_power_spectrum.sample(sampled_wavelengths);
    }

     bool is_delta_light_impl() const {
        return false;
    }

};

};
