#pragma once

#include "math/function.hpp"
#include "math/point.hpp"
#include "math/sampling.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"
namespace pbpt::material {

template<typename T, int N>
struct BxDFSampleRecord {
    radiometry::SampledSpectrum<T, N> f;
    math::Vector<T, 3> wi;
    T pdf;
};

template<typename T, typename Derived>
class BxDF {
public:

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }

    template<int N>
    radiometry::SampledSpectrum<T, N> f(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi
    ) {
        return as_derived().f_impl(sampled_wavelengths, wo, wi);
    }

    template<int N>
    BxDFSampleRecord<T, N> sample_f(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample
    ) {
        return as_derived().sample_f_impl(sampled_wavelengths, wo, u_sample);
    }

    T sample_f_pdf(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi
    ) {
        return as_derived().sample_f_pdf_impl(wo, wi); 
    }

};

template<typename T>
class LambertianBxDF : public BxDF<T, LambertianBxDF<T>> {
    friend class BxDF<T, LambertianBxDF<T>>;
private:
    template<int N>
    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi
    ) {
        return radiometry::SampledSpectrum<T, N>::filled(
            1.0 / math::pi_v<T>
        );
    };

    template<int N>
    BxDFSampleRecord<T, N> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& sampled_wavelengths,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample
    ) {
        auto wi_p = math::sample_cosine_weighted_hemisphere(u_sample);
        auto wi_p_pdf = math::sample_cosine_weighted_hemisphere_pdf(wi_p);
        auto wi = wi_p.to_vector();
        BxDFSampleRecord<T, N> record;
        record.wi = wi;
        record.pdf = wi_p_pdf;
        record.f = this->f(sampled_wavelengths, wo, wi);
        return record;
    }

    T sample_f_pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi
    ) {
        auto wi_p = math::Point<T, 3>(wi);
        return math::sample_cosine_weighted_hemisphere_pdf(wi_p);
    }
};

};