#pragma once

#include <cmath>
#include <optional>

#include "material/bxdf.hpp"
#include "math/function.hpp"
#include "sampler/3d.hpp"

namespace pbpt::material {

template<typename T>
bool is_same_hemisphere(const math::Vector<T, 3>& w, const math::Vector<T, 3>& wp) {
    return w.z() * wp.z() > 0;
}

// --- Lambertian 实现 ---
template<typename T, int N>
class LambertianBxDF : public BxDF<LambertianBxDF<T, N>, T, N> {
    friend class BxDF<LambertianBxDF<T, N>, T, N>;
private:
    radiometry::SampledSpectrum<T, N> m_albedo;

public:
    LambertianBxDF(const radiometry::SampledSpectrum<T, N>& albedo) : m_albedo(albedo) {}

private:
    BxDFTypeFlags type_impl() const {
        return BxDFTypeFlags::DiffuseReflection;
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode,
        const BxDFTypeFlags flags = BxDFTypeFlags::ALL
    ) const {
        if (!is_match_flags(type_impl(), flags)) {
            return radiometry::SampledSpectrum<T, N>::filled(0);
        }
        if (!is_same_hemisphere(wo, wi)) return radiometry::SampledSpectrum<T, N>::filled(0);
        return m_albedo * (1.0 / math::pi_v<T>);
    };

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T,
        const math::Point<T, 2>& u_sample,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        if (!is_match_refl_trans(this->type(), sample_flags)) {
            return std::nullopt;
        }
        // 余弦采样得到的是局部坐标
        auto wi_p = sampler::sample_cosine_weighted_hemisphere(u_sample);
        auto wi = wi_p.to_vector();

        // 如果 wo 在下半球，我们需要翻转 wi 到下半球
        if (wo.z() < 0) wi.z() *= -1;

        BxDFSampleRecord<T, N> record;
        record.wi = wi;
        record.pdf = sampler::sample_cosine_weighted_hemisphere_pdf(wi_p); // cos(theta)/pi
        record.f = this->f_impl(swl, wo, wi, mode); 
        record.sampled_flags = type_impl();
        return record;
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) return 0;
        if (!is_same_hemisphere(wo, wi)) return 0;
        // 注意：这里 wi 是局部坐标，AbsCosTheta 就是 abs(wi.z())
        return std::abs(wi.z()) * (1.0 / math::pi_v<T>);
    }
};

} // namespace pbpt::material
