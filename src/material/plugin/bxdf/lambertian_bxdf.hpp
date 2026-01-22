#pragma once

#include <cmath>

#include "material/bxdf.hpp"
#include "math/function.hpp"
#include "sampler/3d.hpp"

namespace pbpt::material {

// --- Lambertian 实现 ---
template<typename T, int N>
class LambertianBxDF : public BxDF<LambertianBxDF<T, N>, T, N> {
    friend class BxDF<LambertianBxDF<T, N>, T, N>;
private:
    radiometry::SampledSpectrum<T, N> m_albedo;

    // 辅助：检查是否同侧
    bool same_hemisphere(const math::Vector<T, 3>& w, const math::Vector<T, 3>& wp) const {
        return w.z() * wp.z() > 0;
    }

public:
    LambertianBxDF(const radiometry::SampledSpectrum<T, N>& albedo) : m_albedo(albedo) {}

private:
    BxDFTypeFlags type_impl() const {
        return BxDFTypeFlags::DiffuseReflection;
    }

    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi
    ) const {
        if (!same_hemisphere(wo, wi)) return radiometry::SampledSpectrum<T, N>::filled(0);
        return m_albedo * (1.0 / math::pi_v<T>);
    };

    BxDFSampleRecord<T, N> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample
    ) const {
        // 余弦采样得到的是局部坐标
        auto wi_p = sampler::sample_cosine_weighted_hemisphere(u_sample);
        auto wi = wi_p.to_vector();

        // 如果 wo 在下半球，我们需要翻转 wi 到下半球
        if (wo.z() < 0) wi.z() *= -1;

        BxDFSampleRecord<T, N> record;
        record.wi = wi;
        record.pdf = sampler::sample_cosine_weighted_hemisphere_pdf(wi_p); // cos(theta)/pi
        record.f = this->f_impl(swl, wo, wi); // 使用 impl 避免虚函数开销
        record.is_valid = true;
        record.sampled_type = type_impl();
        return record;
    }

    T pdf_impl(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi
    ) const {
        if (!same_hemisphere(wo, wi)) return 0;
        // 注意：这里 wi 是局部坐标，AbsCosTheta 就是 abs(wi.z())
        return std::abs(wi.z()) * (1.0 / math::pi_v<T>);
    }
};

} // namespace pbpt::material
