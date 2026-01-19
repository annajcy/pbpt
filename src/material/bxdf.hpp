#pragma once

#include <variant>
#include "math/function.hpp"
#include "math/point.hpp"
#include "sampler/3d.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

enum class BxDFTypeFlags : int {
    Unset = 0,
    // Directional types
    Reflection = 1 << 0,
    Transmission = 1 << 1,
    // Base types
    Diffuse = 1 << 2,
    Glossy = 1 << 3,
    Specular = 1 << 4,
    // Composite types
    DiffuseReflection = Diffuse | Reflection,
    DiffuseTransmission = Diffuse | Transmission,
    GlossyReflection = Glossy | Reflection,
    GlossyTransmission = Glossy | Transmission,
    SpecularReflection = Specular | Reflection,
    SpecularTransmission = Specular | Transmission,

    ALL = Reflection | Transmission | Diffuse | Glossy | Specular,
    ANY = ALL + 1  // sentinel meaning "no filtering"
};

// 重载位运算符
inline BxDFTypeFlags operator|(BxDFTypeFlags a, BxDFTypeFlags b) {
    return static_cast<BxDFTypeFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline BxDFTypeFlags operator&(BxDFTypeFlags a, BxDFTypeFlags b) {
    return static_cast<BxDFTypeFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline BxDFTypeFlags& operator|=(BxDFTypeFlags& a, BxDFTypeFlags b) {
    a = a | b;
    return a;
}

inline bool has_flag(BxDFTypeFlags value, BxDFTypeFlags flag) {
    return (static_cast<int>(value) & static_cast<int>(flag)) == static_cast<int>(flag);
}

template<typename T, int N>
struct BxDFSampleRecord {
    radiometry::SampledSpectrum<T, N> f;
    math::Vector<T, 3> wi;
    T pdf = 0;
    bool is_valid = false; // 必须添加 valid 标志
    BxDFTypeFlags sampled_type = BxDFTypeFlags::Unset; // 记录采样到的具体类型
};

template<typename Derived, typename T, int N>
class BxDF {
public:
    Derived& as_derived() { return static_cast<Derived&>(*this); }
    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }

    BxDFTypeFlags type() const { return as_derived().type_impl(); }

    radiometry::SampledSpectrum<T, N> f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi
    ) const {
        return as_derived().f_impl(swl, wo, wi);
    }

    BxDFSampleRecord<T, N> sample_f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample
    ) const {
        return as_derived().sample_f_impl(swl, wo, u_sample);
    }

    T pdf(const math::Vector<T, 3>& wo, const math::Vector<T, 3>& wi) const {
        return as_derived().pdf_impl(wo, wi); 
    }

    // 修复：只要有任何位重叠，就认为匹配
    bool is_flags_matched(BxDFTypeFlags flags) const {
        if (flags == BxDFTypeFlags::ANY) return true;
        return has_flag(this->type(), flags);
    }
};


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
