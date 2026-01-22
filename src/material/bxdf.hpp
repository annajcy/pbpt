#pragma once

#include "math/point.hpp"
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


} // namespace pbpt::material
