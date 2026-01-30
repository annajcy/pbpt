#pragma once

#include <optional>
#include "math/point.hpp"
#include "math/vector.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

enum class TransportMode : int {
    Radiance,
    Importance
};

enum class BxDFFlags : int {
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
inline BxDFFlags operator|(BxDFFlags a, BxDFFlags b) {
    return static_cast<BxDFFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline BxDFFlags operator&(BxDFFlags a, BxDFFlags b) {
    return static_cast<BxDFFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline BxDFFlags& operator|=(BxDFFlags& a, BxDFFlags b) {
    a = a | b;
    return a;
}

// is type the subset of filters? 
inline bool is_match_flags(BxDFFlags type, BxDFFlags filters) {
    return filters == BxDFFlags::ANY || (type & filters) == type;
}

// does type has the flag? even if only one flag bit exists in flag, return true
inline bool has_flag(BxDFFlags type, BxDFFlags flag) {
    return (type & flag) != BxDFFlags::Unset;
}

enum class BxDFReflTransFlags : int {
    Unset = 0,
    Reflection = 1 << 0,
    Transmission = 1 << 1,
    All = Reflection | Transmission
};

inline bool operator==(BxDFReflTransFlags a, int b) {
    return static_cast<int>(a) == b;
}

inline bool operator!(BxDFReflTransFlags a) {
    return a == BxDFReflTransFlags::Unset;
}

inline BxDFReflTransFlags operator|(BxDFReflTransFlags a, BxDFReflTransFlags b) {
    return static_cast<BxDFReflTransFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline BxDFReflTransFlags operator&(BxDFReflTransFlags a, BxDFReflTransFlags b) {
    return static_cast<BxDFReflTransFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline BxDFReflTransFlags& operator|=(BxDFReflTransFlags& a, BxDFReflTransFlags b) {
    a = a | b;
    return a;
}

inline bool has_flag(BxDFReflTransFlags type, BxDFReflTransFlags flag) {
    return (type & flag) != BxDFReflTransFlags::Unset;
}

inline bool is_match_refl_trans(BxDFFlags type, BxDFReflTransFlags flags) {
    if (flags == BxDFReflTransFlags::All) return true;
    if (flags == BxDFReflTransFlags::Unset) return false;

    bool matches_reflection = has_flag(flags, BxDFReflTransFlags::Reflection) &&
                              has_flag(type, BxDFFlags::Reflection);
    bool matches_transmission = has_flag(flags, BxDFReflTransFlags::Transmission) &&
                                has_flag(type, BxDFFlags::Transmission);
    return matches_reflection || matches_transmission;
}

template<typename T, int N>
struct BxDFSampleRecord {
    radiometry::SampledSpectrum<T, N> f; // BxDF值
    math::Vector<T, 3> wi; // 入射方向
    T pdf = 0; // 采样概率密度
    T eta = 1; // 折射率比值
    BxDFFlags sampled_flags = BxDFFlags::Unset; // 记录采样到的具体类型
};

template<typename Derived, typename T, int N>
class BxDF {
public:
    Derived& as_derived() { return static_cast<Derived&>(*this); }
    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }
    BxDFFlags type() const { return as_derived().type_impl(); }

    T rou_hh() {
        return T(0);
    }

    T rou_hd() {
        return T(0);
    }

    radiometry::SampledSpectrum<T, N> f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi,
        TransportMode mode
    ) const {
        return as_derived().f_impl(swl, wo, wi, mode);
    }

    std::optional<BxDFSampleRecord<T, N>> sample_f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T uc,
        const math::Point<T, 2>& u2d,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        return as_derived().sample_f_impl(swl, wo, uc, u2d, mode, sample_flags);
    }

    T pdf(
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi, 
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        return as_derived().pdf_impl(wo, wi, mode, sample_flags); 
    }
};


} // namespace pbpt::material
