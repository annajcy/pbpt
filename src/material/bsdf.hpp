#pragma once

#include <optional>
#include <utility>

#include "geometry/frame.hpp"
#include "material/plugin/bxdf/bxdf_type.hpp"

namespace pbpt::material {


template<typename T>
inline geometry::Frame<T> build_shading_frame(
    const geometry::SurfaceInteraction<T>& si,
    const geometry::ShadingInfo<T>& shading
) {
    auto shading_n = shading.n;
    if (shading_n.to_vector().dot(si.n().to_vector()) < 0) {
        shading_n = -shading_n;
    }
    const auto dpdu = si.dpdu();
    if (dpdu.length_squared() > 0) {
        return geometry::Frame<T>(dpdu, shading_n.to_vector());
    }
    return geometry::Frame<T>(shading_n.to_vector());
}

template<typename T, int N>
class BSDF {
private: 
    geometry::Frame<T> m_frame;
    AnyBxDF<T, N> m_bxdf;

public:
    template<typename BxDFType>
    BSDF(const geometry::SurfaceInteraction<T>& si,
        const geometry::ShadingInfo<T>& shading,
        BxDFType&& bxdf
    ) : m_frame(build_shading_frame(si, shading)),
        m_bxdf(AnyBxDF<T, N>(std::move(bxdf))) {}

    BSDF(const geometry::Frame<T>& shading_frame,
        AnyBxDF<T, N> bxdf
    ) : m_frame(shading_frame),
        m_bxdf(std::move(bxdf)) {}

    math::Vector<T, 3> render_to_local(const math::Vector<T, 3>& w) const {
        return m_frame.to_local(w);
    }

    math::Vector<T, 3> local_to_render(const math::Vector<T, 3>& w) const {
        return m_frame.to_render(w);
    }

    // Evaluate f()
    radiometry::SampledSpectrum<T, N> f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi,
        TransportMode mode,
        const BxDFTypeFlags flags = BxDFTypeFlags::ALL
    ) const {
        auto wo_local = m_frame.to_local(wo);
        auto wi_local = m_frame.to_local(wi);
        return std::visit([&](const auto& bxdf) {
            return bxdf.f(swl, wo_local, wi_local, mode, flags);
        }, m_bxdf);
    }

    // Evaluate PDF()
    T pdf(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        auto wo_local = m_frame.to_local(wo);
        auto wi_local = m_frame.to_local(wi);
        return std::visit([&](const auto& bxdf) {
            return bxdf.pdf(wo_local, wi_local, mode, sample_flags);
        }, m_bxdf);
    }

    // Sample BSDF
    std::optional<BxDFSampleRecord<T, N>> sample_f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const T uc,
        const math::Point<T, 2>& u2,
        TransportMode mode,
        const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All
    ) const {
        auto wo_local = m_frame.to_local(wo);
        auto bxdf_sample = std::visit([&](const auto& bxdf) {
            return bxdf.sample_f(swl, wo_local, uc, u2, mode, sample_flags);
        }, m_bxdf);
        if (!bxdf_sample) return std::nullopt;

        bxdf_sample->wi = m_frame.to_render(bxdf_sample->wi);
        return bxdf_sample;
    }
};

} // namespace pbpt::material
