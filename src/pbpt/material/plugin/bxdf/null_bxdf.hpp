#pragma once

#include <cmath>
#include <optional>

#include "pbpt/geometry/spherical.hpp"
#include "pbpt/material/bxdf.hpp"
#include "pbpt/math/vector.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

template <typename T, int N>
class NullBxDF : public BxDF<NullBxDF<T, N>, T, N> {
    friend class BxDF<NullBxDF<T, N>, T, N>;

private:
    BxDFFlags type_impl() const { return BxDFFlags::SpecularTransmission; }

    radiometry::SampledSpectrum<T, N> f_impl(const radiometry::SampledWavelength<T, N>&, const math::Vector<T, 3>&,
                                             const math::Vector<T, 3>&, TransportMode) const {
        return radiometry::SampledSpectrum<T, N>::filled(0);
    }

    std::optional<BxDFSampleRecord<T, N>> sample_f_impl(
        const radiometry::SampledWavelength<T, N>&, const math::Vector<T, 3>& wo, const T, const math::Point<T, 2>&,
        TransportMode, const BxDFReflTransFlags sample_flags = BxDFReflTransFlags::All) const {
        if (!is_match_refl_trans(type_impl(), sample_flags)) {
            return std::nullopt;
        }

        // Null/transparent interface: straight-through (no deviation)
        math::Vector<T, 3> wi = -wo;
        auto abs_cos_theta = std::abs(geometry::cos_theta(wi));
        if (abs_cos_theta == T(0)) {
            return std::nullopt;
        }

        return BxDFSampleRecord<T, N>{.f = radiometry::SampledSpectrum<T, N>::filled(T(1) / abs_cos_theta),
                                      .wi = wi,
                                      .pdf = T(1),
                                      .eta = T(1),
                                      .sampled_flags = BxDFFlags::SpecularTransmission};
    }

    T pdf_impl(const math::Vector<T, 3>&, const math::Vector<T, 3>&, TransportMode,
               const BxDFReflTransFlags = BxDFReflTransFlags::All) const {
        return T(0);
    }
};

}  // namespace pbpt::material
