#pragma once

#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <optional>

#include "pbpt/geometry/frame.hpp"
#include "pbpt/geometry/interaction.hpp"
#include "pbpt/material/bsdf.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/radiometry/spectrum_distribution.hpp"

namespace pbpt::material {

/**
 * @brief CRTP helper that provides the public compute_bsdf entry without virtual dispatch.
 */
template<typename Derived, typename T>
class Material {
public:
    template<int N>
    BSDF<T, N> compute_bsdf(
        const geometry::SurfaceInteraction<T>& si,
        const geometry::ShadingInfo<T>& shading,
        const radiometry::SampledWavelength<T, N>& wavelengths,
        const std::optional<geometry::SurfaceDifferentials<T>>& differentials = std::nullopt
    ) const {
        return as_derived().template compute_bsdf_impl<N>(si, shading, wavelengths, differentials);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

}  // namespace pbpt::material
