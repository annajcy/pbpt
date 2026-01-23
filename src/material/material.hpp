#pragma once

#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "geometry/frame.hpp"
#include "geometry/interaction.hpp"
#include "material/bsdf.hpp"
#include "radiometry/sampled_spectrum.hpp"
#include "radiometry/spectrum_distribution.hpp"

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
        const radiometry::SampledWavelength<T, N>& wavelengths
    ) const {
        return as_derived().template compute_bsdf_impl<N>(si, shading, wavelengths);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

}  // namespace pbpt::material
