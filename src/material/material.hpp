#pragma once

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
        const radiometry::SampledWavelength<T, N>& wavelengths
    ) const {
        return as_derived().template compute_bsdf_impl<N>(si, wavelengths);
    }

    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
};

/**
 * @brief Convenience helpers for building BSDF frames and libraries of materials.
 */
namespace detail {
template<typename T>
inline geometry::Frame<T> build_shading_frame(const geometry::SurfaceInteraction<T>& si) {
    const auto shading_n = si.shading_n().to_vector().normalized();
    const auto dpdu = si.shading_dpdu();
    if (dpdu.length_squared() > 0) {
        return geometry::Frame<T>(dpdu, shading_n);
    }
    return geometry::Frame<T>(shading_n);
}
}  // namespace detail

/**
 * @brief Construct a BSDF from a surface interaction and a set of lobes.
 */
template<typename T, int N>
inline BSDF<T, N> make_bsdf(
    const geometry::SurfaceInteraction<T>& si,
    std::vector<AnyBxDF<T, N>> bxdfs
) {
    geometry::Frame<T> shading_frame = detail::build_shading_frame(si);
    geometry::Frame<T> geometric_frame(si.n().to_vector());
    return BSDF<T, N>(shading_frame, geometric_frame, std::move(bxdfs));
}

/**
 * @brief Diffuse (Lambertian) material implementation.
 */
template<typename T>
class LambertianMaterial : public Material<LambertianMaterial<T>, T> {
private:
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_albedo_dist;

public:
    template<typename SpectrumDistributionType>
    explicit LambertianMaterial(const SpectrumDistributionType& albedo_distribution)
        : m_albedo_dist(albedo_distribution) {}

    template<int N>
    BSDF<T, N> compute_bsdf_impl(
        const geometry::SurfaceInteraction<T>& si,
        const radiometry::SampledWavelength<T, N>& wavelengths
    ) const {
        auto albedo = m_albedo_dist.sample<N>(wavelengths);
        std::vector<AnyBxDF<T, N>> lobes;
        lobes.emplace_back(LambertianBxDF<T, N>(albedo));
        return make_bsdf(si, std::move(lobes));
    }
};

/**
 * @brief Specular dielectric material (glass-like).
 */
template<typename T>
class DielectricMaterial : public Material<DielectricMaterial<T>, T> {
private:
    T m_eta;
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_tint_refl_dist;
    radiometry::PiecewiseLinearSpectrumDistribution<T> m_tint_trans_dist;

public:
    // Constructor with IOR only (white tints)
    explicit DielectricMaterial(T eta)
        : m_eta(eta), 
          m_tint_refl_dist(radiometry::ConstantSpectrumDistribution<T>(T(1))),
          m_tint_trans_dist(radiometry::ConstantSpectrumDistribution<T>(T(1))) {}

    // Constructor with IOR and spectrum distributions
    template<typename SpectrumDistributionTypeR, typename SpectrumDistributionTypeT>
    DielectricMaterial(
        T eta,
        const SpectrumDistributionTypeR& tint_r,
        const SpectrumDistributionTypeT& tint_t)
        : m_eta(eta), m_tint_refl_dist(tint_r), m_tint_trans_dist(tint_t) {}

    template<int N>
    BSDF<T, N> compute_bsdf_impl(
        const geometry::SurfaceInteraction<T>& si,
        const radiometry::SampledWavelength<T, N>& wavelengths
    ) const {
        auto tint_refl = m_tint_refl_dist.sample<N>(wavelengths);
        auto tint_trans = m_tint_trans_dist.sample<N>(wavelengths);
        std::vector<AnyBxDF<T, N>> lobes;
        lobes.emplace_back(DielectricBxDF<T, N>(m_eta, tint_refl, tint_trans));
        return make_bsdf(si, std::move(lobes));
    }
};

template<typename T>
using AnyMaterial = std::variant<
    LambertianMaterial<T>,
    DielectricMaterial<T>
>;

/**
 * @brief Simple material registry keyed by integer IDs.
 *
 * Stores concrete material variants and provides lookup helpers.
 */
template<typename T>
class MaterialLibrary {
private:
    std::vector<AnyMaterial<T>> m_materials;
public:

    template<typename Mat>
    int add_material(Mat material) {
        m_materials.emplace_back(std::move(material));
        return static_cast<int>(m_materials.size()) - 1;
    }

    const AnyMaterial<T>& get(int id) const {
        if (id < 0 || id >= static_cast<int>(m_materials.size())) 
            throw std::out_of_range("MaterialLibrary: Invalid material ID");
        return m_materials[static_cast<std::size_t>(id)];
    }

    AnyMaterial<T>& get(int id) {
        if (id < 0 || id >= static_cast<int>(m_materials.size())) 
            throw std::out_of_range("MaterialLibrary: Invalid material ID");
        return m_materials[static_cast<std::size_t>(id)];
    }

    std::size_t size() const { return m_materials.size(); }
};

}  // namespace pbpt::material
