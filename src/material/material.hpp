#pragma once

#include <utility>
#include <variant>
#include <vector>

#include "geometry/frame.hpp"
#include "geometry/interaction.hpp"
#include "material/bsdf.hpp"
#include "radiometry/sampled_spectrum.hpp"

namespace pbpt::material {

/**
 * @brief CRTP helper that provides the public compute_bsdf entry without virtual dispatch.
 */
template<typename Derived, typename T, int N>
class Material {
public:
    BSDF<T, N> compute_bsdf(const geometry::SurfaceInteraction<T>& si) const {
        return as_derived().compute_bsdf_impl(si);
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
template<typename T, int N>
class LambertianMaterial : public Material<LambertianMaterial<T, N>, T, N> {
private:
    radiometry::SampledSpectrum<T, N> m_albedo;

public:
    explicit LambertianMaterial(const radiometry::SampledSpectrum<T, N>& albedo)
        : m_albedo(albedo) {}

    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si) const {
        std::vector<AnyBxDF<T, N>> lobes;
        lobes.emplace_back(LambertianBxDF<T, N>(m_albedo));
        return make_bsdf(si, std::move(lobes));
    }
};

/**
 * @brief Specular dielectric material (glass-like).
 */
template<typename T, int N>
class DielectricMaterial : public Material<DielectricMaterial<T, N>, T, N> {
private:
    T m_eta;
    radiometry::SampledSpectrum<T, N> m_tint_refl;
    radiometry::SampledSpectrum<T, N> m_tint_trans;

public:
    DielectricMaterial(
        T eta,
        radiometry::SampledSpectrum<T, N> tint_r = radiometry::SampledSpectrum<T, N>::filled(1),
        radiometry::SampledSpectrum<T, N> tint_t = radiometry::SampledSpectrum<T, N>::filled(1))
        : m_eta(eta), m_tint_refl(std::move(tint_r)), m_tint_trans(std::move(tint_t)) {}

    BSDF<T, N> compute_bsdf_impl(const geometry::SurfaceInteraction<T>& si) const {
        std::vector<AnyBxDF<T, N>> lobes;
        lobes.emplace_back(DielectricBxDF<T, N>(m_eta, m_tint_refl, m_tint_trans));
        return make_bsdf(si, std::move(lobes));
    }
};

template<typename T, int N>
using AnyMaterial = std::variant<
    LambertianMaterial<T, N>,
    DielectricMaterial<T, N>
>;

/**
 * @brief Simple material registry keyed by integer IDs.
 *
 * Stores concrete material variants and provides lookup helpers.
 */
template<typename T, int N>
class MaterialLibrary {
private:
    std::vector<AnyMaterial<T, N>> m_materials;
public:

    template<typename Mat>
    int add_material(Mat material) {
        m_materials.emplace_back(std::move(material));
        return static_cast<int>(m_materials.size()) - 1;
    }

    const AnyMaterial<T, N>& get(int id) const {
        if (id < 0 || id >= static_cast<int>(m_materials.size())) 
            throw std::out_of_range("MaterialLibrary: Invalid material ID");
        return m_materials[static_cast<std::size_t>(id)];
    }

    AnyMaterial<T, N>& get(int id) {
        if (id < 0 || id >= static_cast<int>(m_materials.size())) 
            throw std::out_of_range("MaterialLibrary: Invalid material ID");
        return m_materials[static_cast<std::size_t>(id)];
    }

    std::size_t size() const { return m_materials.size(); }
};

}  // namespace pbpt::material
