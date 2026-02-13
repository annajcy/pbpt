#pragma once

#include "pbpt/radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Albedo (reflectance) spectrum derived from an RGB model.
 *
 * Wraps an underlying parametric RGB spectrum (such as
 * RGBSigmoidPolynomialNormalized) and exposes it as a generic
 * SpectrumDistribution.
 */
template <typename T, template<typename> class RSPType>
class RGBAlbedoSpectrumDistribution : public SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBAlbedoSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;

public:
    /// Constructs an albedo spectrum from an underlying RGB basis spectrum.
    RGBAlbedoSpectrumDistribution(const RSPType<T>& rsp) : m_rsp(rsp) {}

    /// Returns a const reference to the underlying RGB spectrum model.
    const RSPType<T>& rsp() const { return m_rsp; }
    /// Returns a mutable reference to the underlying RGB spectrum model.
    RSPType<T>& rsp() { return m_rsp; }

private:
    T at_impl(T lambda) const {
        return m_rsp.at(lambda);
    }
};

/**
 * @brief Unbounded RGB spectrum with a global scale factor.
 *
 * Similar to RGBAlbedoSpectrumDistribution, but allows scaling the
 * underlying spectrum by an arbitrary factor.
 */
template<typename T, template<typename> class RSPType>
class RGBUnboundedSpectrumDistribution : public SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>{
    friend class SpectrumDistribution<RGBUnboundedSpectrumDistribution<T, RSPType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;

public:
    /**
     * @brief Constructs an unbounded spectrum from an RGB basis and scale.
     *
     * @param rsp    Underlying RGB basis spectrum.
     * @param m_scale Global multiplicative factor applied to the spectrum.
     */
    RGBUnboundedSpectrumDistribution(const RSPType<T>& rsp, T m_scale) : m_rsp(rsp), m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda);
    }
};

/**
 * @brief Illuminant spectrum constructed from an RGB basis and reference illuminant.
 *
 * The spectrum is the product of a parametric RGB spectrum and a
 * reference illuminant spectrum, optionally scaled by m_scale.
 */
template<typename T, template<typename> class RSPType, typename IlluminantSpectrumType>
class RGBIlluminantSpectrumDistribution : public SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, IlluminantSpectrumType>, T>{
    friend class SpectrumDistribution<RGBIlluminantSpectrumDistribution<T, RSPType, IlluminantSpectrumType>, T>;

private:
    RSPType<T> m_rsp;
    T m_scale;
    IlluminantSpectrumType m_reference_luminant;

public:
    /**
     * @brief Constructs an illuminant spectrum from an RGB basis and reference illuminant.
     *
     * @param rsp                 Underlying RGB basis spectrum.
     * @param reference_luminant  Reference illuminant spectrum to modulate.
     * @param m_scale             Optional global scale factor.
     */
    RGBIlluminantSpectrumDistribution(const RSPType<T>& rsp, const IlluminantSpectrumType& reference_luminant, T m_scale = T{1.0}) 
        : m_rsp(rsp), m_reference_luminant(reference_luminant) , m_scale(m_scale) {}

private:
    T at_impl(T lambda) const {
        return m_scale * m_rsp.at(lambda) * m_reference_luminant.at(lambda);
    }
};

}  // namespace pbpt::radiometry
