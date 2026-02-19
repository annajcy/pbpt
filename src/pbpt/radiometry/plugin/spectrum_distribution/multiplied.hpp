#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "pbpt/radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Small holder that either owns or references an object.
 *
 * Used to implement spectrum products that can accept both lvalues and
 * rvalues without copying unnecessarily.
 */
template <class S>
struct Hold {
    /// Owned instance when the original object was an rvalue.
    std::shared_ptr<S> owned;
    /// Pointer to the referenced instance when the original object was an lvalue.
    const S* ref = nullptr;

    /// Constructs a holder that references an existing object without taking ownership.
    Hold(const S& s) : owned(), ref(&s) {}

    /// Constructs a holder that takes ownership of a temporary object.
    Hold(S&& s) : owned(std::make_shared<S>(std::move(s))), ref(owned.get()) {}

    /// Returns a const reference to the held object, regardless of ownership mode.
    const S& get() const { return *ref; }
};

/**
 * @brief Spectrum representing the product of two spectra.
 *
 * Values are evaluated lazily as s1(lambda) * s2(lambda).
 */
template <typename T, class D1, class D2>
class MultipliedSpectrumDistribution : public SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T> {
    friend class SpectrumDistribution<MultipliedSpectrumDistribution<T, D1, D2>, T>;

public:
    /// First factor in the product spectrum.
    Hold<D1> s1;
    /// Second factor in the product spectrum.
    Hold<D2> s2;

public:
    /// Constructs from two lvalue spectra (referenced without copying).
    MultipliedSpectrumDistribution(const D1& a, const D2& b) : s1(a), s2(b) {}
    /// Constructs from a temporary first spectrum and an lvalue second spectrum.
    MultipliedSpectrumDistribution(D1&& a, const D2& b) : s1(std::move(a)), s2(b) {}
    /// Constructs from an lvalue first spectrum and a temporary second spectrum.
    MultipliedSpectrumDistribution(const D1& a, D2&& b) : s1(a), s2(std::move(b)) {}
    /// Constructs from two temporary spectra, taking ownership of both.
    MultipliedSpectrumDistribution(D1&& a, D2&& b) : s1(std::move(a)), s2(std::move(b)) {}

    /**
     * @brief Evaluates the product spectrum at a wavelength.
     *
     * Simply multiplies the values of the two underlying spectra at
     * the same wavelength.
     */
    constexpr T at_impl(T lambda) const { return s1.get().at(lambda) * s2.get().at(lambda); }
};

/// Multiply two @c SpectrumDistribution objects to get a spectrum product.
template <typename D1, typename T1, typename D2, typename T2>
inline auto operator*(const SpectrumDistribution<D1, T1>& s1, const SpectrumDistribution<D2, T2>& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(static_cast<const D1&>(s1), static_cast<const D2&>(s2));
}

/// Overload for rvalue-lvalue multiplication to avoid dangling references.
template <typename D1, typename T1, typename D2, typename T2>
inline auto operator*(SpectrumDistribution<D1, T1>&& s1, const SpectrumDistribution<D2, T2>& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(std::move(static_cast<D1&>(s1)), static_cast<const D2&>(s2));
}

/// Overload for lvalue-rvalue multiplication to avoid dangling references.
template <typename D1, typename T1, typename D2, typename T2>
inline auto operator*(const SpectrumDistribution<D1, T1>& s1, SpectrumDistribution<D2, T2>&& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(static_cast<const D1&>(s1), std::move(static_cast<D2&>(s2)));
}

/// Overload for rvalue-rvalue multiplication to avoid dangling references.
template <typename D1, typename T1, typename D2, typename T2>
inline auto operator*(SpectrumDistribution<D1, T1>&& s1, SpectrumDistribution<D2, T2>&& s2) {
    using R = std::common_type_t<T1, T2>;
    return MultipliedSpectrumDistribution<R, D1, D2>(std::move(static_cast<D1&>(s1)), std::move(static_cast<D2&>(s2)));
}

}  // namespace pbpt::radiometry
