#pragma once

namespace pbpt::radiometry::constant {

/// Minimum wavelength (in nm) used by default tabular spectra.
template <typename T>
constexpr T lambda_min = T(360.0);

/// Maximum wavelength (in nm) used by default tabular spectra.
template <typename T>
constexpr T lambda_max = T(830.0);

}  // namespace pbpt::radiometry::constant