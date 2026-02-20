#pragma once

#include <variant>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"

namespace pbpt::integrator {

/// AnyIntegrator variant with N=4 fixed (spectral channels).
template <typename T>
using AnyIntegrator = std::variant<PathIntegrator<T, 4>>;

}  // namespace pbpt::integrator
