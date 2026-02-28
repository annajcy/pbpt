#pragma once

#include <variant>

#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/integrator/plugin/integrator/simple_path_integrator.hpp"

namespace pbpt::integrator {

/// AnyIntegrator variant with N=4 fixed (spectral channels).
template <typename T>
using AnyIntegrator = std::variant<
    SimplePathIntegrator<T, 4>, 
    PathIntegrator<T, 4>
>;

}  // namespace pbpt::integrator
