#pragma once

#include "pbpt/scene/scene.hpp"
#include "pbpt/integrator/plugin/integrator/integrator_type.hpp"

namespace pbpt::serde {

template <typename T>
struct PbptXmlResult {
    pbpt::scene::Scene<T> scene;
    pbpt::integrator::AnyIntegrator<T> integrator;
    int spp{4};
};

}  // namespace pbpt::serde
