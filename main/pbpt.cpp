#include "math/geometry/point.hpp"
#include "math/integration/distribution.hpp"
#include "math/integration/intergrator.hpp"
#include "math/integration/sampler.hpp"
#include <iostream>
#include <memory>

using namespace pbpt;

int main() {
    auto mc_integrator = std::make_shared<math::MonteCarloIntegrator1D>();

    auto res = mc_integrator->estimate(
        [](const math::Pt1& p) {
            return p.x();
        }, 
        std::make_shared<math::UniformDistribution1D>(math::Pt1{0.0}, math::Pt1{2.0}), 
        std::make_shared<math::UniformSampler<1>>(), 
        10000
    );

    std::cout << res << std::endl;
    return 0;
}