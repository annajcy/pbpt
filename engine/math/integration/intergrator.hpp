#pragma once

#include "math/global/type_alias.hpp"
#include "math/integration/distribution.hpp"
#include "math/integration/sampler.hpp"
#include <functional>
#include <memory>

namespace pbpt::math {

template<int N>
class Integrator {
public:
    virtual Float estimate(
        const std::function<Float(const Point<Float, N>&)>& f, 
        const std::shared_ptr<Distribution<N>>& distribution,
        const std::shared_ptr<Sampler<N>>& sampler,
        int sample_count
    ) const = 0;
};

class MonteCarloIntegrator1D : public Integrator<1> {
public:
    Float estimate(
        const std::function<Float(const Point<Float, 1>&)>& f, 
        const std::shared_ptr<Distribution<1>>& distribution,
        const std::shared_ptr<Sampler<1>>& sampler,
        int sample_count
    ) const override {
        Float sum = 0.0;
        for (int i = 0; i < sample_count; ++i) {
            auto u = sampler->generate();
            const Point<Float, 1> p = distribution->sample(u);
            sum += f(p) / distribution->pdf(p);
        }
        return sum / sample_count;
    }
};

}