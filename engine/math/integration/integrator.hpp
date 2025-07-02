#pragma once

#include "math/global/type_alias.hpp"
#include "math/integration/distribution.hpp"
#include "math/integration/sampler.hpp"
#include <functional>
#include <memory>
#include <stdexcept>
#include <cmath>

namespace pbpt::math {

/**
 * @brief Base class for numerical integrators
 * @tparam N Dimension of the integration domain
 */
template<int N>
class Integrator {
public:
    virtual ~Integrator() = default;
    
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
        // Parameter validation
        if (sample_count <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }
        if (!f || !distribution || !sampler) {
            throw std::invalid_argument("Function, distribution, and sampler must not be null");
        }
        
        Float sum = 0.0;
        auto samples = sampler->generate(sample_count);
        
        for (int i = 0; i < sample_count; ++i) {
            auto u = samples[i];
            const Point<Float, 1> p = distribution->sample(u);
            Float pdf_value = distribution->pdf(p);
            
            if (std::abs(pdf_value) < 1e-10) {
                continue;
            }
            
            Float function_value = f(p);
            if (std::isfinite(function_value)) {
                sum += function_value / pdf_value;
            }
        }
        
        return sum / sample_count;
    }
};

template<int N>
class MonteCarloIntegratorND : public Integrator<N> {
public:
    Float estimate(
        const std::function<Float(const Point<Float, N>&)>& f, 
        const std::shared_ptr<Distribution<N>>& distribution,
        const std::shared_ptr<Sampler<N>>& sampler,
        int sample_count
    ) const override {

        if (sample_count <= 0) {
            throw std::invalid_argument("Sample count must be positive");
        }
        if (!f || !distribution || !sampler) {
            throw std::invalid_argument("Function, distribution, and sampler must not be null");
        }
        
        Float sum = 0.0;
        auto samples = sampler->generate(sample_count);
        
        for (int i = 0; i < sample_count; ++i) {
            auto u = samples[i];
            const Point<Float, N> p = distribution->sample(u);
            Float pdf_value = distribution->pdf(p);
            
            if (std::abs(pdf_value) < 1e-10) {
                continue; 
            }
            
            Float function_value = f(p);
            if (std::isfinite(function_value)) {
                sum += function_value / pdf_value;
            }
        }
        
        return sum / sample_count;
    }
};

}