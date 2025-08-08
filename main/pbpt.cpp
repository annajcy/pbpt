#include <iostream>
#include <memory>

#include "math/geometry/point.hpp"
#include "math/global/type_alias.hpp"
#include "math/integration/distribution.hpp"
#include "math/integration/integrator.hpp"
#include "math/integration/sampler.hpp"

using namespace pbpt;

int main() {
    auto mc_integrator = std::make_shared<math::MonteCarloIntegrator1D>();

    auto res = mc_integrator->estimate([](const math::Pt1& p) { return p.x(); },
                                       std::make_shared<math::UniformDistribution1D>(math::Pt1{0.0}, math::Pt1{2.0}),
                                       std::make_shared<math::UniformSampler<1>>(), 10000);

    std::cout << res << std::endl;

    int  sample_count      = 100;
    auto sampler_evaluator = std::make_shared<math::SamplerEvaluator<1>>();
    auto uni_result = sampler_evaluator->evaluate(std::make_shared<math::UniformSampler<1>>()->generate(sample_count));

    std::cout << uni_result.mean << std::endl;
    std::cout << uni_result.variance << std::endl;

    int  strat_layer = 5;
    auto strat_result =
        sampler_evaluator->evaluate(std::make_shared<math::StratifiedSampler<1>>(strat_layer)->generate(sample_count));

    std::cout << strat_result.mean << std::endl;
    std::cout << strat_result.variance << std::endl;

    int                                      mean_sample_count = 10;
    std::vector<math::Point<math::Float, 1>> points;
    points.reserve(mean_sample_count);
    for (int i = 0; i < mean_sample_count; i++) {
        auto pts  = std::make_shared<math::UniformSampler<1>>()->generate(10);
        auto mean = math::Pt1::mid(pts);
        points.push_back(mean);
    }

    auto eval = sampler_evaluator->evaluate(points);
    std::cout << eval.mean << std::endl;
    std::cout << eval.variance << std::endl;

    points.clear();
    points.reserve(mean_sample_count);

    for (int i = 0; i < mean_sample_count; i++) {
        auto pts  = std::make_shared<math::StratifiedSampler<1>>(5)->generate(10);
        auto mean = math::Pt1::mid(pts);
        points.push_back(mean);
    }

    auto eval_ = sampler_evaluator->evaluate(points);
    std::cout << eval_.mean << std::endl;
    std::cout << eval_.variance << std::endl;

    return 0;
}