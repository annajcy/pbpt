#pragma once

#include "integrator/integrator.hpp"
namespace pbpt::integrator {

template<typename T, int N, typename SceneType>
class PathIntegrator : public Integrator<PathIntegrator<T, N, SceneType>, T, N, SceneType> {
    friend class Integrator<PathIntegrator<T, N, SceneType>, T, N, SceneType>;
private:
    int m_ssp = 4;
    int m_max_depth = -1;
    T m_rr_threshold = T(0.1);
    std::string m_output_path = "output.exr";

public:
    PathIntegrator(
        int ssp, int max_depth, T rr_threshold, 
        const std::string& output_path = "output.exr"
    ): m_ssp(ssp), m_max_depth(max_depth), m_rr_threshold(rr_threshold) {}

    int ssp() const { return m_ssp; }
    int max_depth() const { return m_max_depth; }
    T rr_threshold() const { return m_rr_threshold; }
};

}