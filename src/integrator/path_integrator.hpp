#pragma once

#include "integrator/integrator.hpp"
namespace pbpt::integrator {

template<typename T, int N, typename SceneType>
class PathIntegrator : public Integrator<PathIntegrator<T, N, SceneType>, T, N, SceneType> {
    friend class Integrator<PathIntegrator<T, N, SceneType>, T, N, SceneType>;
private:
    int m_ssp = 4;
    int m_max_depth = -1;

public:
    int ssp() const { return m_ssp; }
    int max_depth() const { return m_max_depth; }
    
};

}