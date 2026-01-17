#pragma once

#include "radiometry/sampled_spectrum.hpp"
namespace pbpt::integrator {

template<typename Derived, typename T, int N, typename SceneType>
class Integrator{
public:
    Derived& as_derived() {
        return static_cast<Derived&>(*this);
    }

    const Derived& as_derived() const {
        return static_cast<const Derived&>(*this);
    }
    
};

}