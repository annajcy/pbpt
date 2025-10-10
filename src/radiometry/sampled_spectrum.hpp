#pragma once

#include "math/vector.hpp"

namespace pbpt::radiometry {

template <typename T, int N>
class SampledSpectrum : public math::Vector<T, N> {
public:
    SampledSpectrum() : math::Vector<T, N>() {}
    SampledSpectrum(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

template<typename T, int N> 
class SampledWavelength : public math::Vector<T, N> {
public:
    SampledWavelength() : math::Vector<T, N>() {}
    SampledWavelength(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

template<typename T, int N>
class SampledPdf : public math::Vector<T, N> {
public:
    SampledPdf() : math::Vector<T, N>() {}
    SampledPdf(const math::Vector<T, N>& vec) : math::Vector<T, N>(vec) {}
};

};