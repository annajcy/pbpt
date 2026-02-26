#pragma once
#include "pbpt/light_sampler/light_sampler.hpp"
#include <vector>
#include <span>
#include <algorithm>

namespace pbpt::light_sampler {

template <typename T>
class UniformLightSampler : public LightSampler<UniformLightSampler<T>, T> {
    friend class LightSampler<UniformLightSampler<T>, T>;

private:
    std::vector<const light::AnyLight<T>*> m_lights;

public:
    UniformLightSampler() = default;

    explicit UniformLightSampler(std::span<const light::AnyLight<T>*> lights)
        : m_lights(lights.begin(), lights.end()) {}

private:
    LightSamplerSampleResult<T> sample_impl(T u) const {
        if (m_lights.empty()) {
            return {nullptr, T(0)};
        }
        int count = static_cast<int>(m_lights.size());
        int idx = std::min(static_cast<int>(u * count), count - 1);
        return {m_lights[idx], T(1) / T(count)};
    }

    T pdf_impl(int /*light_index*/) const {
        if (m_lights.empty())
            return T(0);
        return T(1) / T(m_lights.size());
    }

    int light_count_impl() const { return static_cast<int>(m_lights.size()); }
};

}  // namespace pbpt::light_sampler
