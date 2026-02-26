#pragma once
#include "pbpt/light/plugin/light/light_type.hpp"

namespace pbpt::light_sampler {

template <typename T>
struct LightSamplerSampleResult {
    const light::AnyLight<T>* light;  // 返回选中的光源指针，避免拷贝
    T selection_pdf;                  // 选择此光源的概率 (1/N for uniform)
};

template <typename Derived, typename T>
class LightSampler {
public:
    /// 给定随机数 u ∈ [0,1)，选择一个光源
    LightSamplerSampleResult<T> sample(T u) const { return as_derived().sample_impl(u); }

    /// 给定光源 ID，返回选择它的 PDF
    T pdf(int light_index) const { return as_derived().pdf_impl(light_index); }

    /// 场景中有多少光源
    int light_count() const { return as_derived().light_count_impl(); }

    Derived& as_derived() { return static_cast<Derived&>(*this); }
    const Derived& as_derived() const { return static_cast<const Derived&>(*this); }
};

}  // namespace pbpt::light_sampler
