#pragma once

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "geometry/frame.hpp"
#include "bxdf_type.hpp"
#include "sampler/discrete.hpp"

namespace pbpt::material {

template<typename T, int N>
class BSDF {
private: 
    geometry::Frame<T> m_shading_frame;
    geometry::Frame<T> m_geometric_frame;
    std::vector<AnyBxDF<T, N>> m_bxdfs;

public:
    BSDF(
        const geometry::Frame<T>& shading_frame,
        const geometry::Frame<T>& geometric_frame,
        const std::vector<AnyBxDF<T, N>>& bxdfs
    ) : m_shading_frame(shading_frame), 
        m_geometric_frame(geometric_frame),
        m_bxdfs(bxdfs) {}

    // 几何检查：是否在物理几何的同一侧
    bool is_same_hemisphere(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi
    ) const {
        T cos_theta_o = m_geometric_frame.n().dot(wo);
        T cos_theta_i = m_geometric_frame.n().dot(wi);
        return (cos_theta_o * cos_theta_i) > T(0);
    }

    // 计算权重的辅助函数 (统一逻辑)
    // 返回 total_weight
    T compute_effective_weights(
        const BxDFTypeFlags flags,
        const std::vector<T>& custom_weights,
        std::vector<T>& effective_weights_out
    ) const {
        const bool has_custom = !custom_weights.empty() && custom_weights.size() == m_bxdfs.size();
        effective_weights_out.assign(m_bxdfs.size(), T(0));
        T total_weight = 0;

        for (size_t i = 0; i < m_bxdfs.size(); ++i) {
            bool flag_match = false;
            std::visit([&](const auto& bxdf) {
                if (bxdf.is_flags_matched(flags)) flag_match = true;
            }, m_bxdfs[i]);

            if (!flag_match) {
                effective_weights_out[i] = T(0);
                continue;
            }

            const T w = has_custom ? custom_weights[i] : T(1);
            
            effective_weights_out[i] = w;
            total_weight += w;
        }
        return total_weight;
    }

    // Evaluate f()
    radiometry::SampledSpectrum<T, N> f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo, 
        const math::Vector<T, 3>& wi,
        const BxDFTypeFlags flags
    ) const {
        radiometry::SampledSpectrum<T, N> result = radiometry::SampledSpectrum<T, N>::filled(0);
        auto wo_local = m_shading_frame.to_local(wo);
        auto wi_local = m_shading_frame.to_local(wi);
        bool is_geo_same = this->is_same_hemisphere(wo, wi);

        for (const auto& bxdf : m_bxdfs) {
            std::visit([&](const auto& concrete_bxdf) {
                if (!concrete_bxdf.is_flags_matched(flags)) return;

                bool type_reflect = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Reflection);
                bool type_transmit = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Transmission);
                
                // 防漏光检查
                if ((is_geo_same && type_reflect) || (!is_geo_same && type_transmit)) {
                    result += concrete_bxdf.f(swl, wo_local, wi_local);
                }
            }, bxdf);
        }
        return result;
    }

    // Evaluate PDF()
    T pdf(
        const math::Vector<T, 3>& wo,
        const math::Vector<T, 3>& wi,
        const BxDFTypeFlags flags,
        const std::vector<T>& custom_weights = {}
    ) const {
        if (m_bxdfs.empty()) return 0;

        auto wo_local = m_shading_frame.to_local(wo);
        auto wi_local = m_shading_frame.to_local(wi);
        bool is_geo_same = this->is_same_hemisphere(wo, wi);

        std::vector<T> weights;
        T total_weight = compute_effective_weights(flags, custom_weights, weights);
        
        if (total_weight == 0) return 0;
        T inv_total_weight = T(1) / total_weight;

        T final_pdf = 0;

        for (size_t i = 0; i < m_bxdfs.size(); ++i) {
            if (weights[i] == 0) continue;

            std::visit([&](const auto& concrete_bxdf) {
                bool type_reflect = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Reflection);
                bool type_transmit = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Transmission);
                
                if ((is_geo_same && type_reflect) || (!is_geo_same && type_transmit)) {
                    T prob = weights[i] * inv_total_weight;
                    final_pdf += prob * concrete_bxdf.pdf(wo_local, wi_local);
                }
            }, m_bxdfs[i]);
        }
        return final_pdf;
    }

    // Sample BSDF
    BxDFSampleRecord<T, N> sample_f(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample,
        const BxDFTypeFlags flags = BxDFTypeFlags::ANY,
        const std::vector<T>& custom_weights = {}
    ) const {
        BxDFSampleRecord<T, N> result; // valid=false initially
        if (m_bxdfs.empty()) return result;

        // 1. 计算权重
        std::vector<T> weights;
        // 注意：这里不需要 filtered_bxdfs，我们直接用 weights 的索引访问 m_bxdfs
        compute_effective_weights(flags, custom_weights, weights);

        std::vector<T> cdf_buffer;
        auto selection = sampler::sample_discrete(weights, cdf_buffer, u_sample.x());
        if (selection.index == -1) return result; // 采样失败（可能 flags 过滤后为空）

        // 2. 采样选中的 BxDF
        std::visit([&](const auto& concrete_bxdf) {
            auto wo_local = m_shading_frame.to_local(wo);
            
            // 使用重映射后的 u
            math::Point<T, 2> remapped_sample(selection.u_remapped, u_sample.y());
            
            // 调用 BxDF 的 sample_f
            auto bxdf_sample = concrete_bxdf.sample_f(swl, wo_local, remapped_sample);
            
            if (!bxdf_sample.valid) return; // 子采样失败

            auto wi_world = m_shading_frame.to_render(bxdf_sample.wi);
            bool is_geo_same = this->is_same_hemisphere(wo, wi_world);

            bool type_reflect = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Reflection);
            bool type_transmit = concrete_bxdf.is_flags_matched(BxDFTypeFlags::Transmission);

            // 几何一致性检查
            if (!((is_geo_same && type_reflect) || (!is_geo_same && type_transmit))) {
                return;
            }

            result.wi = wi_world;
            result.sampled_type = bxdf_sample.sampled_type; // 从子采样获取具体类型

            // 3. Specular 处理 (重要修复)
            if (has_flag(result.sampled_type, BxDFTypeFlags::Specular)) {
                // 如果是镜面反射，不进行 MIS 混合
                result.f = bxdf_sample.f;
                result.pdf = selection.pdf * bxdf_sample.pdf;
                result.valid = true;
                return;
            }

            // 4. Non-Specular 处理 (MIS 混合)
            // 重新计算混合 PDF (必须传入 custom_weights 以保持一致性)
            result.pdf = this->pdf(wo, wi_world, flags, custom_weights);

            if (result.pdf > 0) {
                // 重新计算混合 f (累加所有 component 的贡献)
                result.f = this->f(swl, wo, wi_world, flags);
                result.valid = true;
            } else {
                result.valid = false;
            }

        }, m_bxdfs[selection.index]); // 使用索引访问原始 m_bxdfs

        return result;
    }
};

} // namespace pbpt::material
