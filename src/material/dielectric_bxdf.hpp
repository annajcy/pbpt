#pragma once

#include "bxdf.hpp"
#include "utils.hpp"

namespace pbpt::material {

template<typename T, int N>
class DielectricBxDF : public BxDF<DielectricBxDF<T, N>, T, N> {
    friend class BxDF<DielectricBxDF<T, N>, T, N>;

private:
    T m_eta; // 材质的折射率 (例如玻璃 1.5)
    radiometry::SampledSpectrum<T, N> m_tint_refl;  // 反射颜色 (通常是纯白)
    radiometry::SampledSpectrum<T, N> m_tint_trans; // 透射颜色 (即玻璃颜色)

public:
    // 构造函数
    DielectricBxDF(T eta, 
                   radiometry::SampledSpectrum<T, N> tint_r = radiometry::SampledSpectrum<T, N>::filled(1),
                   radiometry::SampledSpectrum<T, N> tint_t = radiometry::SampledSpectrum<T, N>::filled(1))
        : m_eta(eta), m_tint_refl(tint_r), m_tint_trans(tint_t) {}

private:
    // 1. 类型：既是反射，又是透射，且都是 Specular
    BxDFTypeFlags type_impl() const {
        return BxDFTypeFlags::SpecularReflection | BxDFTypeFlags::SpecularTransmission;
    }

    // 2. f()：Delta 分布在任何非精确方向的概率密度为 0，值为 0
    radiometry::SampledSpectrum<T, N> f_impl(
        const radiometry::SampledWavelength<T, N>&,
        const math::Vector<T, 3>&, 
        const math::Vector<T, 3>&
    ) const {
        return radiometry::SampledSpectrum<T, N>::filled(0);
    }

    // 3. pdf()：Delta 分布的 PDF 为 0
    T pdf_impl(const math::Vector<T, 3>&, const math::Vector<T, 3>&) const {
        return 0;
    }

    // 4. sample_f()：核心逻辑
    BxDFSampleRecord<T, N> sample_f_impl(
        const radiometry::SampledWavelength<T, N>& swl,
        const math::Vector<T, 3>& wo,
        const math::Point<T, 2>& u_sample
    ) const {
        BxDFSampleRecord<T, N> record;
        record.is_valid = false;

        // 计算菲涅尔反射比 F
        // wo.z() 在局部空间即 cos_theta_i
        T F = material::fresnel_dielectric_eval(wo.z(), m_eta);

        // --- 随机决策：反射还是折射？ ---
        // 我们利用 u_sample.x() 来做这个决策
        // 这是一个完美的 Importance Sampling：根据能量占比 F 来决定概率
        
        if (u_sample.x() < F) {
            // === 走反射分支 (Reflection) ===
            
            // 计算反射方向
            record.wi = math::Vector<T, 3>(-wo.x(), -wo.y(), wo.z());
            
            // 设置类型
            record.sampled_type = BxDFTypeFlags::SpecularReflection;
            
            // 概率 PDF
            // 我们选反射的概率是 F。
            // 注意：因为是 Delta 分布，BSDF::sample_f 里的 pdf 实际上是 "Selection Probability" * "Discrete Probability(1.0)"
            record.pdf = F; 
            
            // 计算 f (Throughput / |cos|)
            // f = Energy / |cos(wi)|
            // Energy = Input * F * Tint
            // 所以 f = F * Tint / |wi.z|
            // 在路径追踪中，我们通常直接返回 throughput weight，但为了符合 f 的定义：
            record.f = m_tint_refl * F / std::abs(record.wi.z());
            
            record.is_valid = true;
        } else {
            // === 走折射分支 (Refraction) ===
            
            // 准备折射所需的 eta 比率
            bool entering = wo.z() > 0;
            T eta_i = entering ? 1.f : m_eta;
            T eta_t = entering ? m_eta : 1.f;
            T eta_ratio = eta_i / eta_t; // Snell's law ratio

            math::Vector<T, 3> wi;
            // 计算折射方向，如果发生全反射 (refract返回false)，则前面 F 应该已经是 1.0 了
            // 所以理论上这里 u < F 应该已经进了反射分支。
            // 加上 check 以防万一浮点误差
            if (!material::refract(wo, wi, eta_ratio)) {
                 return record; // Should not happen if F is correct
            }
            
            record.wi = wi;
            record.sampled_type = BxDFTypeFlags::SpecularTransmission;
            record.pdf = 1 - F; // 选择折射的概率
            
            // 计算 f (Radiance Throughput)
            // 这里的物理有点深：
            // 当光线穿过不同介质时，Radiance 会按折射率平方缩放 (L_o = L_i * (eta_t^2 / eta_i^2))
            // 但是！很多渲染器（包括 PBRT）倾向于在 BxDF 内部处理这个缩放，
            // 使得 integrator 只需要乘 f * cos / pdf。
            
            // 传输系数 ft = (1-F) * Tint
            // Radiance scaling factor = (eta_t^2 / eta_i^2) ? 
            // 注意：Radiance 是 flux per unit projected area per unit solid angle.
            // 立体角压缩了，所以 Radiance 变了。
            // 对于 Basic Path Tracing，如果不做复杂的双向追踪，我们可以简化理解：
            // 我们返回的值 乘以 cos / pdf 应该等于 throughput。
            
            // Throughput = (1-F) * Tint / (eta_ratio^2) ?? 
            // 这是一个经典的坑。
            // 正确的 Radiance Transport (PBRT v3/v4):
            // f = (1-F) * Tint * (1 / eta_ratio^2) / |cos_theta_t|
            // 为什么是 1/eta^2 ? 因为 radiance 在更密的介质里更集中。
            // 当从空气进入玻璃 (eta_ratio = 1/1.5)，eta_ratio < 1, 1/eta^2 > 1。Radiance 增加。
            
            T ft = T(1) - F;
            
            // 修正系数 (Radiance Scaling)
            // 如果不需要 radiance scaling (例如只做颜色计算)，可以去掉 eta_factor
            T eta_factor = (eta_t * eta_t) / (eta_i * eta_i); // eta_t^2 / eta_i^2
            
            // 组合：
            // f = Energy / |cos|
            // Energy = ft * Tint * eta_factor
            record.f = m_tint_trans * ft * eta_factor / std::abs(record.wi.z());
            
            record.is_valid = true;
        }
        
        return record;
    }
};
}