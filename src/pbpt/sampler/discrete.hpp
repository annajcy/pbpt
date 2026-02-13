#pragma once


#include <cstdint>
#include <array>
#include <optional>
#include <utility>
#include <vector>
#include <algorithm>
#include <limits>

#include <vector>
#include <optional>
#include <algorithm>
#include <tuple> // 用于返回三个值


namespace pbpt::sampler {

template<typename T>
struct DiscreteSampleResult {
    int index;
    T pdf;
    T u_remapped;
};

// 返回值：{ index, pdf, u_remapped }
template<typename T>
inline DiscreteSampleResult<T> sample_discrete(
    const std::vector<T>& weights,
    std::vector<T>& cdf_buffer,
    T u
) {
    if (weights.empty()) {
        return DiscreteSampleResult<T>{-1, T(0), T(0)};
    }

    int n = static_cast<int>(weights.size());

    auto& cdf = cdf_buffer;
    cdf.resize(n);

    // 1. 构建 CDF
    cdf[0] = weights[0];
    for (int i = 1; i < n; i++) {
        cdf[i] = cdf[i - 1] + weights[i];
    }
    
    T total_weight = cdf[n - 1];
    if (total_weight == T(0)) {
        return DiscreteSampleResult<T>{-1, T(0), T(0)};
    }

    // 2. 采样逻辑 (直接在非归一化的 CDF 上做，减少除法次数，提高精度)
    // 我们把 u 放大到 total_weight 范围内，而不是把 CDF 缩小到 1
    T u_scaled = u * total_weight;

    auto it = std::upper_bound(cdf.begin(), cdf.end(), u_scaled);
    
    int idx = 0;
    if (it == cdf.end()) {
        idx = n - 1;
    } else {
        idx = static_cast<int>(std::distance(cdf.begin(), it));
    }

    // 3. 准备返回值
    T pdf = weights[idx] / total_weight;

    // 4. 【核心修复】计算重映射后的 u
    // u_remapped = (u_scaled - cdf_prev) / weight_current
    T cdf_prev = (idx == 0) ? T(0) : cdf[idx - 1];
    T u_remapped = (u_scaled - cdf_prev) / weights[idx];

    // 钳制防止浮点误差
    if (u_remapped >= T(1)) u_remapped = T(1) - std::numeric_limits<T>::epsilon();
    if (u_remapped < T(0)) u_remapped = T(0);

    return DiscreteSampleResult<T>{idx, pdf, u_remapped};
}

template<typename T>
inline T sample_discrete_pdf(
    const std::vector<T>& weights,
    int index
) {
    if (weights.empty() || index >= weights.size()) {
        return T(0);
    }

    T total_weight = T(0);
    for (const auto& w : weights) {
        total_weight += w;
    }
    if (total_weight == T(0)) {
        return T(0);
    }

    return weights[index] / total_weight;
}


};
