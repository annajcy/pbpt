#pragma once

#include <array>
#include <tuple>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

#include "core/spectrum.hpp"

namespace pbpt::utils {

// 统一固定范围：360..830 nm（共 471 个样本）
template<typename T>
struct CIE1931_2deg_Info {
    static constexpr int LMin = 360;
    static constexpr int LMax = 830;
    static constexpr int Count = LMax - LMin + 1;
};

// 读取 CIE 官方 CSV（列：wavelength, x_bar, y_bar, z_bar）
template<typename T, int LMin = CIE1931_2deg_Info<T>::LMin, int LMax = CIE1931_2deg_Info<T>::LMax>
inline std::tuple<
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>, // x̄
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>, // ȳ
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>  // z̄
>
make_cie1931_2deg_xyz_from_csv(const std::string& csv_path) {
    static_assert(LMin <= LMax, "Bad wavelength range");
    constexpr int Count = LMax - LMin + 1;

    std::array<T, Count> xb{}, yb{}, zb{};
    std::vector<bool> seen(Count, false);

    std::ifstream fin(csv_path);
    if (!fin) throw std::runtime_error("Cannot open CIE CSV: " + csv_path);

    std::string line;
    // 尽量兼容逗号/空白分隔与表头
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        if (line[0] == '#' || line.find_first_of("0123456789") == std::string::npos) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int wl = 0;
        T x = 0, y = 0, z = 0;
        if (!(iss >> wl >> x >> y >> z)) continue;
        if (wl < LMin || wl > LMax) continue;
        const int idx = wl - LMin;
        xb[idx] = x; yb[idx] = y; zb[idx] = z;
        seen[idx] = true;
    }

    // 基本完备性检查（允许边缘为 0，但不允许大面积缺失）
    int filled = std::count(seen.begin(), seen.end(), true);
    if (filled < Count * 9 / 10) { // 至少 90% 行
        throw std::runtime_error("CIE CSV seems incomplete: only " + std::to_string(filled) + " / " + std::to_string(Count));
    }

    return {
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(xb),
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(yb),
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(zb)
    };
}


template<typename T>
inline T g_piecewise(T lambda, T mu, T tau_left, T tau_right) {
    const T d = lambda - mu;
    const T tau = (lambda < mu) ? tau_left : tau_right;
    const T td = tau * d;
    return std::exp(T(-0.5) * td * td);
}

template<typename T, int LMin, int LMax>
inline std::tuple<
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>, // x̄
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>, // ȳ
    pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>  // z̄
>
make_cie1931_2deg_xyz_analytic() {
    static_assert(LMin <= LMax);
    constexpr int Count = LMax - LMin + 1;

    std::array<T, Count> xb{}, yb{}, zb{};

    for (int i = 0; i < Count; ++i) {
        const T l = static_cast<T>(LMin + i); // nm

        // x̄(λ)
        xb[i] =
            T(1.056) * g_piecewise(l, T(599.8), T(0.0264), T(0.0323)) +
            T(0.362) * g_piecewise(l, T(442.0), T(0.0624), T(0.0374)) -
            T(0.065) * g_piecewise(l, T(501.1), T(0.0490), T(0.0382));

        // ȳ(λ)
        yb[i] =
            T(0.821) * g_piecewise(l, T(568.8), T(0.0213), T(0.0247)) +
            T(0.286) * g_piecewise(l, T(530.9), T(0.0613), T(0.0322));

        // z̄(λ)
        zb[i] =
            T(1.217) * g_piecewise(l, T(437.0), T(0.0845), T(0.0278)) +
            T(0.681) * g_piecewise(l, T(459.0), T(0.0385), T(0.0725));
    }

    return {
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(xb),
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(yb),
        pbpt::core::TabularSpectrumDistribution<T, LMin, LMax>(zb)
    };
}

};
