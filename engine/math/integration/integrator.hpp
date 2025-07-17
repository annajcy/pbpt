#pragma once

#include "math/global/type_alias.hpp"
#include "math/geometry/point.hpp"
#include "math/integration/function.hpp"
#include <memory>
#include <vector>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace pbpt::math {

/**
 * @brief 统一的估计器接口
 * 这是采样流程的第四层：估计器层
 * 负责使用采样器提供的样本来估计积分值
 * @tparam N 积分空间的维度
 */
template<typename T, int N>
requires std::floating_point<T> && (N > 0)
class Integrator {
private:
    std::shared_ptr<Function<T, N>> m_integrand;
    std::shared_ptr<Domain<T, N>> m_domain;

public:
    Integrator(
        const std::shared_ptr<Function<T, N>>& integrand, 
        const std::shared_ptr<Domain<T, N>>& domain) : 
        m_integrand(integrand), m_domain(domain){}

    virtual ~Integrator() = default;
    virtual T estimate() = 0;
};


} // namespace pbpt::math