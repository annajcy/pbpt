#pragma once
#include <variant>
#include <type_traits>

namespace pbpt::utils {

// 检测类型是否为 std::variant
template<typename T>
struct is_variant : std::false_type {};

template<typename... Args>
struct is_variant<std::variant<Args...>> : std::true_type {};

template<typename T>
inline constexpr bool is_variant_v = is_variant<std::decay_t<T>>::value;

// visit_any: 如果是 variant 则 visit，否则直接调用 visitor
template<typename Visitor, typename... Variants>
constexpr auto visit_any(Visitor&& visitor, Variants&&... vars) {
    if constexpr ((is_variant_v<Variants> || ...)) {
        // 只要有一个是 variant，就使用 std::visit
        // 注意：这里我们做了一个简化假设，假设用户不会混用 variant 和非 variant
        // 如果要支持混用，需要把非 variant 包装成只有一个选项的 variant，或者使用更复杂的各种重载技术
        // 这里为了简单，我们用 if constexpr 分别处理 "全是 variant" (假设场景) 或 "需单独处理"
        
        // 更健壮的实现较复杂，对于我们目前的 Scene 应用场景：
        // 我们通常针对 *单个对象* 进行处理，所以下面的特化版本更实用：
        return std::visit(std::forward<Visitor>(visitor), std::forward<Variants>(vars)...);
    } else {
        // 都不是 variant，直接调用 visitor
        return std::forward<Visitor>(visitor)(std::forward<Variants>(vars)...);
    }
}

// 针对单个参数的特化版本（最常用），完美解决你的问题
template<typename Visitor, typename T>
constexpr decltype(auto) visit_one(Visitor&& visitor, T&& arg) {
    if constexpr (is_variant_v<T>) {
        return std::visit(std::forward<Visitor>(visitor), std::forward<T>(arg));
    } else {
        return std::forward<Visitor>(visitor)(std::forward<T>(arg));
    }
}

}