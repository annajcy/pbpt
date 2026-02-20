#pragma once

#include <concepts>
#include <string_view>

#include <pugixml.hpp>

#include "pbpt/scene/scene.hpp"
#include "pbpt/serde/context.hpp"

namespace pbpt::serde {

template <typename SerdeT>
consteval bool serde_domain_equals(std::string_view expected) {
    return SerdeT::domain == expected;
}

template <typename SerdeT>
concept DomainSerdeConcept = requires {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
};

template <typename ValueT>
struct ValueWriteTarget {
    const ValueT& value;
};

template <typename ValueT>
struct IdValueWriteTarget {
    std::string_view id;
    const ValueT& value;
};

template <typename T>
struct ShapeWriteTarget {
    const scene::ShapeInstanceRecord<T>& record;
};

template <typename T, typename SerdeT>
concept SerdeConcept = DomainSerdeConcept<SerdeT> && requires(const pugi::xml_node& node, LoadContext<T>& lctx,
                                                              const typename SerdeT::write_target& write_target,
                                                              pugi::xml_node& wnode, WriteContext<T>& wctx) {
    typename SerdeT::load_result;
    typename SerdeT::write_target;
    { SerdeT::load(node, lctx) } -> std::same_as<typename SerdeT::load_result>;
    { SerdeT::write(write_target, wnode, wctx) } -> std::same_as<void>;
};

template <typename T, typename SerdeT>
concept ValueSerdeConcept =
    SerdeConcept<T, SerdeT> && (!std::same_as<typename SerdeT::load_result, void>) &&
    std::same_as<typename SerdeT::write_target, IdValueWriteTarget<typename SerdeT::load_result>>;

template <typename T, typename SerdeT>
concept ShapeSerdeConcept = SerdeConcept<T, SerdeT> && std::same_as<typename SerdeT::load_result, void> &&
                            std::same_as<typename SerdeT::write_target, ShapeWriteTarget<T>>;

template <typename T, typename SerdeT>
concept TextureSerdeConcept = ValueSerdeConcept<T, SerdeT> && serde_domain_equals<SerdeT>("texture");

template <typename T, typename SerdeT>
concept MaterialSerdeConcept = ValueSerdeConcept<T, SerdeT> && serde_domain_equals<SerdeT>("material");

template <typename T, typename SerdeT>
concept CameraSerdeConcept =
    SerdeConcept<T, SerdeT> && std::same_as<typename SerdeT::load_result, void> &&
    requires { typename SerdeT::value_type; } &&
    std::same_as<typename SerdeT::write_target, ValueWriteTarget<typename SerdeT::value_type>> &&
    serde_domain_equals<SerdeT>("camera");

template <typename T, typename SerdeT>
concept IntegratorSerdeConcept =
    SerdeConcept<T, SerdeT> && std::same_as<typename SerdeT::load_result, void> &&
    requires { typename SerdeT::value_type; } &&
    std::same_as<typename SerdeT::write_target, ValueWriteTarget<typename SerdeT::value_type>> &&
    serde_domain_equals<SerdeT>("integrator");

template <typename T, typename SerdeT>
concept SamplerSerdeConcept =
    SerdeConcept<T, SerdeT> && std::same_as<typename SerdeT::load_result, void> &&
    requires { typename SerdeT::value_type; } &&
    std::same_as<typename SerdeT::write_target, ValueWriteTarget<typename SerdeT::value_type>> &&
    serde_domain_equals<SerdeT>("sampler");

}  // namespace pbpt::serde
