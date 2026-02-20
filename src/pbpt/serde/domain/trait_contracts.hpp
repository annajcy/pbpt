#pragma once

#include <string_view>
#include <string>
#include <concepts>
#include <pugixml.hpp>
#include "pbpt/serde/context.hpp"
#include "pbpt/scene/scene.hpp"

namespace pbpt::serde {

template <typename T, typename SerdeT>
concept TextureSerdeConcept = requires(const pugi::xml_node& node, LoadContext<T>& lctx,
                                       const typename SerdeT::value_type& val, const std::string& id,
                                       pugi::xml_node& wnode, WriteContext<T>& wctx) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    typename SerdeT::value_type;
    { SerdeT::load(node, lctx) } -> std::same_as<typename SerdeT::value_type>;
    { SerdeT::write(val, id, wnode, wctx) } -> std::same_as<void>;
} && SerdeT::domain == "texture";

template <typename T, typename SerdeT>
concept MaterialSerdeConcept = requires(const pugi::xml_node& node, LoadContext<T>& lctx,
                                        const typename SerdeT::value_type& val, const std::string& id,
                                        pugi::xml_node& wnode, WriteContext<T>& wctx) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    typename SerdeT::value_type;
    { SerdeT::load(node, lctx) } -> std::same_as<typename SerdeT::value_type>;
    { SerdeT::write(val, id, wnode, wctx) } -> std::same_as<void>;
} && SerdeT::domain == "material";

template <typename T, typename SerdeT>
concept ShapeSerdeConcept = requires(const pugi::xml_node& node, LoadContext<T>& lctx,
                                     const scene::ShapeInstanceRecord<T>& record, pugi::xml_node& wnode,
                                     WriteContext<T>& wctx) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    { SerdeT::load(node, lctx) } -> std::same_as<void>;  // shape loads into context's resources
    { SerdeT::write(record, wnode, wctx) } -> std::same_as<void>;
} && SerdeT::domain == "shape";

template <typename T, typename SerdeT>
concept CameraSerdeConcept = requires(const pugi::xml_node& node, scene::Scene<T>& scene, LoadContext<T>& lctx,
                                      const scene::Scene<T>& const_scene, pugi::xml_node& wnode,
                                      WriteContext<T>& wctx) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    { SerdeT::load(node, scene, lctx) } -> std::same_as<void>;
    { SerdeT::write(const_scene, wnode, wctx) } -> std::same_as<void>;
} && SerdeT::domain == "camera";

template <typename T, typename SerdeT>
concept IntegratorSerdeConcept = requires(const pugi::xml_node& node, scene::Scene<T>& scene,
                                          const scene::Scene<T>& const_scene, pugi::xml_node& wnode) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    { SerdeT::load(node, scene) } -> std::same_as<void>;
    { SerdeT::write(const_scene, wnode) } -> std::same_as<void>;
} && SerdeT::domain == "integrator";

template <typename T, typename SerdeT>
concept SamplerSerdeConcept = requires(const pugi::xml_node& node, scene::Scene<T>& scene,
                                       const scene::Scene<T>& const_scene, pugi::xml_node& wnode) {
    { SerdeT::domain } -> std::same_as<const std::string_view&>;
    { SerdeT::xml_type } -> std::same_as<const std::string_view&>;
    { SerdeT::load(node, scene) } -> std::same_as<void>;
    { SerdeT::write(const_scene, wnode) } -> std::same_as<void>;
} && SerdeT::domain == "sampler";

}  // namespace pbpt::serde
