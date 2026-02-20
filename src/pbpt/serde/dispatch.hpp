#pragma once

#include <string>
#include <string_view>
#include <stdexcept>
#include <tuple>
#include <variant>
#include <pugixml.hpp>


#include "pbpt/serde/context.hpp"
#include "pbpt/serde/domain/trait_contracts.hpp"
#include "pbpt/texture/plugin/texture/texture_type.hpp"
#include "pbpt/material/plugin/material/material_type.hpp"

namespace pbpt::serde {

template <typename T, typename Tuple, std::size_t Index = 0>
texture::AnyTexture<T> dispatch_load_texture(const std::string& type, const pugi::xml_node& node,
                                             LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            return SerdeT::load(node, ctx);
        }
        return dispatch_load_texture<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported texture type: " + type +
                                 " in node: " + std::string(node.attribute("id").value()));
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_texture(const texture::AnyTexture<T>& any_tex, const std::string& id, pugi::xml_node& node,
                            WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (const auto* val = std::get_if<typename SerdeT::value_type>(&any_tex)) {
            SerdeT::write(IdValueWriteTarget<typename SerdeT::value_type>{id, *val}, node, ctx);
            return;
        }
        dispatch_write_texture<T, Tuple, Index + 1>(any_tex, id, node, ctx);
    } else {
        throw std::runtime_error("Unsupported texture type for serialization: " + id);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
material::AnyMaterial<T> dispatch_load_material(const std::string& type, const pugi::xml_node& node,
                                                LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            return SerdeT::load(node, ctx);
        }
        return dispatch_load_material<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported material type: " + type +
                                 " in node: " + std::string(node.attribute("id").value()));
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_material(const material::AnyMaterial<T>& any_mat, const std::string& id, pugi::xml_node& node,
                             WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (const auto* val = std::get_if<typename SerdeT::value_type>(&any_mat)) {
            SerdeT::write(IdValueWriteTarget<typename SerdeT::value_type>{id, *val}, node, ctx);
            return;
        }
        dispatch_write_material<T, Tuple, Index + 1>(any_mat, id, node, ctx);
    } else {
        throw std::runtime_error("Unsupported material type for serialization: " + id);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_load_shape(const std::string& type, const pugi::xml_node& node, LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::load(node, ctx);
            return;
        }
        dispatch_load_shape<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported shape type: " + type +
                                 " in node: " + std::string(node.attribute("id").value()));
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_shape(const scene::ShapeInstanceRecord<T>& record, pugi::xml_node& node, WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (record.shape_type == SerdeT::xml_type) {
            SerdeT::write(ShapeWriteTarget<T>{record}, node, ctx);
            return;
        }
        dispatch_write_shape<T, Tuple, Index + 1>(record, node, ctx);
    } else {
        throw std::runtime_error("Unsupported shape type for serialization: " + record.shape_type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_load_camera(const std::string& type, const pugi::xml_node& node, LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::load(node, ctx);
            return;
        }
        dispatch_load_camera<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported camera type: " + type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_camera(const std::string& type, const scene::Scene<T>& scene, pugi::xml_node& node,
                           WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::write(SceneWriteTarget<T>{scene}, node, ctx);
            return;
        }
        dispatch_write_camera<T, Tuple, Index + 1>(type, scene, node, ctx);
    } else {
        throw std::runtime_error("Unsupported camera xml type: " + type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_load_integrator(const std::string& type, const pugi::xml_node& node, LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::load(node, ctx);
            return;
        }
        dispatch_load_integrator<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported integrator type: " + type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_integrator(const std::string& type, const scene::Scene<T>& scene, pugi::xml_node& node,
                               WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::write(SceneWriteTarget<T>{scene}, node, ctx);
            return;
        }
        dispatch_write_integrator<T, Tuple, Index + 1>(type, scene, node, ctx);
    } else {
        throw std::runtime_error("Unsupported integrator xml type: " + type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_load_sampler(const std::string& type, const pugi::xml_node& node, LoadContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::load(node, ctx);
            return;
        }
        dispatch_load_sampler<T, Tuple, Index + 1>(type, node, ctx);
    } else {
        throw std::runtime_error("Unsupported sampler type: " + type);
    }
}

template <typename T, typename Tuple, std::size_t Index = 0>
void dispatch_write_sampler(const std::string& type, const scene::Scene<T>& scene, pugi::xml_node& node,
                            WriteContext<T>& ctx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        using SerdeT = std::tuple_element_t<Index, Tuple>;
        if (type == SerdeT::xml_type) {
            SerdeT::write(SceneWriteTarget<T>{scene}, node, ctx);
            return;
        }
        dispatch_write_sampler<T, Tuple, Index + 1>(type, scene, node, ctx);
    } else {
        throw std::runtime_error("Unsupported sampler xml type: " + type);
    }
}

}  // namespace pbpt::serde
