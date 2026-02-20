#pragma once

#include <tuple>
#include <type_traits>
#include <string_view>
#include <utility>

#include "pbpt/texture/plugin/serde.hpp"
#include "pbpt/material/plugin/serde.hpp"
#include "pbpt/shape/plugin/serde.hpp"
#include "pbpt/camera/plugin/serde.hpp"
#include "pbpt/integrator/plugin/serde.hpp"
#include "pbpt/lds/plugin/serde.hpp"

namespace pbpt::serde {

namespace detail {

template <typename Tuple, std::size_t Index, std::size_t Candidate>
consteval bool xml_type_unique_against_rest() {
    if constexpr (Candidate >= std::tuple_size_v<Tuple>) {
        return true;
    } else {
        using CurrentSerde = std::tuple_element_t<Index, Tuple>;
        using CandidateSerde = std::tuple_element_t<Candidate, Tuple>;
        constexpr std::string_view current = CurrentSerde::xml_type;
        constexpr std::string_view candidate = CandidateSerde::xml_type;
        return (current != candidate) && xml_type_unique_against_rest<Tuple, Index, Candidate + 1>();
    }
}

template <typename Tuple, std::size_t Index>
consteval bool xml_type_unique_from_index() {
    if constexpr (Index >= std::tuple_size_v<Tuple>) {
        return true;
    } else {
        return xml_type_unique_against_rest<Tuple, Index, Index + 1>() && xml_type_unique_from_index<Tuple, Index + 1>();
    }
}

}  // namespace detail

template <typename Tuple>
consteval bool has_unique_xml_types() {
    return detail::xml_type_unique_from_index<Tuple, 0>();
}

template <typename T>
using TextureSerdeList = std::tuple<BitmapTextureSerde<T>, CheckerboardTextureSerde<T>>;

template <typename T>
using MaterialSerdeList =
    std::tuple<DiffuseMaterialSerde<T>, DielectricMaterialSerde<T>, DielectricSpecularMaterialSerde<T>,
               DielectricRoughMaterialSerde<T>, ConductorMaterialSerde<T>, ConductorSpecularMaterialSerde<T>,
               ConductorRoughMaterialSerde<T>>;

template <typename T>
using ShapeSerdeList = std::tuple<ObjShapeSerde<T>>;

template <typename T>
using CameraSerdeList = std::tuple<PerspectiveCameraSerde<T>>;

template <typename T>
using IntegratorSerdeList = std::tuple<PathIntegratorSerde<T>>;

template <typename T>
using SamplerSerdeList = std::tuple<LdsSamplerSerde<T>>;

static_assert(has_unique_xml_types<TextureSerdeList<float>>(), "Duplicate xml_type in TextureSerdeList.");
static_assert(has_unique_xml_types<MaterialSerdeList<float>>(), "Duplicate xml_type in MaterialSerdeList.");
static_assert(has_unique_xml_types<ShapeSerdeList<float>>(), "Duplicate xml_type in ShapeSerdeList.");
static_assert(has_unique_xml_types<CameraSerdeList<float>>(), "Duplicate xml_type in CameraSerdeList.");
static_assert(has_unique_xml_types<IntegratorSerdeList<float>>(), "Duplicate xml_type in IntegratorSerdeList.");
static_assert(has_unique_xml_types<SamplerSerdeList<float>>(), "Duplicate xml_type in SamplerSerdeList.");

}  // namespace pbpt::serde
