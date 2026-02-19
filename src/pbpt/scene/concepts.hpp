#pragma once

namespace pbpt::scene {

template <typename ResourcesT>
concept MaterialLibraryConcept = requires(const ResourcesT& resources) {
    resources.any_material_library.get(0);
};

template <typename ResourcesT>
concept LightLibraryConcept = requires(const ResourcesT& resources) {
    resources.any_light_library.get(0);
};

template <typename ResourcesT>
concept SceneResourcesConcept = MaterialLibraryConcept<ResourcesT> && LightLibraryConcept<ResourcesT>;

}  // namespace pbpt::scene
