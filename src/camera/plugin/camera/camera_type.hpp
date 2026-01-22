#pragma once

#include <variant>
#include "camera/plugin/camera/projective_cameras.hpp"

namespace pbpt::camera {
// Camera Variant
// 这里列出所有支持的相机类型
template<typename T>
using AnyCamera = std::variant<
    camera::ThinLensPerspectiveCamera<T>
    // 将来可以添加: camera::OrthographicCamera<T>, camera::EnvironmentCamera<T> 等
>;
};
