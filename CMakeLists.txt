cmake_minimum_required(VERSION 3.28)
project(pbpt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(PBPT_BUILD_TESTS "Build the project's tests" ON)
option(BUILD_DOCUMENTATION "Build the Doxygen documentation" ON)

# ====================================================================
# 全局构建选项和配置
# ====================================================================
# <--- 修改点: 直接使用 RENDER_BACKEND 作为用户开关
set(RENDER_BACKEND "Vulkan" CACHE STRING "Select the rendering backend (Vulkan or OpenGL)")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "Vulkan" "OpenGL")
message(STATUS "Selected rendering backend: ${RENDER_BACKEND}")

# 创建一个INTERFACE库来承载全局编译定义
add_library(pbpt_options INTERFACE)

# <--- 修改点: 使用 RENDER_BACKEND 的值直接构造宏的值
# 这行代码会根据变量值生成 -DRENDER_BACKEND=RenderBackend::Vulkan 或 -DRENDER_BACKEND=RenderBackend::OpenGL 编译指令
target_compile_definitions(pbpt_options INTERFACE 
  "RENDER_BACKEND=RenderBackend::${RENDER_BACKEND}"
)

if (RENDER_BACKEND STREQUAL "Vulkan")
  target_compile_definitions(pbpt_options INTERFACE 
    "RENDER_BACKEND_VULKAN"
  )
endif()

if (RENDER_BACKEND STREQUAL "OpenGL")
  target_compile_definitions(pbpt_options INTERFACE 
    "RENDER_BACKEND_OPENGL"
  )
endif()

# 将src目录添加到 include 路径，方便 C++ include "RenderBackend.h"
target_include_directories(pbpt_options INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ====================================================================
# 包含依赖项和辅助函数
# ====================================================================
include(extension) # 所有 FetchContent 的魔法都在这个文件里
include(utils)        # 包含我们的辅助函数，比如复制DLL

# ====================================================================
# 添加项目源代码
# ====================================================================
add_subdirectory(src)

# ====================================================================
# 测试构建选项
# ====================================================================
if(PBPT_BUILD_TESTS)
  # 为测试启用 CTest
  enable_testing()
  add_subdirectory(test)
  message(STATUS "Tests are enabled.")
else()
  message(STATUS "Tests are disabled. To enable them, set PBPT_BUILD_TESTS to ON.")
endif()


# ====================================================================
# 文档生成
# ====================================================================
if(BUILD_DOCUMENTATION)
  add_subdirectory(docs)
endif()