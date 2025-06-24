cmake_minimum_required(VERSION 3.28)
project(pbpt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)

option(PBPT_BUILD_TESTS "Build the project's tests" ON)
option(PBPT_BUILD_DOCUMENTATION "Build the Doxygen documentation" ON)
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ON)

set(RENDER_BACKEND "Vulkan" CACHE STRING "Select the rendering backend (Vulkan or OpenGL)")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "Vulkan" "OpenGL")
message(STATUS "Selected rendering backend: ${RENDER_BACKEND}")

if (RENDER_BACKEND STREQUAL "Vulkan")
  add_definitions("-DRENDER_BACKEND_VULKAN")
endif()

if (RENDER_BACKEND STREQUAL "OpenGL")
  add_definitions("-DRENDER_BACKEND_OPENGL")
endif()

include(extension) 
include(utils)

# 添加项目源代码
add_subdirectory(engine)
add_subdirectory(pbpt)

# 测试构建选项
if(PBPT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
  message(STATUS "Tests are enabled.")
else()
  message(STATUS "Tests are disabled. To enable them, set PBPT_BUILD_TESTS to ON.")
endif()

# 文档生成
if(PBPT_BUILD_DOCUMENTATION)
  include(document)
  message(STATUS "Documentation is enabled.")
else()
  message(STATUS "Documentation is disabled. To enable it, set PBPT_BUILD_DOCUMENTATION to ON.")
endif()

