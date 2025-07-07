cmake_minimum_required(VERSION 3.28)
project(pbpt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(BUILD_SHARED_LIBS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# 检查编译器是否为 GCC (GNU) 或 Clang
if(CMAKE_COMPILER_IS_GNU_CXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 为 GCC 和 Clang 添加 -Wno-error 来禁用 "warnings as errors"
    if (APPLE)
       add_compile_options("-Wno-error" "-Wno-deprecated-non-prototype")
    else()
      add_compile_options("-Wno-error" "-Wno-nontrivial-memcall" "-Wno-deprecated-non-prototype")
    endif()
   
    message(STATUS "Compiler is GCC/Clang, disabling warnings as errors with -Wno-error")

# 检查编译器是否为 MSVC (Visual Studio)
elseif(MSVC)
    # 为 MSVC 添加 /WX- 来禁用 "warnings as errors"
    add_compile_options("/WX-")
    message(STATUS "Compiler is MSVC, disabling warnings as errors with /WX-")

endif()

include(FetchContent)

option(PBPT_BUILD_TESTS "Build the project's tests" ON)
option(PBPT_BUILD_DOCUMENTATION "Build the Doxygen documentation" ON)
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ON)
option(FLOAT_64BIT "Use double precision floating point" OFF)
set(RENDER_BACKEND "Vulkan" CACHE STRING "Select the rendering backend (Vulkan or OpenGL)")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "Vulkan" "OpenGL")
message(STATUS "Selected rendering backend: ${RENDER_BACKEND}")

if (RENDER_BACKEND STREQUAL "Vulkan")
  add_definitions("-DRENDER_BACKEND_VULKAN")
endif()

if (RENDER_BACKEND STREQUAL "OpenGL")
  add_definitions("-DRENDER_BACKEND_OPENGL")
endif()

if(FLOAT_64BIT)
  add_definitions("-DFLOAT_64BIT")
endif()

include(extension) 
include(utils)

# 添加项目源代码
add_subdirectory(engine)
add_subdirectory(main)

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

