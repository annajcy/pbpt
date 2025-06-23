set(RENDER_BACKEND "Vulkan" CACHE STRING "Select the rendering backend (Vulkan or OpenGL)")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "Vulkan" "OpenGL")
message(STATUS "Selected rendering backend: ${RENDER_BACKEND}")

if (RENDER_BACKEND STREQUAL "Vulkan")
  find_package(Vulkan REQUIRED)
  message(STATUS "Vulkan_VERSION: ${Vulkan_VERSION}")
elseif(RENDER_BACKEND STREQUAL "OpenGL")
  find_package(OpenGL REQUIRED)
  message(STATUS "OpenGL_VERSION: ${OpenGL_VERSION}")
  FetchContent_Declare(glbinding GIT_REPOSITORY https://github.com/cginternals/glbinding.git GIT_TAG v3.3.0)
  set(OPTION_BUILD_TESTS OFF)
  FetchContent_MakeAvailable(glbinding)
  message(STATUS "glbinding_VERSION: ${glbinding_VERSION}")
endif()


if (RENDER_BACKEND STREQUAL "Vulkan")
  add_definitions("-DRENDER_BACKEND_VULKAN")
  list(APPEND render_backend 
    Vulkan::Vulkan
  )
endif()

if (RENDER_BACKEND STREQUAL "OpenGL")
  add_definitions("-DRENDER_BACKEND_OPENGL")
  list(APPEND render_backend 
    glbinding::glbinding
    glbinding::glbinding-aux
    OpenGL::GL
  )
endif()