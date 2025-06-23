set(RENDER_BACKEND "Vulkan" CACHE STRING "Select the rendering backend (Vulkan or OpenGL)")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "Vulkan" "OpenGL")
message(STATUS "Selected rendering backend: ${RENDER_BACKEND}")


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