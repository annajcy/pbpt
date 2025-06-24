// main_test_logic.h

#pragma once
#include <iostream>

// Forward declare to avoid including heavy headers here
struct GLFWwindow;

// Define RENDER_BACKEND_VULKAN or RENDER_BACKEND_OPENGL before including this file.

#if defined(RENDER_BACKEND_VULKAN)
  #include "imgui_impl_vulkan.h"
#elif defined(RENDER_BACKEND_OPENGL)
  #include "imgui_impl_opengl3.h"
#else
  #error "No render backend macro defined for test!"
#endif


// The functions to be tested
inline void init_render_backend(GLFWwindow* window, void* vulkan_init_info = nullptr) {
#if defined(RENDER_BACKEND_VULKAN)
    // For Vulkan, the Init function requires a struct with handles
    ImGui_ImplVulkan_Init(static_cast<ImGui_ImplVulkan_InitInfo*>(vulkan_init_info));
    std::cout << "Runtime: Initialized Vulkan Backend" << std::endl;
#elif defined(RENDER_BACKEND_OPENGL)
    // For OpenGL, it just needs a GLSL version string
    ImGui_ImplOpenGL3_Init("#version 130");
    std::cout << "Runtime: Initialized OpenGL Backend" << std::endl;
#endif
}

inline void shutdown_render_backend() {
#if defined(RENDER_BACKEND_VULKAN)
    ImGui_ImplVulkan_Shutdown();
#elif defined(RENDER_BACKEND_OPENGL)
    ImGui_ImplOpenGL3_Shutdown();
#endif
}