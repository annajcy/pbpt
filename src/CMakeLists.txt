add_library(pbpt INTERFACE)
target_include_directories(pbpt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pbpt INTERFACE 
    imgui_vk 
    stb_impl 
    glfw 
    Vulkan::Loader 
    assimp::assimp 
    slang::slang
    OpenEXR::OpenEXR
)
