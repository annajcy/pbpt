find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(RGB_SPECTRUM_INPUT ${CMAKE_SOURCE_DIR}/asset/rgb2spec/rgbspectrum_srgb.cpp)
set(RGB_SPECTRUM_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/rgb_spectrum_lut_data.cpp)

add_custom_command(
    OUTPUT ${RGB_SPECTRUM_OUTPUT}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/generated
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/script/generate_rgb_spectrum_lut.py
            --input ${RGB_SPECTRUM_INPUT}
            --output ${RGB_SPECTRUM_OUTPUT}
    DEPENDS ${RGB_SPECTRUM_INPUT} ${CMAKE_SOURCE_DIR}/script/generate_rgb_spectrum_lut.py
    COMMENT "Generating RGB->spectrum LUT translation unit"
    VERBATIM
)
set_source_files_properties(${RGB_SPECTRUM_OUTPUT} PROPERTIES GENERATED TRUE)
add_custom_target(generate_rgb_spectrum_lut DEPENDS ${RGB_SPECTRUM_OUTPUT})

add_library(pbpt_rgb_spectrum_lut STATIC
    ${RGB_SPECTRUM_OUTPUT}
)
add_dependencies(pbpt_rgb_spectrum_lut generate_rgb_spectrum_lut)
target_include_directories(pbpt_rgb_spectrum_lut PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(pbpt INTERFACE)
target_include_directories(pbpt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pbpt INTERFACE 
    pbpt_rgb_spectrum_lut
    imgui_vk 
    stb_impl 
    glfw 
    Vulkan::Loader 
    assimp::assimp 
    slang::slang
    OpenEXR::OpenEXR
)
