add_library(pbpt_engine INTERFACE)
target_include_directories(pbpt_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pbpt_engine INTERFACE ${ext_lib} ${render_backend})

