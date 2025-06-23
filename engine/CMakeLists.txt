add_subdirectory(math)
add_library(pbpt_engine INTERFACE)
target_link_libraries(pbpt_engine INTERFACE pbpt_math ${ext_lib} ${render_backend})

