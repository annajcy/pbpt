add_executable(render_backend render_backend.cpp)
target_link_libraries(render_backend PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(render_backend ${ext_move_lib})

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(main ${ext_move_lib})