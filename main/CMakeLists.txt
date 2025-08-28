add_executable(render_backend_test render_backend_test.cpp)
target_link_libraries(render_backend_test PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(render_backend_test ${ext_move_lib})

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(main ${ext_move_lib})

add_executable(function_test function_test.cpp)
target_link_libraries(function_test PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(function_test ${ext_move_lib})

