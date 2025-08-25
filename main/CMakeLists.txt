add_executable(backend_test backend_test.cpp)
target_link_libraries(backend_test PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(backend_test ${ext_move_lib})

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(main ${ext_move_lib})
