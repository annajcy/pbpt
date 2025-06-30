add_executable(pbpt pbpt.cpp)
target_link_libraries(pbpt PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(pbpt ${ext_move_lib})

add_executable(pbpt_test pbpt_test.cpp)
target_link_libraries(pbpt_test PRIVATE pbpt_engine)
add_runtime_dependency_copy_step(pbpt_test ${ext_move_lib})
