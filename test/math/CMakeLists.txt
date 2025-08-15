# Include subdirectories
add_subdirectory(geometry)
add_subdirectory(integration)
add_subdirectory(tools)

# Performance tests (separate from regular tests)
add_executable(test_matrix_performance geometry/test_matrix.cpp)
target_link_libraries(test_matrix_performance PRIVATE pbpt_engine gtest_main)
target_compile_definitions(test_matrix_performance PRIVATE PERFORMANCE_TEST=1)
add_test(NAME test_matrix_performance COMMAND test_matrix_performance)




