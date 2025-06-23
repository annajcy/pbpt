add_executable(test_vec test_vec.cpp)
target_link_libraries(test_vec PRIVATE pbpt_engine gtest_main)
gtest_discover_tests(test_vec)

add_executable(test_point test_point.cpp)
target_link_libraries(test_point PRIVATE pbpt_engine gtest_main)
gtest_discover_tests(test_point)

add_executable(test_ray test_ray.cpp)
target_link_libraries(test_ray PRIVATE pbpt_engine gtest_main )
gtest_discover_tests(test_ray)

