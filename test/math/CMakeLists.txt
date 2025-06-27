add_executable(test_vector test_vector.cpp)
target_link_libraries(test_vector PRIVATE pbpt_engine gtest_main)
gtest_discover_tests(test_vector)

add_executable(test_point test_point.cpp)
target_link_libraries(test_point PRIVATE pbpt_engine gtest_main)
gtest_discover_tests(test_point)

add_executable(test_ray test_ray.cpp)
target_link_libraries(test_ray PRIVATE pbpt_engine gtest_main )
gtest_discover_tests(test_ray)

add_executable(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE pbpt_engine gtest_main )
gtest_discover_tests(test_matrix)

add_executable(test_homo test_homo.cpp)
target_link_libraries(test_homo PRIVATE pbpt_engine gtest_main )
gtest_discover_tests(test_homo)

add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform PRIVATE pbpt_engine gtest_main )
gtest_discover_tests(test_transform)

add_executable(test_bounding_box test_bounding_box.cpp)
target_link_libraries(test_bounding_box PRIVATE pbpt_engine gtest_main)
gtest_discover_tests(test_bounding_box)



