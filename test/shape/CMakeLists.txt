add_executable(test_sphere test_sphere.cpp)
target_link_libraries(test_sphere PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_sphere)

add_executable(test_triangle test_triangle.cpp)
target_link_libraries(test_triangle PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_triangle)
