# Sampler tests
add_executable(test_1d test_1d.cpp)
target_link_libraries(test_1d PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_1d)

add_executable(test_2d test_2d.cpp)
target_link_libraries(test_2d PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_2d)

add_executable(test_3d test_3d.cpp)
target_link_libraries(test_3d PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_3d)

add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_utils)
