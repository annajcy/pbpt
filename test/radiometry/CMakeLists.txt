# Core tests

add_executable(test_spectrum test_spectrum.cpp)
target_link_libraries(test_spectrum PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_spectrum)

add_executable(test_color test_color.cpp)
target_link_libraries(test_color PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_color)