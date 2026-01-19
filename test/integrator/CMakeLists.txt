# Core tests

add_executable(test_integrator test_integrator.cpp)
target_link_libraries(test_integrator PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_integrator)
