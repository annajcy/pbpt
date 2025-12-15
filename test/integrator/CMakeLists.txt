# Core tests

add_executable(test_integrator test_integrator.cpp)
target_link_libraries(test_integrator PRIVATE pbpt gtest::gtest)
add_test(NAME test_integrator COMMAND test_integrator)
