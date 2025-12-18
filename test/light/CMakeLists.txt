add_executable(test_light test_light.cpp)
target_link_libraries(test_light PRIVATE pbpt gtest::gtest)
add_test(NAME test_light COMMAND test_light)
