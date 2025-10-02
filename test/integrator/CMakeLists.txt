# Integration tests
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE pbpt gtest::gtest)
add_test(NAME test_integration COMMAND test_integration)

add_executable(test_sampler test_sampler.cpp)
target_link_libraries(test_sampler PRIVATE pbpt gtest::gtest)
add_test(NAME test_sampler COMMAND test_sampler)
