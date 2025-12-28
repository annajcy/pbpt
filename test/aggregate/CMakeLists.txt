add_executable(test_aggregate test_aggregate.cpp)
target_link_libraries(test_aggregate PRIVATE pbpt gtest::gtest)
add_test(NAME test_aggregate COMMAND test_aggregate)
