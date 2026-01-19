add_executable(test_aggregate test_aggregate.cpp)
target_link_libraries(test_aggregate PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_aggregate)
