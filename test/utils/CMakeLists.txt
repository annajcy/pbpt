add_executable(test_visit_any test_visit_any.cpp)
target_link_libraries(test_visit_any PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_visit_any)

add_executable(test_library test_library.cpp)
target_link_libraries(test_library PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_library)
