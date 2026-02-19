add_executable(test_visit_any test_visit_any.cpp)
target_link_libraries(test_visit_any PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_visit_any)

add_executable(test_library test_library.cpp)
target_link_libraries(test_library PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_library)

add_executable(test_image_io test_image_io.cpp)
target_link_libraries(test_image_io PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_image_io)

add_executable(test_execution_profiler test_execution_profiler.cpp)
target_link_libraries(test_execution_profiler PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_execution_profiler)
