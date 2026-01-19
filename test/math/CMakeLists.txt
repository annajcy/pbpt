# Basic math type tests
add_executable(test_vector test_vector.cpp)
target_link_libraries(test_vector PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_vector)

add_executable(test_tuple test_tuple.cpp)
target_link_libraries(test_tuple PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_tuple)

add_executable(test_point test_point.cpp)
target_link_libraries(test_point PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_point)

add_executable(test_normal test_normal.cpp)
target_link_libraries(test_normal PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_normal)

add_executable(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_matrix)

add_executable(test_homo test_homo.cpp)
target_link_libraries(test_homo PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_homo)

add_executable(test_octahedral test_octahedral.cpp)
target_link_libraries(test_octahedral PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_octahedral)
