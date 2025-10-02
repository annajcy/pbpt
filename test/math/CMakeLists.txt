# Basic math type tests
add_executable(test_vector test_vector.cpp)
target_link_libraries(test_vector PRIVATE pbpt gtest::gtest)
add_test(NAME test_vector COMMAND test_vector)

add_executable(test_tuple test_tuple.cpp)
target_link_libraries(test_tuple PRIVATE pbpt gtest::gtest)
add_test(NAME test_tuple COMMAND test_tuple)

add_executable(test_point test_point.cpp)
target_link_libraries(test_point PRIVATE pbpt gtest::gtest)
add_test(NAME test_point COMMAND test_point)

add_executable(test_point_eps test_point_eps.cpp)
target_link_libraries(test_point_eps PRIVATE pbpt gtest::gtest)
add_test(NAME test_point_eps COMMAND test_point_eps)

add_executable(test_normal test_normal.cpp)
target_link_libraries(test_normal PRIVATE pbpt gtest::gtest)
add_test(NAME test_normal COMMAND test_normal)

add_executable(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE pbpt gtest::gtest)
add_test(NAME test_matrix COMMAND test_matrix)

add_executable(test_matrix_enhanced test_matrix_enhanced.cpp)
target_link_libraries(test_matrix_enhanced PRIVATE pbpt gtest::gtest)
add_test(NAME test_matrix_enhanced COMMAND test_matrix_enhanced)

add_executable(test_homo test_homo.cpp)
target_link_libraries(test_homo PRIVATE pbpt gtest::gtest)
add_test(NAME test_homo COMMAND test_homo)

add_executable(test_homo_enhanced test_homo_enhanced.cpp)
target_link_libraries(test_homo_enhanced PRIVATE pbpt gtest::gtest)
add_test(NAME test_homo_enhanced COMMAND test_homo_enhanced)

add_executable(test_interval test_interval.cpp)
target_link_libraries(test_interval PRIVATE pbpt gtest::gtest)
add_test(NAME test_interval COMMAND test_interval)

add_executable(test_octahedral test_octahedral.cpp)
target_link_libraries(test_octahedral PRIVATE pbpt gtest::gtest)
add_test(NAME test_octahedral COMMAND test_octahedral)





