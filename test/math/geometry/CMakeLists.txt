# Geometry tests
add_executable(test_vector test_vector.cpp)
target_link_libraries(test_vector PRIVATE pbpt_engine gtest_main)
add_test(NAME test_vector COMMAND test_vector)

add_executable(test_tuple test_tuple.cpp)
target_link_libraries(test_tuple PRIVATE pbpt_engine gtest_main)
add_test(NAME test_tuple COMMAND test_tuple)

add_executable(test_point test_point.cpp)
target_link_libraries(test_point PRIVATE pbpt_engine gtest_main)
add_test(NAME test_point COMMAND test_point)

add_executable(test_normal test_normal.cpp)
target_link_libraries(test_normal PRIVATE pbpt_engine gtest_main)
add_test(NAME test_normal COMMAND test_normal)

add_executable(test_ray test_ray.cpp)
target_link_libraries(test_ray PRIVATE pbpt_engine gtest_main)
add_test(NAME test_ray COMMAND test_ray)

add_executable(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE pbpt_engine gtest_main)
add_test(NAME test_matrix COMMAND test_matrix)

add_executable(test_matrix_enhanced test_matrix_enhanced.cpp)
target_link_libraries(test_matrix_enhanced PRIVATE pbpt_engine gtest_main)
add_test(NAME test_matrix_enhanced COMMAND test_matrix_enhanced)

add_executable(test_homo test_homo.cpp)
target_link_libraries(test_homo PRIVATE pbpt_engine gtest_main)
add_test(NAME test_homo COMMAND test_homo)

add_executable(test_homo_enhanced test_homo_enhanced.cpp)
target_link_libraries(test_homo_enhanced PRIVATE pbpt_engine gtest_main)
add_test(NAME test_homo_enhanced COMMAND test_homo_enhanced)

add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform PRIVATE pbpt_engine gtest_main)
add_test(NAME test_transform COMMAND test_transform)

add_executable(test_bounds test_bounds.cpp)
target_link_libraries(test_bounds PRIVATE pbpt_engine gtest_main)
add_test(NAME test_bounds COMMAND test_bounds)

add_executable(test_spherical test_spherical.cpp)
target_link_libraries(test_spherical PRIVATE pbpt_engine gtest_main)
add_test(NAME test_spherical COMMAND test_spherical)

add_executable(test_quaternion test_quaternion.cpp)
target_link_libraries(test_quaternion PRIVATE pbpt_engine gtest_main)
add_test(NAME test_quaternion COMMAND test_quaternion)

add_executable(test_directional_cone test_directional_cone.cpp)
target_link_libraries(test_directional_cone PRIVATE pbpt_engine gtest_main)
add_test(NAME test_directional_cone COMMAND test_directional_cone)

add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame PRIVATE pbpt_engine gtest_main)
add_test(NAME test_frame COMMAND test_frame)

add_executable(test_interval test_interval.cpp)
target_link_libraries(test_interval PRIVATE pbpt_engine gtest_main)
add_test(NAME test_interval COMMAND test_interval)

add_executable(test_point_eps test_point_eps.cpp)
target_link_libraries(test_point_eps PRIVATE pbpt_engine gtest_main)
add_test(NAME test_point_eps COMMAND test_point_eps)
