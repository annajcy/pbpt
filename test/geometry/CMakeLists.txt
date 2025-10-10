# Geometry tests
add_executable(test_bounds test_bounds.cpp)
target_link_libraries(test_bounds PRIVATE pbpt gtest::gtest)
add_test(NAME test_bounds COMMAND test_bounds)

add_executable(test_directional_cone test_directional_cone.cpp)
target_link_libraries(test_directional_cone PRIVATE pbpt gtest::gtest)
add_test(NAME test_directional_cone COMMAND test_directional_cone)

add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame PRIVATE pbpt gtest::gtest)
add_test(NAME test_frame COMMAND test_frame)

add_executable(test_ray test_ray.cpp)
target_link_libraries(test_ray PRIVATE pbpt gtest::gtest)
add_test(NAME test_ray COMMAND test_ray)

add_executable(test_spherical test_spherical.cpp)
target_link_libraries(test_spherical PRIVATE pbpt gtest::gtest)
add_test(NAME test_spherical COMMAND test_spherical)

add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform PRIVATE pbpt gtest::gtest)
add_test(NAME test_transform COMMAND test_transform)

add_executable(test_interaction test_interaction.cpp)
target_link_libraries(test_interaction PRIVATE pbpt gtest::gtest)
add_test(NAME test_interaction COMMAND test_interaction)
