# Camera module tests
add_executable(test_projective_camera test_projective_camera.cpp)
target_link_libraries(test_projective_camera PRIVATE pbpt gtest::gtest)
add_test(NAME test_projective_camera COMMAND test_projective_camera)

add_executable(test_render_transform test_render_transform.cpp)
target_link_libraries(test_render_transform PRIVATE pbpt gtest::gtest)
add_test(NAME test_render_transform COMMAND test_render_transform)

add_executable(test_spherical_camera test_spherical_camera.cpp)
target_link_libraries(test_spherical_camera PRIVATE pbpt gtest::gtest)
add_test(NAME test_spherical_camera COMMAND test_spherical_camera)
