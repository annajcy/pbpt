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

add_executable(test_pixel_sensor test_pixel_sensor.cpp)
target_link_libraries(test_pixel_sensor PRIVATE pbpt gtest::gtest)
add_test(NAME test_pixel_sensor COMMAND test_pixel_sensor)

add_executable(test_hdr_film test_hdr_film.cpp)
target_link_libraries(test_hdr_film PRIVATE pbpt gtest::gtest)
add_test(NAME test_hdr_film COMMAND test_hdr_film)

add_executable(test_pixel_filter test_pixel_filter.cpp)
target_link_libraries(test_pixel_filter PRIVATE pbpt gtest::gtest)
add_test(NAME test_pixel_filter COMMAND test_pixel_filter)
