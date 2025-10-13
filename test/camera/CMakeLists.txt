# Camera module tests
add_executable(test_camera test_camera.cpp)
target_link_libraries(test_camera PRIVATE pbpt gtest::gtest)
add_test(NAME test_camera COMMAND test_camera)
