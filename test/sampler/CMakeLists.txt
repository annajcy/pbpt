# Sampler tests
add_executable(test_1d test_1d.cpp)
target_link_libraries(test_1d PRIVATE pbpt gtest::gtest)
add_test(NAME test_1d COMMAND test_1d)

add_executable(test_2d test_2d.cpp)
target_link_libraries(test_2d PRIVATE pbpt gtest::gtest)
add_test(NAME test_2d COMMAND test_2d)

add_executable(test_3d test_3d.cpp)
target_link_libraries(test_3d PRIVATE pbpt gtest::gtest)
add_test(NAME test_3d COMMAND test_3d)

add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils PRIVATE pbpt gtest::gtest)
add_test(NAME test_utils COMMAND test_utils)
