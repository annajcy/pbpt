# Geometry tests
add_executable(test_bounds test_bounds.cpp)
target_link_libraries(test_bounds PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_bounds)

add_executable(test_directional_cone test_directional_cone.cpp)
target_link_libraries(test_directional_cone PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_directional_cone)

add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_frame)

add_executable(test_ray test_ray.cpp)
target_link_libraries(test_ray PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_ray)

add_executable(test_spherical test_spherical.cpp)
target_link_libraries(test_spherical PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_spherical)

add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_transform)

add_executable(test_interaction test_interaction.cpp)
target_link_libraries(test_interaction PRIVATE pbpt GTest::gtest_main)
gtest_discover_tests(test_interaction)
