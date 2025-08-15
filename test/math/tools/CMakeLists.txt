# Tools tests
add_executable(test_octahedral test_octahedral.cpp)
target_link_libraries(test_octahedral PRIVATE pbpt_engine gtest_main)
add_test(NAME test_octahedral COMMAND test_octahedral)
