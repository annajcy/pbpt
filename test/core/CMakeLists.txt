# Core tests
add_executable(test_interaction test_interaction.cpp)
target_link_libraries(test_interaction PRIVATE pbpt_engine gtest_main)
add_test(NAME test_interaction COMMAND test_interaction)
