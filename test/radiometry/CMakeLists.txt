# Core tests

add_executable(test_radiometric_integrals test_radiometric_integrals.cpp)
target_link_libraries(test_radiometric_integrals PRIVATE pbpt gtest::gtest)
add_test(NAME test_radiometric_integrals COMMAND test_radiometric_integrals)

add_executable(test_spectrum test_spectrum.cpp)
target_link_libraries(test_spectrum PRIVATE pbpt gtest::gtest)
add_test(NAME test_spectrum COMMAND test_spectrum)

add_executable(test_color test_color.cpp)
target_link_libraries(test_color PRIVATE pbpt gtest::gtest)
add_test(NAME test_color COMMAND test_color)