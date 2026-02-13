cmake_minimum_required(VERSION 3.15)
project(pbpt_test_package LANGUAGES CXX)

find_package(pbpt CONFIG REQUIRED)

add_executable(test_package test_package.cpp)
target_compile_features(test_package PRIVATE cxx_std_23)
target_link_libraries(test_package PRIVATE pbpt::pbpt)
