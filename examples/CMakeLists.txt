add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

add_executable(cbox cbox.cpp)
target_link_libraries(cbox PRIVATE pbpt)