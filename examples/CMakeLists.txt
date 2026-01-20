add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

add_executable(cbox cbox.cpp)
target_link_libraries(cbox PRIVATE pbpt)

add_executable(image_io image_io.cpp)
target_link_libraries(image_io PRIVATE pbpt)