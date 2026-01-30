add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

add_executable(cbox cbox.cpp)
target_link_libraries(cbox PRIVATE pbpt)

add_executable(cbox_cu_spec cbox_cu_spec.cpp)
target_link_libraries(cbox_cu_spec PRIVATE pbpt)

add_executable(cbox_diele_spec cbox_diele_spec.cpp)
target_link_libraries(cbox_diele_spec PRIVATE pbpt)

add_executable(cbox_rough cbox_rough.cpp)
target_link_libraries(cbox_rough PRIVATE pbpt)

add_executable(image_io image_io.cpp)
target_link_libraries(image_io PRIVATE pbpt)
