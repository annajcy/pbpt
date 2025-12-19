add_executable(third_party third_party.cpp)
target_link_libraries(third_party PRIVATE pbpt)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

add_executable(simple_scene simple_scene.cpp)
target_link_libraries(simple_scene PRIVATE pbpt)

add_executable(light_scene light_scene.cpp)
target_link_libraries(light_scene PRIVATE pbpt)

add_executable(triangle_scene triangle_scene.cpp)
target_link_libraries(triangle_scene PRIVATE pbpt)

add_executable(cornell_box_scene cornell_box_scene.cpp)
target_link_libraries(cornell_box_scene PRIVATE pbpt)
