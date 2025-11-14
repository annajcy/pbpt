add_executable(third_party third_party.cpp)
target_link_libraries(third_party PRIVATE pbpt)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

add_executable(simple_scene simple_scene.cpp)
target_link_libraries(simple_scene PRIVATE pbpt)

add_executable(spectrum_scene spectrum_scene.cpp)
target_link_libraries(spectrum_scene PRIVATE pbpt)
