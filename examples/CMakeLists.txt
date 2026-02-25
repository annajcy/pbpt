add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)

# Unified cbox entrypoint: renders all cbox example scenes from a single target.
add_executable(cbox cbox.cpp)
target_link_libraries(cbox PRIVATE pbpt)

add_executable(image_io image_io.cpp)
target_link_libraries(image_io PRIVATE pbpt)

add_executable(pbpt_cli pbpt_cli.cpp)
target_link_libraries(pbpt_cli PRIVATE pbpt)