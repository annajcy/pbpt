add_executable(render_backend render_backend.cpp)
target_link_libraries(render_backend PRIVATE pbpt)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)
