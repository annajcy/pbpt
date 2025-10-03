add_executable(third_party third_party.cpp)
target_link_libraries(third_party PRIVATE pbpt)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pbpt)
