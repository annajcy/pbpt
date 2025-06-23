add_executable(
  test_render_backend
  test_render_backend.cpp
)

target_link_libraries(test_render_backend PRIVATE 
  ${pbpt_lib}
  ${ext_lib}
  ${render_backend}
)

add_runtime_dependency_copy_step(test_render_backend slang::slang)

gtest_discover_tests(test_render_backend)
