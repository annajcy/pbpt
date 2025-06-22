add_executable(
  test_vulkan
  test_vulkan.cpp
)

target_link_libraries(test_vulkan PRIVATE 
  ${pbpt_lib}
  ${ext_lib}
  ${render_backend}
)

add_runtime_dependency_copy_step(test_vulkan slang::slang)
