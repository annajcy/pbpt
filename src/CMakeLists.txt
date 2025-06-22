add_subdirectory(math)

add_executable(
  pbpt
  main.cpp
)

list(APPEND pbpt_lib
  pbpt_options
  pbpt_math 
)

target_link_libraries(pbpt PRIVATE 
  ${pbpt_lib}
  ${ext_lib}
  ${render_backend}
)

add_runtime_dependency_copy_step(pbpt slang::slang)