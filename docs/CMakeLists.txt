find_package(Doxygen COMPONENTS dot)

if(DOXYGEN_FOUND)
    # 设置 Doxyfile.in 模板文件的位置和生成后 Doxyfile 的位置
    set(DOXYFILE_IN "Doxyfile.in")
    set(DOXYFILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

    # 使用 configure_file 命令，根据模板生成最终的配置文件
    # 它会自动替换所有 @VAR@ 占位符
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

    message(STATUS "Doxygen found, configured to generate documentation.")

    # 添加一个自定义目标来生成文档
    # USES_TERMINAL 确保 Doxygen 的输出能正确显示在终端
    # ALL 关键字表示这个目标不会被默认的 `make` 或 `ninja` 命令执行
    # 你必须显式地构建它，例如 `make docs`
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen..."
        VERBATIM USES_TERMINAL
    )
else()
    message(WARNING "Doxygen not found. Documentation target will not be created. Try to install Doxygen and graphviz.")
endif()