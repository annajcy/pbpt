find_package(Doxygen COMPONENTS dot)

if(DOXYGEN_FOUND)
    set(DOXYFILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")
    message(STATUS "Doxygen found, configured to generate documentation.")
    add_custom_target(pbpt_docs
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/docs/api/doxygen"
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating PBPT API documentation with Doxygen..."
        VERBATIM USES_TERMINAL
    )
else()
    message(WARNING "Doxygen not found. Documentation target will not be created. Try to install Doxygen and graphviz.")
endif()