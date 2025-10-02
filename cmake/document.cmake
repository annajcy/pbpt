find_package(Doxygen COMPONENTS dot)

if(DOXYGEN_FOUND)
    set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")
    set(DOXYFILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)
    message(STATUS "Doxygen found, configured to generate documentation.")
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen..."
        VERBATIM USES_TERMINAL
    )
else()
    message(WARNING "Doxygen not found. Documentation target will not be created. Try to install Doxygen and graphviz.")
endif()