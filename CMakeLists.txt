cmake_minimum_required(VERSION 3.27)
project(pbpt VERSION 0.1.0 LANGUAGES CXX)

message(STATUS "CMake version: ${CMAKE_VERSION}")

option(ENABLE_COMPILE_COMMANDS "Copy compile_commands.json to build root" ${PROJECT_IS_TOP_LEVEL})
if (ENABLE_COMPILE_COMMANDS AND PROJECT_IS_TOP_LEVEL)
  set(_SRC "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
  set(_DST "${PROJECT_SOURCE_DIR}/build/compile_commands.json")

  add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_SRC}" "${_DST}"
    COMMENT "Copying compile_commands.json -> ${_DST}"
    VERBATIM
  )

  message(STATUS "compile_commands.json will be copied to build root.")
elseif (ENABLE_COMPILE_COMMANDS)
  message(STATUS "compile_commands copy is skipped for embedded PBPT builds.")
else()
  message(STATUS "compile_commands.json will NOT be copied to build root.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress duplicate library warnings on macOS
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(PBPT_BUILD_TESTS "Build the project's tests" ${PROJECT_IS_TOP_LEVEL})
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ${PROJECT_IS_TOP_LEVEL})
option(PBPT_BUILD_DOCUMENTATION "Build the Doxygen documentation" ${PROJECT_IS_TOP_LEVEL})
option(PBPT_FLOAT_64BIT "Use double precision floating point" OFF)
option(PBPT_INT_64BIT "Use 64-bit integer type" OFF)

if(PBPT_FLOAT_64BIT)
  add_definitions("-DPBPT_FLOAT_64BIT")
  message(STATUS "Using double precision floating point.")
else()
  message(STATUS "Using single precision floating point. To enable double precision, set PBPT_FLOAT_64BIT to ON.")
endif()

if(PBPT_INT_64BIT)
  add_definitions("-DPBPT_INT_64BIT")
  message(STATUS "Using 64-bit integer type.")
else()
  message(STATUS "Using 32-bit integer type. To enable 64-bit integer type, set PBPT_INT_64BIT to ON.")
endif()

include(third_party) 
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
add_subdirectory(src)

set(PBPT_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/pbpt")

install(
  TARGETS pbpt pbpt_rgb_spectrum_lut pbpt_stb_impl
  EXPORT pbptTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pbptConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/pbptConfig.cmake
  INSTALL_DESTINATION ${PBPT_CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/pbptConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/pbptConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/pbptConfigVersion.cmake
  DESTINATION ${PBPT_CONFIG_INSTALL_DIR}
)
install(
  EXPORT pbptTargets
  NAMESPACE pbpt::
  DESTINATION ${PBPT_CONFIG_INSTALL_DIR}
)

if(PBPT_BUILD_EXAMPLES AND PROJECT_IS_TOP_LEVEL)
  add_subdirectory(examples)
  message(STATUS "Examples are enabled.")
elseif(PBPT_BUILD_EXAMPLES)
  message(STATUS "Examples are disabled for embedded PBPT builds.")
else()
  message(STATUS "Examples are disabled. To enable them, set PBPT_BUILD_EXAMPLES to ON.")
endif()

if(PBPT_BUILD_TESTS AND PROJECT_IS_TOP_LEVEL)
  enable_testing()
  add_subdirectory(test)
  message(STATUS "Tests are enabled.")
elseif(PBPT_BUILD_TESTS)
  message(STATUS "Tests are disabled for embedded PBPT builds.")
else()
  message(STATUS "Tests are disabled. To enable them, set PBPT_BUILD_TESTS to ON.")
endif()

if(PBPT_BUILD_DOCUMENTATION AND PROJECT_IS_TOP_LEVEL)
  include(document)
  message(STATUS "Documentation is enabled.")
elseif(PBPT_BUILD_DOCUMENTATION)
  message(STATUS "Documentation is disabled for embedded PBPT builds.")
else()
  message(STATUS "Documentation is disabled. To enable it, set PBPT_BUILD_DOCUMENTATION to ON.")
endif()
