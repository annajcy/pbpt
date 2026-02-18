cmake_minimum_required(VERSION 3.27)
project(pbpt VERSION 0.1.0 LANGUAGES CXX)

message(STATUS "CMake version: ${CMAKE_VERSION}")

set(PBPT_DEFAULT_ENABLE_COMPILE_COMMANDS ${PROJECT_IS_TOP_LEVEL})
option(ENABLE_COMPILE_COMMANDS "Copy compile_commands.json to build root" ${PBPT_DEFAULT_ENABLE_COMPILE_COMMANDS})
if (ENABLE_COMPILE_COMMANDS AND PROJECT_IS_TOP_LEVEL)
  set(_SRC "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
  set(_DST "${PROJECT_SOURCE_DIR}/build/compile_commands.json")

  add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_SRC}" "${_DST}"
    COMMENT "Copying compile_commands.json -> ${_DST}"
    VERBATIM
  )

  message(STATUS "ENABLE_COMPILE_COMMANDS is ON; compile_commands.json will be copied to build root.")
elseif (ENABLE_COMPILE_COMMANDS)
  message(STATUS "ENABLE_COMPILE_COMMANDS is ON; compile_commands copy target is skipped for embedded PBPT builds.")
else()
  message(STATUS "ENABLE_COMPILE_COMMANDS is OFF; compile_commands.json will NOT be copied to build root.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress duplicate library warnings on macOS
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PBPT_DEFAULT_BUILD_TESTS ${PROJECT_IS_TOP_LEVEL})
set(PBPT_DEFAULT_BUILD_EXAMPLES ${PROJECT_IS_TOP_LEVEL})
set(PBPT_DEFAULT_BUILD_DOCUMENTATION ${PROJECT_IS_TOP_LEVEL})

option(PBPT_BUILD_TESTS "Build the project's tests" ${PBPT_DEFAULT_BUILD_TESTS})
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ${PBPT_DEFAULT_BUILD_EXAMPLES})
option(PBPT_BUILD_DOCUMENTATION "Build the Doxygen documentation" ${PBPT_DEFAULT_BUILD_DOCUMENTATION})
option(PBPT_FLOAT_64BIT "Use double precision floating point" OFF)
option(PBPT_INT_64BIT "Use 64-bit integer type" OFF)

if(PBPT_FLOAT_64BIT)
  add_definitions("-DPBPT_FLOAT_64BIT")
  message(STATUS "PBPT_FLOAT_64BIT is ON; double precision floating point will be used.")
else()
  message(STATUS "PBPT_FLOAT_64BIT is OFF; single precision floating point will be used.")
endif()

if(PBPT_INT_64BIT)
  add_definitions("-DPBPT_INT_64BIT")
  message(STATUS "PBPT_INT_64BIT is ON; 64-bit integer type will be used.")
else()
  message(STATUS "PBPT_INT_64BIT is OFF; 32-bit integer type will be used.")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/third_party.cmake)
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

if(PBPT_BUILD_EXAMPLES)
  add_subdirectory(examples)
  message(STATUS "PBPT_BUILD_EXAMPLES is ON; examples will be built.")
else()
  message(STATUS "PBPT_BUILD_EXAMPLES is OFF; examples will NOT be built.")
endif()

if(PBPT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
  message(STATUS "PBPT_BUILD_TESTS is ON; tests will be built.")
else()
  message(STATUS "PBPT_BUILD_TESTS is OFF; tests will NOT be built.")
endif()

if(PBPT_BUILD_DOCUMENTATION)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/document.cmake)
  message(STATUS "PBPT_BUILD_DOCUMENTATION is ON; documentation will be built.")
else()
  message(STATUS "PBPT_BUILD_DOCUMENTATION is OFF; documentation will NOT be built.")
endif()
