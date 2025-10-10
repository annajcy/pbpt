cmake_minimum_required(VERSION 3.27)
project(pbpt LANGUAGES CXX)

message(STATUS "CMake version: ${CMAKE_VERSION}")

option(ENABLE_COMPILE_COMMANDS "Symlink compile_commands.json to build root" ON)
if (ENABLE_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
    file(CREATE_LINK
      "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json"
      "${CMAKE_SOURCE_DIR}/build/compile_commands.json"
      SYMBOLIC)
  endif()
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress duplicate library warnings on macOS
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(PBPT_BUILD_TESTS "Build the project's tests" ON)
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ON)
option(PBPT_BUILD_DOCUMENTATION "Build the Doxygen documentation" OFF)
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
add_subdirectory(src)

if(PBPT_BUILD_EXAMPLES)
  add_subdirectory(examples)
  message(STATUS "Examples are enabled.")
else()
  message(STATUS "Examples are disabled. To enable them, set PBPT_BUILD_EXAMPLES to ON.")
endif()

if(PBPT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
  message(STATUS "Tests are enabled.")
else()
  message(STATUS "Tests are disabled. To enable them, set PBPT_BUILD_TESTS to ON.")
endif()

if(PBPT_BUILD_DOCUMENTATION)
  include(document)
  message(STATUS "Documentation is enabled.")
else()
  message(STATUS "Documentation is disabled. To enable it, set PBPT_BUILD_DOCUMENTATION to ON.")
endif()

