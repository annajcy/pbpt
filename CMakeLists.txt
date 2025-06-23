cmake_minimum_required(VERSION 3.28)
project(pbpt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)

option(PBPT_BUILD_TESTS "Build the project's tests" ON)
option(BUILD_DOCUMENTATION "Build the Doxygen documentation" ON)
option(PBPT_BUILD_EXAMPLES "Build the project's examples" ON)

# ====================================================================
# 全局构建选项和配置
# ====================================================================
include(platform)
include(extension) 
include(utils)

# ====================================================================
# 添加项目源代码
# ====================================================================
add_subdirectory(src)

# ====================================================================
# 测试构建选项
# ====================================================================
if(PBPT_BUILD_TESTS)
  # 为测试启用 CTest
  enable_testing()
  add_subdirectory(test)
  message(STATUS "Tests are enabled.")
else()
  message(STATUS "Tests are disabled. To enable them, set PBPT_BUILD_TESTS to ON.")
endif()


# ====================================================================
# 文档生成
# ====================================================================
if(BUILD_DOCUMENTATION)
  add_subdirectory(docs)
endif()