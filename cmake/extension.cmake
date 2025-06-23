# --- 测试框架 (只有在需要时才获取) ---
if(PBPT_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
  )
  # 使用普通 set，而非 CACHE，确保每次配置时都能应用这些设置
  set(gtest_build_samples OFF)
  set(gtest_build_tests OFF)
  FetchContent_MakeAvailable(googletest)
else()
  message(STATUS "PBPT_BUILD_TESTS is OFF")
endif()

# --------------------------------------------------------------------
# 下载并解压 Slang SDK (这部分逻辑很棒，保持不变)
# --------------------------------------------------------------------
set(SLANG_VERSION "2025.10.4")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(SLANG_DIR_NAME "slang-${SLANG_VERSION}-linux-x86_64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(SLANG_DIR_NAME "slang-${SLANG_VERSION}-macos-aarch64")
  else()
    set(SLANG_DIR_NAME "slang-${SLANG_VERSION}-macos-x86_64")
  endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(SLANG_DIR_NAME "slang-${SLANG_VERSION}-windows-x86_64")
else()
  message(FATAL_ERROR "Unsupported platform for automatic Slang SDK download: ${CMAKE_SYSTEM_NAME}")
endif()
set(SLANG_DOWNLOAD_URL "https://github.com/shader-slang/slang/releases/download/v${SLANG_VERSION}/${SLANG_DIR_NAME}.zip")
FetchContent_Declare(slang_dep URL ${SLANG_DOWNLOAD_URL})
FetchContent_Populate(slang_dep)
FetchContent_GetProperties(slang_dep)
if(NOT slang_dep_SOURCE_DIR)
    message(FATAL_ERROR "Could not get source directory for slang dependency after population.")
endif()
find_package(slang ${SLANG_VERSION} REQUIRED PATHS "${slang_dep_SOURCE_DIR}" NO_DEFAULT_PATH)
if(slang_FOUND)
    message(STATUS "Successfully found Slang ${slang_VERSION} via FetchContent.")
endif()

# --- GLFW ---
FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS    OFF)
set(GLFW_BUILD_DOCS     OFF)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
  zlib                           # 内容的名称，可以自定义
  GIT_REPOSITORY https://github.com/madler/zlib.git
  GIT_TAG        v1.3.1          # 指定一个具体的 Git 标签或提交哈希
)
set(ZLIB_BUILD_EXAMPLES OFF)
FetchContent_MakeAvailable(zlib)

# --- Assimp ---
FetchContent_Declare(assimp GIT_REPOSITORY https://github.com/assimp/assimp.git GIT_TAG v5.4.1)
set(ASSIMP_BUILD_ZLIB OFF CACHE BOOL "Do not build Assimp's bundled zlib")
set(ASSIMP_BUILD_TESTS OFF)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
set(ASSIMP_INSTALL OFF)
FetchContent_MakeAvailable(assimp)

# --- ImGui ---
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.91.9b)
# IMGUI_BUILD_EXAMPLES 是 imgui 内部的选项，无需设置，FetchContent_MakeAvailable 会处理
FetchContent_MakeAvailable(imgui)

# 因为它有自己的源文件需要编译。
add_library(imgui_lib STATIC) # 使用一个不与 FetchContent 目标冲突的新名字

# ImGui 的源文件是其内部实现，使用 PRIVATE
target_sources(imgui_lib PRIVATE
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
)

# ImGui 的头文件目录需要被链接到它的目标所包含，使用 PUBLIC
target_include_directories(imgui_lib PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

# 为 ImGui 添加后端相关的源文件和依赖
if(RENDER_BACKEND STREQUAL "Vulkan")
  target_sources(imgui_lib PRIVATE ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp)
  # ImGui 的使用者也需要链接 Vulkan 和 glfw，所以用 PUBLIC
  target_link_libraries(imgui_lib PUBLIC Vulkan::Vulkan glfw)
elseif(RENDER_BACKEND STREQUAL "OpenGL")
  target_sources(imgui_lib PRIVATE ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  # ImGui 的使用者也需要链接 OpenGL 和 glfw，所以用 PUBLIC
  target_link_libraries(imgui_lib PUBLIC OpenGL::GL glfw)
endif()

# --- STB ---
FetchContent_Declare(stb GIT_REPOSITORY https://github.com/nothings/stb.git GIT_TAG master)
FetchContent_MakeAvailable(stb)
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${stb_SOURCE_DIR})

# --- GLM (OpenGL Mathematics) ---
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1)
FetchContent_MakeAvailable(glm)
# glm 目标已由 FetchContent_MakeAvailable 创建，它是一个 INTERFACE 库，可以直接使用

# ====================================================================
#  将所有依赖项聚合到列表中，供主目标使用
# ====================================================================
# 通用第三方库
list(APPEND ext_lib
  imgui_lib      # 使用我们新创建的、配置正确的 imgui 目标
  glm::glm       # 最好使用由 FetchContent 创建的带命名空间的目标名
  stb
  assimp::assimp # 使用由 FetchContent 创建的带命名空间的目标名
  slang::slang
)

# 测试专用库
if (PBPT_BUILD_TESTS) 
  list(APPEND ext_lib
    GTest::gtest_main # 使用由 FetchContent 创建的带命名空间的目标名
  )
endif()

list(APPEND ext_move_lib slang::slang)