// --- src/main.cpp ---

// 使用 glbinding 时，总是先包含 glbinding 的头文件
#include <glbinding/glbinding.h>
#include <glbinding/gl/gl.h>

// 包含 glbinding-aux 以使用辅助功能，例如错误检查回调
#include <glbinding-aux/debug.h>

// 包含 GLFW
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <iostream>

// 使用 glbinding 的命名空间
using namespace gl;

// 定义一个简单的 GLFW 错误回调函数，用于打印错误信息
void error_callback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

int main() {
    // ------------------------------------------------------------------
    // 1. 初始化 GLFW
    // ------------------------------------------------------------------
    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // ------------------------------------------------------------------
    // 2. 创建窗口和 OpenGL 上下文
    // ------------------------------------------------------------------
    // 设置 OpenGL 版本 (这里是 3.3) 和核心模式 (Core Profile)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    #ifdef __APPLE__
        // 在 macOS 上需要这行代码才能使用核心模式
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    #endif

    // 创建窗口
    GLFWwindow* window = glfwCreateWindow(800, 600, "GLFW + glbinding", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // 将窗口的 OpenGL 上下文设置为当前线程的上下文
    glfwMakeContextCurrent(window);

    // ------------------------------------------------------------------
    // 3. 初始化 glbinding (这是关键一步!)
    // ------------------------------------------------------------------
    // 我们使用 glfwGetProcAddress 作为函数指针加载器来初始化 glbinding
    glbinding::initialize(glfwGetProcAddress);

    // (可选但强烈推荐) 启用一个调试回调，当发生 OpenGL 错误时自动打印信息
    glbinding::aux::enableGetErrorCallback();

    std::cout << "OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    // ------------------------------------------------------------------
    // 4. 设置渲染视口和主循环
    // ------------------------------------------------------------------
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    // 主循环 (Game Loop)
    while (!glfwWindowShouldClose(window)) {
        // --- 输入处理 ---
        glfwPollEvents(); // 检查并处理窗口事件，如键盘、鼠标输入

        // --- 渲染指令 ---
        // 设置清屏颜色 (这里是深蓝色)
        glClearColor(0.1f, 0.2f, 0.4f, 1.0f);
        // 清除颜色缓冲区
        glClear(GL_COLOR_BUFFER_BIT);

        // (在这里添加你的其他绘图代码)

        // --- 交换缓冲区并显示 ---
        glfwSwapBuffers(window); // 将后台缓冲区的内容显示到前台
    }

    // ------------------------------------------------------------------
    // 5. 清理和退出
    // ------------------------------------------------------------------
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}