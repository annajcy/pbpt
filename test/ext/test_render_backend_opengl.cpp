#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <gtest/gtest.h>

// 宏定义和头文件包含顺序很重要
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glbinding/glbinding.h>
#include <glbinding/gl/gl.h>

using namespace gl;

// --- 测试固件 (Test Fixture) ---
// 这个类封装了所有测试共享的资源和设置/清理逻辑
class OpenGLTest : public ::testing::Test {
protected:
    GLFWwindow* window = nullptr;
    GLuint shaderProgram = 0;
    GLuint VAO = 0, VBO = 0;

    // 在每个测试用例开始前运行
    void SetUp() override {
        // --- 1. 初始化 GLFW ---
        glfwSetErrorCallback([](int error, const char* description) {
            FAIL() << "GLFW Error " << error << ": " << description;
        });
        ASSERT_TRUE(glfwInit());

        // 设置 OpenGL 3.3 Core Profile
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, 1);
#endif
        // 在测试中，我们通常不需要一个可见的窗口
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

        // --- 2. 创建窗口和上下文 ---
        window = glfwCreateWindow(800, 600, "OpenGL Google Test", nullptr, nullptr);
        ASSERT_NE(window, nullptr) << "Failed to create GLFW window";
        glfwMakeContextCurrent(window);

        // --- 3. 初始化 glbinding ---
        glbinding::initialize(glfwGetProcAddress);

        // --- 4. 构建和编译着色器程序 ---
        CreateShaderProgram();
        ASSERT_NE(shaderProgram, 0);

        // --- 5. 设置顶点数据和缓冲区 ---
        CreateVertexBuffers();
        ASSERT_NE(VAO, 0);
        ASSERT_NE(VBO, 0);
    }

    // 在每个测试用例结束后运行
    void TearDown() override {
        // --- 7. 清理资源 ---
        if (shaderProgram != 0) glDeleteProgram(shaderProgram);
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
        
        if (window != nullptr) glfwDestroyWindow(window);
        glfwTerminate();
    }

    // 辅助函数：创建着色器
    void CreateShaderProgram() {
        const char* vertexShaderSource = R"(#version 330 core
            layout (location = 0) in vec3 aPos;
            void main() { gl_Position = vec4(aPos, 1.0); })";
        const char* fragmentShaderSource = R"(#version 330 core
            out vec4 FragColor;
            void main() { FragColor = vec4(1.0, 0.0, 1.0, 1.0); })"; // Simple magenta color

        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);
        // 实际项目中应检查编译错误

        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);
        // 实际项目中应检查编译错误

        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);
        // 实际项目中应检查链接错误

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    // 辅助函数：创建顶点缓冲
    void CreateVertexBuffers() {
        std::vector<float> vertices = { 0.5f, -0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.0f, 0.5f, 0.0f };
        
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
};

// --- 测试用例 (Test Case) ---

// 测试：初始化和资源创建是否能无错误地完成
// 这个测试实际上是隐式地通过 SetUp() 的成功执行来验证的
TEST_F(OpenGLTest, InitializationSucceeds) {
    // 如果 SetUp() 成功运行到这里并且没有触发 ASSERT 失败，
    // 就证明初始化是成功的。
    SUCCEED();
}

// 测试：渲染单帧是否会产生 OpenGL 错误
TEST_F(OpenGLTest, RenderSingleFrameWithoutErrors) {
    // 确保开始时没有错误
    ASSERT_EQ(glGetError(), GL_NO_ERROR);

    // b. 渲染
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // 激活着色器程序
    glUseProgram(shaderProgram);

    // c. 创建变换矩阵并发送到 uniform (在 GTest 中可以简化或固定)
    // 在这个测试中，我们甚至可以不设置 uniform，因为着色器已简化
    
    // 绘制三角形
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    // d. 交换缓冲区 (在无头测试中仍然需要，以完成绘制命令)
    glfwSwapBuffers(window);
    
    // a. 处理事件 (确保命令被处理)
    glfwPollEvents();

    // 检查在所有渲染命令之后是否产生了任何 OpenGL 错误
    EXPECT_EQ(glGetError(), GL_NO_ERROR) << "OpenGL error occurred during rendering.";
}


TEST_F(OpenGLTest, RenderedPixelIsCorrectColor) {
    // 1. 轮询事件以确保窗口和上下文完全准备就绪
    glfwPollEvents();

    // 2. 获取帧缓冲区的实际尺寸
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    //  sanity check: 确保我们有一个有效的绘图表面
    ASSERT_GT(width, 0) << "Framebuffer width is zero.";
    ASSERT_GT(height, 0) << "Framebuffer height is zero.";
    
    // 3. 【关键修复】设置视口！
    // 告诉 OpenGL 渲染区域的大小，这必须在绘图前完成。
    glViewport(0, 0, width, height);

    // 4. 渲染场景
    glClearColor(1.0f, 1.0f, 0.0f, 1.0f); // 黑色背景
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    
    // 5. 【关键修复】明确指定从后台缓冲区读取
    // 因为我们的绘图命令是发送到后台缓冲区的
    glReadBuffer(GL_BACK);
    
    // 6. 确保所有命令在读取像素前完成
    glFinish();

    // 7. 读取像素
    int read_x = width / 2;
    int read_y = static_cast<int>(height * 0.75); // 三角形顶部附近
    unsigned char pixel[4] = {0}; // 初始化为0

    glReadPixels(read_x, read_y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

    // 8. 检查OpenGL在此过程中是否出错
    GLenum err = glGetError();
    //ASSERT_EQ(err, GL_NO_ERROR) << "OpenGL error after glReadPixels: " << err;

    // 9. 验证像素颜色
    EXPECT_EQ(pixel[0], 255) << "Red channel";
    EXPECT_EQ(pixel[1], 255)   << "Green channel";
    EXPECT_EQ(pixel[2], 0) << "Blue channel";
    EXPECT_EQ(pixel[3], 255) << "Alpha channel";
}