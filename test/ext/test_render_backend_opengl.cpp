#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

// 宏定义和头文件包含顺序很重要
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

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
        window = glfwCreateWindow(800, 600, "OpenGL Test Suite", nullptr, nullptr);
        ASSERT_NE(window, nullptr) << "Failed to create GLFW window";
        glfwMakeContextCurrent(window);

        // --- 3. 初始化 GLAD ---
        int gladInitResult = gladLoadGL();
        ASSERT_TRUE(gladInitResult) << "Failed to initialize GLAD";
        
        // 输出OpenGL版本信息
        const GLubyte* version = glGetString(GL_VERSION);
        const GLubyte* renderer = glGetString(GL_RENDERER);
        const GLubyte* vendor = glGetString(GL_VENDOR);
        
        ASSERT_NE(version, nullptr);
        ASSERT_NE(renderer, nullptr);
        ASSERT_NE(vendor, nullptr);
        
        std::cout << "OpenGL Version: " << version << std::endl;
        std::cout << "Renderer: " << renderer << std::endl;
        std::cout << "Vendor: " << vendor << std::endl;

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
        // --- 清理资源 ---
        if (shaderProgram != 0) glDeleteProgram(shaderProgram);
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
        
        if (window != nullptr) glfwDestroyWindow(window);
        glfwTerminate();
    }

    // 辅助函数：检查着色器编译错误
    void CheckShaderCompileErrors(GLuint shader, const std::string& type) {
        GLint success;
        GLchar infoLog[1024];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            FAIL() << "Shader compilation error of type: " << type << "\n" << infoLog;
        }
    }

    // 辅助函数：检查程序链接错误
    void CheckProgramLinkErrors(GLuint program) {
        GLint success;
        GLchar infoLog[1024];
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(program, 1024, NULL, infoLog);
            FAIL() << "Program linking error:\n" << infoLog;
        }
    }

    // 辅助函数：检查OpenGL错误
    void CheckOpenGLError(const std::string& operation) {
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::string message;
            switch (error) {
                case GL_INVALID_ENUM: message = "GL_INVALID_ENUM"; break;
                case GL_INVALID_VALUE: message = "GL_INVALID_VALUE"; break;
                case GL_INVALID_OPERATION: message = "GL_INVALID_OPERATION"; break;
                case GL_OUT_OF_MEMORY: message = "GL_OUT_OF_MEMORY"; break;
                default: message = "Unknown error " + std::to_string(error); break;
            }
            FAIL() << "OpenGL error during " << operation << ": " << message;
        }
    }

    // 辅助函数：创建着色器
    void CreateShaderProgram() {
        const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
void main() {
    gl_Position = vec4(aPos, 1.0);
}
)";
        
        const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(1.0, 0.0, 1.0, 1.0); // 洋红色
}
)";

        // 编译顶点着色器
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);
        CheckShaderCompileErrors(vertexShader, "VERTEX");

        // 编译片段着色器
        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);
        CheckShaderCompileErrors(fragmentShader, "FRAGMENT");

        // 链接着色器程序
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);
        CheckProgramLinkErrors(shaderProgram);

        // 清理着色器对象
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    void CreateVertexBuffers() {
        // 定义一个简单的三角形顶点数据
        std::vector<float> vertices = {
            // 位置坐标 (x, y, z)
             0.0f,  0.5f, 0.0f,  // 顶部顶点
            -0.5f, -0.5f, 0.0f,  // 左下顶点
             0.5f, -0.5f, 0.0f   // 右下顶点
        };
        
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
        
        // 设置顶点属性指针
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        
        // 解绑VAO
        glBindVertexArray(0);
        
        CheckOpenGLError("CreateVertexBuffers");
    }
};

// --- 测试用例 (Test Cases) ---

// 测试：初始化和资源创建是否能无错误地完成
TEST_F(OpenGLTest, InitializationSucceeds) {
    // 如果 SetUp() 成功运行到这里并且没有触发 ASSERT 失败，
    // 就证明初始化是成功的。
    SUCCEED();
}

// 测试：OpenGL上下文信息
TEST_F(OpenGLTest, ContextInformation) {
    // 检查OpenGL版本
    const GLubyte* version = glGetString(GL_VERSION);
    ASSERT_NE(version, nullptr);
    
    // 检查着色器语言版本
    const GLubyte* glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);
    ASSERT_NE(glslVersion, nullptr);
    
    std::cout << "OpenGL Version: " << version << std::endl;
    std::cout << "GLSL Version: " << glslVersion << std::endl;
    
    CheckOpenGLError("ContextInformation");
}

// 测试：渲染单帧是否会产生 OpenGL 错误
TEST_F(OpenGLTest, RenderSingleFrameWithoutErrors) {
    // 确保开始时没有错误
    CheckOpenGLError("BeforeRendering");

    // 设置视口
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    // 清除颜色缓冲区
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // 使用着色器程序
    glUseProgram(shaderProgram);
    
    // 绘制三角形
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    // 交换缓冲区
    glfwSwapBuffers(window);
    
    // 处理事件
    glfwPollEvents();

    // 检查在所有渲染命令之后是否产生了任何 OpenGL 错误
    CheckOpenGLError("AfterRendering");
}

// 测试：验证渲染的像素颜色
TEST_F(OpenGLTest, RenderedPixelColorValidation) {
    // 轮询事件以确保窗口和上下文完全准备就绪
    glfwPollEvents();

    // 获取帧缓冲区的实际尺寸
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    // 确保我们有一个有效的绘图表面
    ASSERT_GT(width, 0) << "Framebuffer width is zero.";
    ASSERT_GT(height, 0) << "Framebuffer height is zero.";
    
    // 设置视口
    glViewport(0, 0, width, height);

    // 渲染场景
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // 黑色背景
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    
    // 指定从后台缓冲区读取
    glReadBuffer(GL_BACK);
    
    // 确保所有命令在读取像素前完成
    glFinish();

    // 读取三角形中心附近的像素
    int read_x = width / 2;
    int read_y = height / 2;
    unsigned char pixel[4] = {0};

    glReadPixels(read_x, read_y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

    // 检查OpenGL在此过程中是否出错
    CheckOpenGLError("PixelReading");

    // 验证像素颜色（洋红色：R=255, G=0, B=255, A=255）
    EXPECT_EQ(pixel[0], 255) << "Red channel should be 255";
    EXPECT_EQ(pixel[1], 0)   << "Green channel should be 0";
    EXPECT_EQ(pixel[2], 255) << "Blue channel should be 255";
    EXPECT_EQ(pixel[3], 255) << "Alpha channel should be 255";
}

// 测试：多次渲染的稳定性
TEST_F(OpenGLTest, MultipleRenderStability) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    
    // 进行多次渲染循环
    for (int i = 0; i < 10; ++i) {
        glClearColor(0.1f * i, 0.2f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        
        // 每次渲染后检查错误
        CheckOpenGLError("MultipleRender_" + std::to_string(i));
    }
}