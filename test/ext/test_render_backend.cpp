#include <vector>

#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <cstring>

#include "gtest/gtest.h"

#if defined (RENDER_BACKEND_VULKAN) 

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

class HelloTriangleApplication {
private:
    GLFWwindow* m_window{};
    int m_width{800};
    int m_height{600};

    VkInstance m_vk_instance{};
    VkDebugUtilsMessengerEXT m_debug_messenger{};

    const std::vector<const char*> m_validation_layers = {
        "VK_LAYER_KHRONOS_validation"
    };

    bool m_enable_validation_layers{};

public:
    void run() {
        init_window();
        init_vulkan();
        loop();
        clean_up();
    }

private:

    static VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
        VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
        VkDebugUtilsMessageTypeFlagsEXT message_type,
        const VkDebugUtilsMessengerCallbackDataEXT* p_callback_data,
        void* p_user_data
    ) {
        std::cerr << "Validation layer: ";
        
        if (message_type & VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT) {
            std::cerr << "[GENERAL] ";
        }
        if (message_type & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT) {
            std::cerr << "[VALIDATION] ";
        }
        if (message_type & VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT) {
            std::cerr << "[PERFORMANCE] ";
        }
        if (message_severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
            std::cerr << "[ERROR] ";
        }

        std::cerr << std::endl << p_callback_data->pMessage << std::endl;
        
        return VK_FALSE;
    }

    void setup_debug_messenger() {
        if (!m_enable_validation_layers) return;

        VkDebugUtilsMessengerCreateInfoEXT debug_create_info{
            .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,
            .messageSeverity =  VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
            .messageType =  VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
            .pfnUserCallback = debug_callback,
            .pUserData = nullptr
        };

        auto func = (PFN_vkCreateDebugUtilsMessengerEXT) 
            vkGetInstanceProcAddr(m_vk_instance, "vkCreateDebugUtilsMessengerEXT");
        if (func != nullptr) {
            VkResult result = func(
                m_vk_instance, 
                &debug_create_info, 
                nullptr, 
                &m_debug_messenger
            );
            if (result != VK_SUCCESS) {
                throw std::runtime_error("failed to set up debug messenger!");
            }
        } else {
            throw std::runtime_error("failed to find vkCreateDebugUtilsMessengerEXT!");
        }
    }


    void init_window() {
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        m_window = glfwCreateWindow(
            m_width, 
            m_height, 
            "Hello Triangle", 
            nullptr, nullptr
        );
    }

    void init_vulkan() {

#ifdef NDEBUG
        m_enable_validation_layers = false;
#else
        m_enable_validation_layers = true;
#endif

        if (m_enable_validation_layers && !check_validation_layer_support()) {
            throw std::runtime_error("validation layers requested, but not available!");
        }

        if (create_vk_instance() != VK_SUCCESS) {
            throw std::runtime_error("failed to create instance!");
        }

        setup_debug_messenger();
        std::cout << "Vulkan instance created successfully!" << std::endl;
    }

    VkResult create_vk_instance() {
        VkApplicationInfo app_info{
            .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
            .pApplicationName = "Hello Triangle",
            .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
            .pEngineName = "No Engine",
            .engineVersion = VK_MAKE_VERSION(1, 0, 0)
        };

        VkInstanceCreateInfo create_info{
            .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
            .pApplicationInfo = &app_info
        };

        unsigned int glfw_extension_count{};
        const char** glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);

        std::vector<const char*> required_extensions{};
        for (int i = 0; i < glfw_extension_count; i ++) {
            required_extensions.push_back(glfw_extensions[i]);
        }

        required_extensions.emplace_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
        if (m_enable_validation_layers) {
            required_extensions.emplace_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }

        create_info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
        create_info.enabledExtensionCount = required_extensions.size();
        create_info.ppEnabledExtensionNames = required_extensions.data();

        VkDebugUtilsMessengerCreateInfoEXT debug_create_info{};
        if (m_enable_validation_layers) {
            debug_create_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
            debug_create_info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
                                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
            debug_create_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
            debug_create_info.pfnUserCallback = debug_callback;
            create_info.pNext = &debug_create_info;
        } else {
            create_info.pNext = nullptr;
        }

        if (m_enable_validation_layers) {
            create_info.enabledLayerCount = m_validation_layers.size();
            create_info.ppEnabledLayerNames = m_validation_layers.data();
        } else {
            create_info.enabledLayerCount = 0;
            create_info.ppEnabledLayerNames = nullptr;
        }

        VkResult result = vkCreateInstance(
            &create_info, 
            nullptr, 
            &m_vk_instance
        );

        check_extension_properties();
        return result;
    }

    void check_extension_properties() {
        unsigned int extension_count = 0;
        vkEnumerateInstanceExtensionProperties(
            nullptr, 
            &extension_count, 
            nullptr
        );

        std::vector<VkExtensionProperties> properties(extension_count);
        vkEnumerateInstanceExtensionProperties(
            nullptr,
            &extension_count,
            properties.data()
        );

        std::cout << "Available Vulkan extensions: " << extension_count << std::endl;
        for (const auto& property : properties) {
            std::cout << property.extensionName << std::endl;
        }
    }
    
    bool check_validation_layer_support() {
        unsigned int layer_count = 0;
        vkEnumerateInstanceLayerProperties(
            &layer_count,  nullptr
        );
        std::vector<VkLayerProperties> available_layers(layer_count);
        vkEnumerateInstanceLayerProperties(
            &layer_count,
            available_layers.data()
        );
        std::cout << "Available Vulkan layer count: " << layer_count << std::endl;
        for (const auto& layer : available_layers) {
            std::cout << layer.layerName << std::endl;
        }

        for (const auto& layer : m_validation_layers) {
            bool layer_found = false;
            for (const auto& available_layer : available_layers) {
                if (strcmp(layer, available_layer.layerName) == 0) {
                    layer_found = true;
                    break;
                }
            }
            if (!layer_found) {
                std::cerr << "Validation layer not found: " << layer << std::endl;
                return false;
            }
        }
        return true;
    }

    void loop() {
        while (!glfwWindowShouldClose(m_window)) {
            glfwPollEvents();
            break;
        }
    }

    void clean_up() {

        if (m_enable_validation_layers) {
            auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) 
                vkGetInstanceProcAddr(m_vk_instance, "vkDestroyDebugUtilsMessengerEXT");
            if (func != nullptr) {
                func(m_vk_instance, m_debug_messenger, nullptr);
            }
        }

        vkDestroyInstance(m_vk_instance, nullptr);
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }
};

//GOOGLE TEST
GTEST_TEST(HelloTriangleApplication, VULKAN_BACKEND_SUPPORT) {
    HelloTriangleApplication app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        GTEST_SKIP();
    }
    SUCCEED();
}


#endif


#if defined (RENDER_BACKEND_OPENGL)

#include <iostream>
#include <stdexcept> // 用于 std::runtime_error

// --- 依赖项 ---
// 使用 glbinding 时，总是先包含 glbinding 的头文件
#include <glbinding/glbinding.h>
#include <glbinding/gl/gl.h>
// 包含 glbinding-aux 以使用辅助功能，例如错误检查回调
#include <glbinding-aux/debug.h>
// 包含 GLFW
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
// GTest (用于测试)
#include <gtest/gtest.h>


// 使用 glbinding 的命名空间
using namespace gl;

/**
 * @class HelloTriangleApplication
 * @brief 封装了使用 GLFW 和 OpenGL (通过 glbinding) 创建窗口和渲染循环的所有逻辑。
 */
class HelloTriangleApplication {
public:
    /**
     * @brief 运行应用程序的主入口点。
     * * 这个方法会依次调用初始化、主循环和清理函数。
     * 如果在初始化过程中发生错误，会抛出 std::runtime_error 异常。
     */
    void run() {
        init_window_and_context();
        main_loop();
        cleanup();
    }

private:
    /**
     * @brief GLFW 错误回调函数。
     * * 必须是静态成员，因为 GLFW 不知道如何处理 C++ 的 'this' 指针。
     * @param error 错误码。
     * @param description 错误的描述文本。
     */
    static void error_callback(int error, const char* description) {
        std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
    }

    /**
     * @brief 初始化 GLFW、创建窗口和 OpenGL 上下文，并初始化 glbinding。
     */
    void init_window_and_context() {
        // 1. 初始化 GLFW
        glfwSetErrorCallback(error_callback);
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }

        // 2. 创建窗口和 OpenGL 上下文
        // 设置 OpenGL 版本 (这里是 3.3) 和核心模式 (Core Profile)
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        #ifdef __APPLE__
            // 在 macOS 上需要这行代码才能使用核心模式
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        #endif

        // 创建窗口
        m_window = glfwCreateWindow(m_width, m_height, "GLFW + glbinding", nullptr, nullptr);
        if (!m_window) {
            glfwTerminate(); // 在抛出异常前先终止GLFW
            throw std::runtime_error("Failed to create GLFW window");
        }

        // 将窗口的 OpenGL 上下文设置为当前线程的上下文
        glfwMakeContextCurrent(m_window);

        // 3. 初始化 glbinding (这是关键一步!)
        // 我们使用 glfwGetProcAddress 作为函数指针加载器来初始化 glbinding
        glbinding::initialize(glfwGetProcAddress);

        // (可选但强烈推荐) 启用一个调试回调，当发生 OpenGL 错误时自动打印信息
        glbinding::aux::enableGetErrorCallback();

        std::cout << "OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl;
        std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
        std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

        // 4. 设置渲染视口
        int fb_width, fb_height;
        glfwGetFramebufferSize(m_window, &fb_width, &fb_height);
        glViewport(0, 0, fb_width, fb_height);
    }

    /**
     * @brief 应用程序的主循环（渲染循环）。
     */
    void main_loop() {
        while (!glfwWindowShouldClose(m_window)) {
            // --- 输入处理 ---
            glfwPollEvents(); // 检查并处理窗口事件

            // --- 渲染指令 ---
            glClearColor(0.1f, 0.2f, 0.4f, 1.0f); // 设置清屏颜色
            glClear(GL_COLOR_BUFFER_BIT);        // 清除颜色缓冲区

            // (在这里添加你的其他绘图代码)

            // --- 交换缓冲区并显示 ---
            glfwSwapBuffers(m_window); // 将后台缓冲区的内容显示到前台

            break;
        }
    }

    /**
     * @brief 清理资源，销毁窗口并终止 GLFW。
     */
    void cleanup() {
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }

private:
    GLFWwindow* m_window = nullptr; // 指向GLFW窗口的指针，初始化为nullptr
    const int m_width = 800;        // 窗口宽度
    const int m_height = 600;       // 窗口高度
};


// --- 单元测试 ---
// 保持您原有的 GTest 结构不变
GTEST_TEST(HelloTriangleApplication, OPENGL_BACKEND_SUPPORT) {
    HelloTriangleApplication app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        GTEST_SKIP() << "HelloTriangleApplication::run() threw an exception.";
    }
    // 如果没有异常抛出，测试通过
    SUCCEED();
}

#endif // RENDER_BACKEND_OPENGL