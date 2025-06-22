#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <cstring>

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

int main() {
    HelloTriangleApplication app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 0;
    }

    return 0;
}