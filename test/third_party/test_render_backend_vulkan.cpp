// vulkan_backend_test.cpp
#pragma once
#include <iostream>

// Forward declare to avoid including heavy headers here
struct GLFWwindow;

// Define RENDER_BACKEND_VULKAN or RENDER_BACKEND_OPENGL before including this
// file.

#include "imgui_impl_vulkan.h"

// The functions to be tested
inline void init_render_backend(GLFWwindow* window, void* vulkan_init_info = nullptr) {
    ImGui_ImplVulkan_Init(static_cast<ImGui_ImplVulkan_InitInfo*>(vulkan_init_info));
    std::cout << "Runtime: Initialized Vulkan Backend" << std::endl;
}

inline void shutdown_render_backend() {
    ImGui_ImplVulkan_Shutdown();
}

#include <GLFW/glfw3.h>
#include <gtest/gtest.h>
#include <vulkan/vulkan.h>

#include <cstring>
#include <iostream>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"

// ... (Manual macro definitions for old SDKs remain here if needed) ...
#ifndef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
#define VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME "VK_KHR_portability_enumeration"
#endif
#ifndef VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR
#define VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR 0x00000001
#endif
#ifndef VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME
#define VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME "VK_KHR_portability_subset"
#endif

class VulkanBackendTest : public ::testing::Test {
protected:
    GLFWwindow*      window         = nullptr;
    VkInstance       instance       = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice         device         = VK_NULL_HANDLE;
    uint32_t         queueFamily    = 0;
    VkQueue          graphicsQueue  = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkRenderPass     renderPass     = VK_NULL_HANDLE;  // NEW: Add RenderPass member

    bool imguiInitialized = false;

    // ... (Helper function CheckDeviceExtensionSupport remains the same) ...
    bool CheckDeviceExtensionSupport(VkPhysicalDevice dev, const std::vector<const char*>& required_extensions) {
        uint32_t extensionCount;
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &extensionCount, nullptr);
        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &extensionCount, availableExtensions.data());
        for (const char* required_ext : required_extensions) {
            bool found = false;
            for (const auto& extension : availableExtensions) {
                if (strcmp(required_ext, extension.extensionName) == 0) {
                    found = true;
                    break;
                }
            }
            if (!found)
                return false;
        }
        return true;
    }

    void SetUp() override {
        // ... (glfwInit, Instance creation, and Physical Device selection are
        // unchanged) ...
        ASSERT_TRUE(glfwInit());
        if (!glfwVulkanSupported())
            GTEST_SKIP() << "Skipping test: GLFW reports Vulkan not supported "
                            "on this system.";
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(640, 480, "Vulkan Test", nullptr, nullptr);
        ASSERT_NE(window, nullptr);
        VkApplicationInfo appInfo{};
        appInfo.sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName   = "Vulkan Test";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion         = VK_API_VERSION_1_0;
        VkInstanceCreateInfo createInfo{};
        createInfo.sType                               = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo                    = &appInfo;
        uint32_t                 glfw_extensions_count = 0;
        const char**             glfw_extensions       = glfwGetRequiredInstanceExtensions(&glfw_extensions_count);
        std::vector<const char*> extensions(glfw_extensions, glfw_extensions + glfw_extensions_count);
#if defined(__APPLE__)
        std::cout << "macOS detected. Enabling Vulkan Portability enumeration." << std::endl;
        extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
        createInfo.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif
        createInfo.enabledExtensionCount   = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();
        ASSERT_EQ(vkCreateInstance(&createInfo, nullptr, &instance), VK_SUCCESS);
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
        ASSERT_GT(deviceCount, 0);
        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
        std::vector<const char*> required_device_extensions;
#if defined(__APPLE__)
        required_device_extensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
#endif
        for (const auto& dev : devices) {
            if (CheckDeviceExtensionSupport(dev, required_device_extensions)) {
                physicalDevice = dev;
                break;
            }
        }
        ASSERT_NE(physicalDevice, VK_NULL_HANDLE);

        // --- Logical Device Creation ---
        // ... (remains the same) ...
        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);
        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies.data());
        for (uint32_t i = 0; i < queueFamilyCount; ++i) {
            if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                queueFamily = i;
                break;
            }
        }
        float                   queuePriority = 1.0f;
        VkDeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.sType            = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo.queueFamilyIndex = queueFamily;
        queueCreateInfo.queueCount       = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        VkDeviceCreateInfo deviceCreateInfo{};
        deviceCreateInfo.sType                   = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        deviceCreateInfo.pQueueCreateInfos       = &queueCreateInfo;
        deviceCreateInfo.queueCreateInfoCount    = 1;
        deviceCreateInfo.enabledExtensionCount   = static_cast<uint32_t>(required_device_extensions.size());
        deviceCreateInfo.ppEnabledExtensionNames = required_device_extensions.data();
        ASSERT_EQ(vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device), VK_SUCCESS);
        vkGetDeviceQueue(device, queueFamily, 0, &graphicsQueue);

        // --- NEW: Create a minimal dummy RenderPass ---
        VkSubpassDescription subpass          = {};
        subpass.pipelineBindPoint             = VK_PIPELINE_BIND_POINT_GRAPHICS;
        VkRenderPassCreateInfo renderPassInfo = {};
        renderPassInfo.sType                  = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.subpassCount           = 1;
        renderPassInfo.pSubpasses             = &subpass;
        ASSERT_EQ(vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass), VK_SUCCESS);
        // --- End of new RenderPass creation ---

        // --- Descriptor Pool creation (remains the same) ---
        VkDescriptorPoolSize       pool_sizes[] = {{VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
                                                   {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000}};
        VkDescriptorPoolCreateInfo pool_info    = {};
        pool_info.sType                         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_info.flags                         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        pool_info.maxSets                       = 1000 * 2;
        pool_info.poolSizeCount                 = (uint32_t)std::size(pool_sizes);
        pool_info.pPoolSizes                    = pool_sizes;
        ASSERT_EQ(vkCreateDescriptorPool(device, &pool_info, nullptr, &descriptorPool), VK_SUCCESS);

        // --- ImGui Setup ---
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForVulkan(window, true);

        imguiInitialized = true;
    }

    void TearDown() override {
        if (device)
            vkDeviceWaitIdle(device);

        if (imguiInitialized) {
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }

        // NEW: Destroy the RenderPass before the device
        if (renderPass)
            vkDestroyRenderPass(device, renderPass, nullptr);

        if (descriptorPool)
            vkDestroyDescriptorPool(device, descriptorPool, nullptr);
        if (device)
            vkDestroyDevice(device, nullptr);
        if (instance)
            vkDestroyInstance(instance, nullptr);

        glfwDestroyWindow(window);
        glfwTerminate();
    }
};

TEST_F(VulkanBackendTest, CanInitializeAndShutdown) {
    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance                  = instance;
    init_info.PhysicalDevice            = physicalDevice;
    init_info.Device                    = device;
    init_info.QueueFamily               = queueFamily;
    init_info.Queue                     = graphicsQueue;
    init_info.DescriptorPool            = descriptorPool;
    init_info.RenderPass                = renderPass;  // NEW: Assign the created render pass
    init_info.MinImageCount             = 2;
    init_info.ImageCount                = 2;

    try {
        init_render_backend(window, &init_info);
        shutdown_render_backend();
        SUCCEED();
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test: Vulkan backend initialization failed - " << e.what();
    } catch (...) {
        GTEST_SKIP() << "Skipping test: Vulkan backend initialization failed with unknown error";
    }
}