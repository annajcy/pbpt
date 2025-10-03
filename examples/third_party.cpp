#include <iostream>
#include <vector>
#include <string>

#include "assimp/Importer.hpp"
#include "imgui.h"
#include "slang.h"
#include "vulkan/vulkan.h"
#include "vulkan/vulkan.hpp"
#include "stb_image.h"
#include "stb_image_write.h"

void test_slang() {
    std::cout << "\n=== Testing Slang Shader Compiler ===" << std::endl;
    std::cout << "Slang BindingType::BaseMask: " << static_cast<int>(slang::BindingType::BaseMask) << std::endl;
    
    slang::IGlobalSession* globalSession = nullptr;
    SlangResult result = slang_createGlobalSession(SLANG_API_VERSION, &globalSession);
    
    if (SLANG_SUCCEEDED(result) && globalSession) {
        std::cout << "✓ Slang global session created successfully" << std::endl;
        globalSession->release();
    } else {
        std::cout << "✗ Failed to create Slang global session" << std::endl;
    }
}

void test_imgui() {
    std::cout << "\n=== Testing ImGui ===" << std::endl;
    ImGuiContext* ctx = ImGui::CreateContext();
    if (ctx) {
        std::cout << "✓ ImGui context created successfully" << std::endl;
        std::cout << "ImGui Version: " << ImGui::GetVersion() << std::endl;
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1920.0f, 1080.0f);
        std::cout << "Display Size: " << io.DisplaySize.x << "x" << io.DisplaySize.y << std::endl;
        ImGuiStyle& style = ImGui::GetStyle();
        std::cout << "Window Rounding: " << style.WindowRounding << std::endl;
        ImGui::DestroyContext(ctx);
        std::cout << "✓ ImGui context destroyed" << std::endl;
    } else {
        std::cout << "✗ Failed to create ImGui context" << std::endl;
    }
}

void test_vulkan() {
    std::cout << "\n=== Testing Vulkan ===" << std::endl;
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "PBPT Backend Test";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "PBPT";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_0;
    
    std::cout << "Application Name: " << appInfo.pApplicationName << std::endl;
    std::cout << "Engine Name: " << appInfo.pEngineName << std::endl;
    std::cout << "API Version: " << VK_VERSION_MAJOR(appInfo.apiVersion) << "." << VK_VERSION_MINOR(appInfo.apiVersion) << "." << VK_VERSION_PATCH(appInfo.apiVersion) << std::endl;
    
    try {
        uint32_t version = vk::enumerateInstanceVersion();
        std::cout << "Vulkan Instance Version: " << VK_VERSION_MAJOR(version) << "." << VK_VERSION_MINOR(version) << "." << VK_VERSION_PATCH(version) << std::endl;
        auto layers = vk::enumerateInstanceLayerProperties();
        std::cout << "Available Vulkan Layers (" << layers.size() << "):" << std::endl;
        for (const auto& layer : layers) {
            std::cout << "  - " << layer.layerName << std::endl;
        }
        auto extensions = vk::enumerateInstanceExtensionProperties();
        std::cout << "Available Vulkan Extensions (" << extensions.size() << "):" << std::endl;
        int count = 0;
        for (const auto& ext : extensions) {
            std::cout << "  - " << ext.extensionName << std::endl;
            if (++count >= 10) {
                std::cout << "  ... and " << (extensions.size() - 10) << " more" << std::endl;
                break;
            }
        }
        std::cout << "✓ Vulkan API accessible" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Vulkan error: " << e.what() << std::endl;
    }
}

void test_stb_image() {
    std::cout << "\n=== Testing STB Image ===" << std::endl;
    const int w = 100, h = 100, ch = 3;
    std::vector<unsigned char> img(w * h * ch);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            size_t idx = (y * w + x) * ch;
            img[idx + 0] = static_cast<unsigned char>(255 * x / w);
            img[idx + 1] = 0;
            img[idx + 2] = static_cast<unsigned char>(255 * y / h);
        }
    }
    if (stbi_write_png("test.png", w, h, ch, img.data(), w * ch)) {
        std::cout << "✓ Successfully wrote image to test.png (" << w << "x" << h << ")" << std::endl;
    } else {
        std::cout << "✗ Failed to write image" << std::endl;
        return;
    }
    int rw, rh, rch;
    unsigned char* data = stbi_load("test.png", &rw, &rh, &rch, 0);
    if (data) {
        std::cout << "✓ Successfully loaded image: " << rw << "x" << rh << ", " << rch << " channels" << std::endl;
        std::cout << "  Top-left pixel: R=" << (int)data[0] << " G=" << (int)data[1] << " B=" << (int)data[2] << std::endl;
        stbi_image_free(data);
    } else {
        std::cout << "✗ Failed to load image: " << stbi_failure_reason() << std::endl;
    }
}

void test_assimp() {
    std::cout << "\n=== Testing Assimp ===" << std::endl;
    Assimp::Importer importer;
    std::vector<std::string> formats = {"obj", "fbx", "dae", "gltf", "glb", "stl", "ply", "3ds"};
    for (const auto& fmt : formats) {
        std::cout << "  " << fmt << ": " << (importer.IsExtensionSupported(fmt) ? "✓ Supported" : "✗ Not supported") << std::endl;
    }
    std::string extensions;
    importer.GetExtensionList(extensions);
    std::cout << "\nAll extensions: " << extensions << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "PBPT Render Backend Library Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    test_slang();
    test_imgui();
    test_vulkan();
    test_stb_image();
    test_assimp();
    std::cout << "\n========================================" << std::endl;
    std::cout << "All tests completed!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
