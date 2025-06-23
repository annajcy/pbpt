#include <iostream>
#include "assimp/Importer.hpp"
#include "math/point.hpp"

#include "slang.h"
#include "imgui.h"
#include <vector>

#if defined (RENDER_BACKEND_VULKAN)
    #include "vulkan/vulkan.h"
    #include "vulkan/vulkan_core.h"
    #include "vulkan/vulkan.hpp"
    #include "imgui_impl_vulkan.h"
#endif

#if defined (RENDER_BACKEND_OPENGL)

#endif

#include "glm/glm.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace pbpt;

int main() {

    math::Point3 p(1.0, 2.0, 3.0);
    std::cout << "p: " << p << std::endl;
    std::cout << static_cast<int>(slang::BindingType::BaseMask) << std::endl;

    ImGui::CreateContext();
    std::cout << ImGui::GetVersion() << std::endl;

    glm::vec3 v(1.0, 2.0, 3.0);
    std::cout << v.x << std::endl;
    std::cout << v.y << std::endl;
    std::cout << v.z << std::endl;
    std::cout << glm::dot(v, v) << std::endl;

    const int width_write = 100;
    const int height_write = 100;
    const int channels_write = 3; // RGB

    // 创建一个图像数据缓冲区
    // 使用 std::vector 来自动管理内存
    std::vector<unsigned char> image_data(width_write * height_write * channels_write);

    // 填充图像数据 (生成一个红色的图像)
    for (int y = 0; y < height_write; ++y) {
        for (int x = 0; x < width_write; ++x) {
            // 计算像素在缓冲区中的索引
            size_t index = (y * width_write + x) * channels_write;
            // 设置像素颜色 (R=255, G=0, B=0)
            image_data[index + 0] = 255; // R
            image_data[index + 1] = 0;   // G
            image_data[index + 2] = 0;   // B
        }
    }

    // 设置 stride (每行数据的字节数)
    int stride_in_bytes = width_write * channels_write;

    // 写入 PNG 文件
    // stbi_write_png(文件名, 宽, 高, 通道数, 数据指针, 每行字节数)
    int success = stbi_write_png("test.png", width_write, height_write, channels_write, image_data.data(), stride_in_bytes);

    if (success) {
        std::cout << "成功写入图像到 test.png" << std::endl;
    } else {
        std::cerr << "错误: 写入图像失败" << std::endl;
        return 1; // 返回错误码
    }

    std::cout << "\n--- 开始读取 PNG 文件 ---\n" << std::endl;

    // --- 读取 PNG 文件 ---

    int width_read, height_read, channels_read;

    // stbi_load(文件名, &宽, &高, &每个像素的组件数, 强制的组件数)
    // 最后一个参数为 0 表示使用文件自身的通道数
    unsigned char* data = stbi_load("test.png", &width_read, &height_read, &channels_read, 0);

    // 检查加载是否成功
    if (data == nullptr) {
        std::cerr << "错误: 加载图像失败" << std::endl;
        // 打印 stb_image 的错误原因
        std::cerr << "STB Reason: " << stbi_failure_reason() << std::endl;
        return 1;
    }

    // 打印加载后的图像信息
    // 注意：data 是一个指向像素数据的指针，直接打印它只会显示内存地址。
    // 只有当 data 为 nullptr 时，打印它才有助于调试。
    std::cout << "data 指针: " << (void*)data << std::endl; // 将指针转换为 void* 以打印地址
    std::cout << "宽度: " << width_read << std::endl;
    std::cout << "高度: " << height_read << std::endl;
    std::cout << "通道数: " << channels_read << std::endl;

    // 释放由 stbi_load 分配的内存
    stbi_image_free(data);

    Assimp::Importer importer{};
    bool is_supported = importer.IsExtensionSupported("obj");
    std::cout << "IsExtensionSupported: " << is_supported << std::endl;

    return 0;
}