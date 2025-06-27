#include <iostream>
#include "assimp/Importer.hpp"
#include "math/bounding_box.hpp"
#include "math/point.hpp"

#include "math/global.hpp"
#include "math/vector.hpp"
#include "slang.h"
#include "imgui.h"
#include <vector>

#include "math/matrix.hpp"


#if defined (RENDER_BACKEND_VULKAN)
    #include "vulkan/vulkan.h"
    #include "vulkan/vulkan_core.h"
    #include "vulkan/vulkan.hpp"
    #include "imgui_impl_vulkan.h"
#endif

#if defined (RENDER_BACKEND_OPENGL)

#endif

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace pbpt;

int main() {

    math::Pt3 p(1.0, 2.0, 3.0);
    std::cout << "p: " << p << std::endl;
    std::cout << static_cast<int>(slang::BindingType::BaseMask) << std::endl;

    ImGui::CreateContext();
    std::cout << ImGui::GetVersion() << std::endl;

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

    using namespace math;

    Mat2 m(Vec2(4, 7), Vec2(2, 6)); // Determinant is 10
    std::cout << "Determinant: " << m.determinant() << std::endl;
    Mat2 m_inv = m.inversed();
    std::cout << "Inverse: " << m_inv << std::endl;
    Mat2 identity = m * m_inv;
    std::cout << "Identity: " << identity << std::endl;
    // Test singular matrix exception
    Mat2 singular(Vec2(2, 4), Vec2(2, 4));
    std::cout << "Determinant: " << singular.determinant() << std::endl;
    //singular.inverse();

    Mat4 m1 = Mat4::identity();
    m1[0][1] = 5; m1[0][2] = 6;
    m1[1][1] = 7; m1[1][2] = 8;

    std::cout << "m1: " << m1 << std::endl;
    
    // Test contiguous submatrix
    Mat2 sub = m1.view<2, 2>(0, 1).to_matrix();
    std::cout << "Submatrix: " << sub << std::endl;

    Mat2 d(
        1, 2, 
        3, 4
    );

    for (int i = 0; i < 2; i ++) {
        for (int j = 0; j < 2; j ++)
            std::cout << d[i][j] << " ";
        std::cout << std::endl;
    }
        
    std::cout << std::endl;

    Bound3 box;

    box.unite(Pt3(1, 2, 3)).unite(Pt3(4, 5, 6)).unite(Pt3(7, 8, 9)).unite(Bound3{Pt3(10, 11, 12), Pt3(13, 14, 15)});

    std::cout << "box: " << box << std::endl;
    std::cout << "is_contain: " << box.contains(Pt3(4, 5, 6)) << std::endl;

    std::cout << (Pt3(4.0, 7.0, 6.0) <= Pt3(4.0, 5.0, 6.0))<< std::endl;
    std::cout << box.center() << std::endl;

    return 0;
}