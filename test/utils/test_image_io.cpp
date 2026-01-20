#include <gtest/gtest.h>
#include <filesystem>
#include <vector>
#include <cmath>
#include "utils/image_io.hpp"
#include "texture/image.hpp"
#include "math/vector.hpp"

using namespace pbpt;

// Test fixture for file cleanup
class ImageIOTest : public ::testing::Test {
protected:
    std::filesystem::path temp_file;

    void SetUp() override {
        // Generate a unique temp filename
        temp_file = std::filesystem::temp_directory_path() / "test_image.exr";
        if (std::filesystem::exists(temp_file)) {
            std::filesystem::remove(temp_file);
        }
    }

    void TearDown() override {
        if (std::filesystem::exists(temp_file)) {
            std::filesystem::remove(temp_file);
        }
    }
};

TEST_F(ImageIOTest, WriteAndReadBasicColor) {
    int width = 10;
    int height = 10;
    texture::Image<math::Vector<double, 3>> original_img(width, height);

    // Fill with a solid color
    math::Vector<double, 3> color(0.1, 0.5, 0.9);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            original_img.get_pixel(x, y) = color;
        }
    }

    // Write to file
    utils::write_hdr_image(temp_file, original_img);

    // Read back
    ASSERT_TRUE(std::filesystem::exists(temp_file));
    auto loaded_img = utils::read_hdr_image<double>(temp_file);

    // Check dimensions
    ASSERT_EQ(loaded_img.width(), width);
    ASSERT_EQ(loaded_img.height(), height);

    // Check pixels
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto pixel = loaded_img.get_pixel(x, y);
            EXPECT_NEAR(pixel.x(), color.x(), 1e-4);
            EXPECT_NEAR(pixel.y(), color.y(), 1e-4);
            EXPECT_NEAR(pixel.z(), color.z(), 1e-4);
        }
    }
}

TEST_F(ImageIOTest, WriteAndReadGradient) {
    int width = 32;
    int height = 32;
    texture::Image<math::Vector<float, 3>> original_img(width, height);

    // Fill with gradient
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float u = static_cast<float>(x) / (width - 1);
            float v = static_cast<float>(y) / (height - 1);
            original_img.get_pixel(x, y) = math::Vector<float, 3>(u, v, 1.0f - u);
        }
    }

    // Write to file
    utils::write_hdr_image(temp_file, original_img);

    // Read back (as double to test type conversion if needed)
    auto loaded_img = utils::read_hdr_image<float>(temp_file);

    ASSERT_EQ(loaded_img.width(), width);
    ASSERT_EQ(loaded_img.height(), height);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float u = static_cast<float>(x) / (width - 1);
            float v = static_cast<float>(y) / (height - 1);
            math::Vector<float, 3> expected(u, v, 1.0f - u);
            
            auto pixel = loaded_img.get_pixel(x, y);
            EXPECT_NEAR(pixel.x(), expected.x(), 1e-4);
            EXPECT_NEAR(pixel.y(), expected.y(), 1e-4);
            EXPECT_NEAR(pixel.z(), expected.z(), 1e-4);
        }
    }
}

TEST_F(ImageIOTest, WriteAndReadPNG) {
    int width = 16;
    int height = 16;
    texture::Image<math::Vector<float, 3>> original_img(width, height);
    
    // Fill with simple patterns (Red, Green, Blue corners)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (x < width/2 && y < height/2) original_img.get_pixel(x, y) = math::Vector<float, 3>(1, 0, 0); // Red
            else if (x >= width/2 && y < height/2) original_img.get_pixel(x, y) = math::Vector<float, 3>(0, 1, 0); // Green
            else original_img.get_pixel(x, y) = math::Vector<float, 3>(0, 0, 1); // Blue
        }
    }

    std::filesystem::path png_path = std::filesystem::temp_directory_path() / "test_image.png";
    
    // Use generic read/write or specific
    utils::write_ldr_image(png_path, original_img);
    
    ASSERT_TRUE(std::filesystem::exists(png_path));
    
    // Read back using generic read_image
    auto loaded_img = utils::read_image<float>(png_path);
    
    ASSERT_EQ(loaded_img.width(), width);
    ASSERT_EQ(loaded_img.height(), height);
    
    // Check approximate values (8-bit quantization introduces error)
    // 1.0 -> 255 -> 1.0. Error should be small (1/255 approx 0.004)
    float tolerance = 0.01f;
    
    auto p1 = loaded_img.get_pixel(0, 0); // Red
    EXPECT_NEAR(p1.x(), 1.0f, tolerance);
    EXPECT_NEAR(p1.y(), 0.0f, tolerance);
    EXPECT_NEAR(p1.z(), 0.0f, tolerance);

    auto p2 = loaded_img.get_pixel(width-1, 0); // Green
    EXPECT_NEAR(p2.x(), 0.0f, tolerance);
    EXPECT_NEAR(p2.y(), 1.0f, tolerance);
    EXPECT_NEAR(p2.z(), 0.0f, tolerance);
    
    std::filesystem::remove(png_path);
}

TEST_F(ImageIOTest, DispatchWriteRead) {
    int width = 8;
    int height = 8;
    texture::Image<math::Vector<float, 3>> original_img(width, height);
    
    // Fill with solid color
    math::Vector<float, 3> color(0.2f, 0.4f, 0.6f);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            original_img.get_pixel(x, y) = color;
        }
    }

    // 1. Test .exr dispatch
    {
        auto exr_path = std::filesystem::temp_directory_path() / "dispatch_test.exr";
        utils::write_image(exr_path, original_img);
        ASSERT_TRUE(std::filesystem::exists(exr_path));
        
        auto loaded = utils::read_image<float>(exr_path);
        EXPECT_NEAR(loaded.get_pixel(0,0).x(), color.x(), 1e-4);
        std::filesystem::remove(exr_path);
    }
    
    // 2. Test .png dispatch
    {
        auto png_path = std::filesystem::temp_directory_path() / "dispatch_test.png";
        utils::write_image(png_path, original_img);
        ASSERT_TRUE(std::filesystem::exists(png_path));
        
        auto loaded = utils::read_image<float>(png_path);
        // PNG is 8-bit quantized, so error margin is larger
        EXPECT_NEAR(loaded.get_pixel(0,0).x(), color.x(), 0.01f);
        std::filesystem::remove(png_path);
    }

    // 3. Test Unsupported
    {
        auto bad_path = std::filesystem::temp_directory_path() / "test.unsupported";
        EXPECT_THROW(
            utils::write_image(bad_path, original_img),
            std::runtime_error
        );
    }
}

TEST_F(ImageIOTest, ReadNonExistentFile) {
    EXPECT_THROW(utils::read_hdr_image<double>("non_existent_file.exr"), std::runtime_error);
}
