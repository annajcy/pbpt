#pragma once

#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfInputFile.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include "pbpt/math/point.hpp"
#include "pbpt/texture/image.hpp"

namespace pbpt::utils {

/**
 * @brief Write a floating-point RGB image to an OpenEXR file.
 *
 * @tparam T          Scalar type of the image.
 * @param output_path Target file path for the EXR image.
 * @param image       Source image to write.
 *
 * @throws std::runtime_error if writing the EXR file fails.
 */
template<typename T>
void write_hdr_image(
    const std::filesystem::path& output_path,
    const texture::Image<math::Vector<T, 3>>& image
) {
    int width = image.width();
    int height = image.height();

    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    std::vector<float> buffer(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3
    );

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int flipped_y = height - 1 - y;  // Flip Y axis for EXR coordinate system
            auto rgb = image.get_pixel(x, flipped_y);
            std::size_t idx = (
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                static_cast<std::size_t>(x)
            ) * 3;
            buffer[idx + 0] = static_cast<float>(rgb.x());
            buffer[idx + 1] = static_cast<float>(rgb.y());
            buffer[idx + 2] = static_cast<float>(rgb.z());
        }
    }

    const std::size_t pixel_stride = sizeof(float) * 3;
    const std::size_t row_stride = pixel_stride * static_cast<std::size_t>(width);
    auto* base = reinterpret_cast<char*>(buffer.data());

    Imf::Header header(width, height);
    header.channels().insert("R", Imf::Channel(Imf::FLOAT));
    header.channels().insert("G", Imf::Channel(Imf::FLOAT));
    header.channels().insert("B", Imf::Channel(Imf::FLOAT));

    Imf::FrameBuffer frame_buffer;
    frame_buffer.insert("R", Imf::Slice(Imf::FLOAT, base, pixel_stride, row_stride));
    frame_buffer.insert("G", Imf::Slice(Imf::FLOAT, base + sizeof(float), pixel_stride, row_stride));
    frame_buffer.insert("B", Imf::Slice(Imf::FLOAT, base + sizeof(float) * 2, pixel_stride, row_stride));

    try {
        Imf::OutputFile file(output_path.string().c_str(), header);
        file.setFrameBuffer(frame_buffer);
        file.writePixels(height);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to write image to " + output_path.string() + ": " + e.what());
    }
}

/**
 * @brief Read an RGB image from an OpenEXR file.
 *
 * @tparam T          Scalar type of the output image.
 * @param input_path  Source file path for the EXR image.
 * @return            Image read from file.
 *
 * @throws std::runtime_error if reading the EXR file fails.
 */
template<typename T>
texture::Image<math::Vector<T, 3>> read_hdr_image(const std::filesystem::path& input_path) {
    if (!std::filesystem::exists(input_path)) {
        throw std::runtime_error("File not found: " + input_path.string());
    }

    try {
        Imf::InputFile file(input_path.string().c_str());
        Imath::Box2i dw = file.header().dataWindow();
        int width = dw.max.x - dw.min.x + 1;
        int height = dw.max.y - dw.min.y + 1;

        if (width <= 0 || height <= 0) {
             throw std::runtime_error("Invalid image dimensions in EXR file");
        }

        texture::Image<math::Vector<T, 3>> image(width, height);

        // We only support reading R, G, B channels for now.
        // We'll read everything into a float buffer first to handle potential type mismatches 
        // (e.g. T=double, or file is HALF) in a uniform way, then convert.
        // This avoids complex template logic for PixelType and ensures compatibility.
        
        // Note: OpenEXR's FrameBuffer::insert expects pointer to where the data window *starts*.
        // Since we are reading into a fresh buffer 0..w, 0..h, we need to adjust the pointer
        // so that (dw.min.x, dw.min.y) maps to index 0.
        
        std::vector<float> buffer(
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3
        );
        
        const std::size_t pixel_stride = sizeof(float) * 3;
        const std::size_t row_stride = pixel_stride * static_cast<std::size_t>(width);
        
        // Base pointer calculation for OpenEXR:
        // address(x, y) = base + x * xStride + y * yStride
        // We want address(dw.min.x, dw.min.y) = buffer.data()
        // base + dw.min.x * stride + dw.min.y * row_stride = buffer.data()
        // => base = buffer.data() - (dw.min.x * stride + dw.min.y * row_stride)
        
        char* base = reinterpret_cast<char*>(buffer.data());
        base -= (static_cast<size_t>(dw.min.x) * pixel_stride + static_cast<size_t>(dw.min.y) * row_stride);

        Imf::FrameBuffer frame_buffer;
        frame_buffer.insert("R", Imf::Slice(Imf::FLOAT, base, pixel_stride, row_stride));
        frame_buffer.insert("G", Imf::Slice(Imf::FLOAT, base + sizeof(float), pixel_stride, row_stride));
        frame_buffer.insert("B", Imf::Slice(Imf::FLOAT, base + sizeof(float) * 2, pixel_stride, row_stride));

        file.setFrameBuffer(frame_buffer);
        file.readPixels(dw.min.y, dw.max.y);

        // Convert float buffer to target Image<Vector<T, 3>>
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int flipped_y = height - 1 - y;  // Flip Y axis for EXR coordinate system
                std::size_t src_idx = (
                    static_cast<std::size_t>(flipped_y) * static_cast<std::size_t>(width) + 
                    static_cast<std::size_t>(x)
                ) * 3;
                
                T r = static_cast<T>(buffer[src_idx + 0]);
                T g = static_cast<T>(buffer[src_idx + 1]);
                T b = static_cast<T>(buffer[src_idx + 2]);
                
                image.get_pixel(x, y) = math::Vector<T, 3>(r, g, b);
            }
        }

        return image;

    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to read EXR file " + input_path.string() + ": " + e.what());
    }
}

/**
 * @brief Write an 8-bit RGB PNG image.
 *
 * @tparam T          Scalar type of the image (double or float).
 * @param output_path Target file path for the PNG image.
 * @param image       Source image to write.
 */
template<typename T>
void write_ldr_image(
    const std::filesystem::path& output_path,
    const texture::Image<math::Vector<T, 3>>& image
) {
    int width = image.width();
    int height = image.height();
    
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    std::vector<unsigned char> data(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3
    );

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int flipped_y = height - 1 - y;  // Flip Y axis for EXR coordinate system
            auto pixel = image.get_pixel(x, flipped_y);
            // Simple linear quantization to 8-bit. 
            // Users should gamma-correct before calling if needed.
            auto clamp = [](T val) {
                return std::max(T(0), std::min(T(1), val));
            };
            
            data[(y * width + x) * 3 + 0] = static_cast<unsigned char>(clamp(pixel.x()) * 255.0 + 0.5);
            data[(y * width + x) * 3 + 1] = static_cast<unsigned char>(clamp(pixel.y()) * 255.0 + 0.5);
            data[(y * width + x) * 3 + 2] = static_cast<unsigned char>(clamp(pixel.z()) * 255.0 + 0.5);
        }
    }

    if (stbi_write_png(output_path.string().c_str(), width, height, 3, data.data(), width * 3) == 0) {
        throw std::runtime_error("Failed to write PNG to " + output_path.string());
    }
}

/**
 * @brief Read an LDR image (PNG, JPG, BMP, TGA) using stb_image.
 *
 * @tparam T          Scalar type of the output image (converts 8-bit to [0,1] float/double).
 * @param input_path  Source file path.
 * @return            Image read from file.
 */
template<typename T>
texture::Image<math::Vector<T, 3>> read_ldr_image(const std::filesystem::path& input_path) {
    if (!std::filesystem::exists(input_path)) {
        throw std::runtime_error("File not found: " + input_path.string());
    }

    int width, height, channels;
    // Force 3 channels (RGB)
    unsigned char* data = stbi_load(input_path.string().c_str(), &width, &height, &channels, 3);
    
    if (!data) {
        throw std::runtime_error("Failed to load image " + input_path.string() + ": " + stbi_failure_reason());
    }

    texture::Image<math::Vector<T, 3>> image(width, height);
    T scale = T(1) / T(255);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int flipped_y = height - 1 - y;  // Flip Y axis for EXR coordinate system
            std::size_t idx = (static_cast<std::size_t>(flipped_y) * width + x) * 3;
            image.get_pixel(x, y) = math::Vector<T, 3>(
                static_cast<T>(data[idx + 0]) * scale,
                static_cast<T>(data[idx + 1]) * scale,
                static_cast<T>(data[idx + 2]) * scale
            );
        }
    }

    stbi_image_free(data);
    return image;
}

/**
 * @brief Generic image write function that dispatches based on file extension.
 * Supports .exr via OpenEXR and .png via stb_image_write.
 * 
 * @tparam T          Scalar type of the image.
 * @param output_path Target file path (extension determines format).
 * @param image       Source image to write.
 */
template<typename T>
void write_image(const std::filesystem::path& output_path, const texture::Image<math::Vector<T, 3>>& image) {
    std::string ext = output_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });

    if (ext == ".exr") {
        write_hdr_image(output_path, image);
    } else if (ext == ".png") {
        write_ldr_image(output_path, image);
    } else {
        throw std::runtime_error("Unsupported image format for writing: " + ext);
    }
}

/**
 * @brief Generic image read function that dispatches based on file extension.
 * Supports .exr via OpenEXR and .png/.jpg/.bmp/.tga via stb_image.
 */
template<typename T>
texture::Image<math::Vector<T, 3>> read_image(const std::filesystem::path& input_path) {
    std::string ext = input_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });

    if (ext == ".exr") {
        return read_hdr_image<T>(input_path);
    } else {
        return read_ldr_image<T>(input_path);
    }
}

template<typename T>
void convert_image(
    const std::filesystem::path& input_path,
    const std::filesystem::path& output_path
) {
    auto hdr_image = read_image<T>(input_path);
    write_image<T>(output_path, hdr_image);
}

}
