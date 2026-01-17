#pragma once

#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfOutputFile.h>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include "math/point.hpp"

namespace pbpt::utils {

/**
 * @brief Write a floating-point RGB image to an OpenEXR file.
 *
 * The film object is sampled via `film.get_pixel_rgb(Point<int,2>)`
 * for each pixel in a scanline order. The result is stored in a
 * contiguous float buffer and written as three FLOAT channels (R,G,B)
 * using the OpenEXR library.
 *
 * @tparam FilmType Type exposing `get_pixel_rgb(Point<int,2>) -> RGB-like`.
 * @param film        Source film or framebuffer to sample.
 * @param output_path Target file path for the EXR image.
 * @param width       Image width in pixels.
 * @param height      Image height in pixels.
 *
 * @throws std::runtime_error if writing the EXR file fails.
 */
template<typename FilmType>
void write_exr(
    const std::filesystem::path& output_path,
    const FilmType& film,
    int width,
    int height
) {
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    std::vector<float> buffer(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3
    );

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto rgb = film.get_pixel_rgb(math::Point<int, 2>(x, y));
            std::size_t idx = (
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                static_cast<std::size_t>(x)
            ) * 3;
            buffer[idx + 0] = static_cast<float>(rgb.r());
            buffer[idx + 1] = static_cast<float>(rgb.g());
            buffer[idx + 2] = static_cast<float>(rgb.b());
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

}
