#pragma once

#include <algorithm>
#include <vector>
#include "math/vector.hpp"

namespace pbpt::texture {

template<typename T>
class Image {
private:
    int m_width;
    int m_height;
    std::vector<T> m_data;

public:
    Image() {}
    Image(int width, int height) : m_width(width), m_height(height) {
        m_data.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    }

    int width() const { return m_width; }
    int height() const { return m_height; }

    const T& get_pixel(int x, int y) const {
        return m_data[static_cast<std::size_t>(y) * static_cast<std::size_t>(m_width) + static_cast<std::size_t>(x)];
    }

    T& get_pixel(int x, int y) {
        return m_data[static_cast<std::size_t>(y) * static_cast<std::size_t>(m_width) + static_cast<std::size_t>(x)];
    }

    std::vector<T>& data() { return m_data; }
    const std::vector<T>& data() const { return m_data; }
};

template<typename T, int N>
struct ImageNImpl {
    using type = Image<math::Vector<T, N>>;
};

template<typename T>
struct ImageNImpl<T, 1> {
    using type = Image<T>;
};

template<typename T, int N>
using ImageN = typename ImageNImpl<T, N>::type;

template<typename T>
inline ImageN<T, 3> image4_to_image3(const ImageN<T, 4>& img4) {
    ImageN<T, 3> img3(img4.width(), img4.height());
    std::transform(
        img4.data().begin(), img4.data().end(),
        img3.data().begin(),
        [](const math::Vector<T, 4>& v) {
            return math::Vector<T, 3>(v.r(), v.g(), v.b());
        }
    );
    return img3;
}

template<typename T>
inline ImageN<T, 4> image3_to_image4(const ImageN<T, 3>& img3, T alpha = static_cast<T>(1)) {
    ImageN<T, 4> img4(img3.width(), img3.height());
    std::transform(
        img3.data().begin(), img3.data().end(),
        img4.data().begin(),
        [alpha](const math::Vector<T, 3>& v) {
            return math::Vector<T, 4>(v.r(), v.g(), v.b(), alpha);
        }
    );
    return img4;
}

template<typename T>
inline ImageN<T, 3> image1_to_image3(const ImageN<T, 1>& img1) {
    ImageN<T, 3> img3(img1.width(), img1.height());
    std::transform(
        img1.data().begin(), img1.data().end(),
        img3.data().begin(),
        [](const T& v) {
            return math::Vector<T, 3>(v, v, v);
        }
    );
    return img3;
}

template<typename T>
inline ImageN<T, 1> image3_to_image1(const ImageN<T, 3>& img3) {
    ImageN<T, 1> img1(img3.width(), img3.height());
    std::transform(
        img3.data().begin(), img3.data().end(),
        img1.data().begin(),
        [](const math::Vector<T, 3>& v) {
            return (v.r() + v.g() + v.b()) / static_cast<T>(3);
        }
    );
    return img1;
}

template<typename T>
inline ImageN<T, 1> image3_to_image1_perceived(const ImageN<T, 3>& img3) {
    ImageN<T, 1> img1(img3.width(), img3.height());
    std::transform(
        img3.data().begin(), img3.data().end(),
        img1.data().begin(),
        [](const math::Vector<T, 3>& v) {
            return static_cast<T>(0.299) * v.r() + 
                    static_cast<T>(0.587) * v.g() + 
                    static_cast<T>(0.114) * v.b();
        }
    );
    return img1;
}

};