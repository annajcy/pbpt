/**
 * @file
 * @brief Octahedral encoding and decoding of unit 3D directions.
 */
#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "vector.hpp" 

namespace pbpt::math {

/**
 * @brief Compact encoding of a unit 3D direction using octahedral projection.
 *
 * The vector is mapped from the unit sphere to a 2D octahedron,
 * then quantized into two 16-bit integers. This is useful for
 * storing normals or directions in G-buffers or other compressed
 * representations.
 *
 * @tparam T Floating-point scalar type used during encode/decode.
 */
template <typename T>
class OctahedralVector {
private:
    /// Quantized x component in octahedral space.
    std::uint16_t m_x_encoded{};
    /// Quantized y component in octahedral space.
    std::uint16_t m_y_encoded{};

public:
    /**
     * @brief Encodes a unit-length direction vector.
     *
     * The input vector is assumed to be normalized; it is projected
     * using octahedral mapping and each component is stored in 16 bits.
     */
    explicit OctahedralVector(const Vector<T, 3>& v) {
        Vector<T, 2> p = encode_direction(v);
        m_x_encoded = quantize(p[0]);
        m_y_encoded = quantize(p[1]);
    }

    /**
     * @brief Decodes the stored representation back to a unit vector.
     */
    Vector<T, 3> decode() const {
        T x = dequantize(m_x_encoded);
        T y = dequantize(m_y_encoded);
        return decode_direction(Vector<T, 2>{x, y});
    }

private:
    static constexpr uint16_t quantize(T v) {
        return static_cast<uint16_t>(std::clamp((v * T(0.5) + T(0.5)) * T(65535.0) + T(0.5), T(0), T(65535)));
    }

    static constexpr T dequantize(uint16_t u) {
        return T(u) / T(65535.0) * T(2.0) - T(1.0);
    }

    static Vector<T, 2> encode_direction(const Vector<T, 3>& v) {
        T x = v[0], y = v[1], z = v[2];
        T invL1 = T(1.0) / (std::abs(x) + std::abs(y) + std::abs(z));
        x *= invL1;
        y *= invL1;
        z *= invL1;

        if (is_less(z, 0)) {
            T ox = (T(1) - std::abs(y)) * (x >= 0 ? 1 : -1);
            T oy = (T(1) - std::abs(x)) * (y >= 0 ? 1 : -1);
            return Vector<T, 2>{ox, oy};
        }
        return Vector<T, 2>{x, y};
    }

    static Vector<T, 3> decode_direction(const Vector<T, 2>& e) {
        T x = e[0];
        T y = e[1];
        T z = T(1) - std::abs(x) - std::abs(y);

        if (is_less(z, 0)) {
            T ox = (T(1) - std::abs(y)) * (x >= 0 ? T(1) : T(-1));
            T oy = (T(1) - std::abs(x)) * (y >= 0 ? T(1) : T(-1));
            x = ox;
            y = oy;
        }

        Vector<T, 3> v{x, y, z};
        return v.normalized();
    }
};

} // namespace pbpt::math
