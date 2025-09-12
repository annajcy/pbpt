#pragma once

#include <array>
#include "core/spectrum.hpp"

#include "math/matrix.hpp"
#include "math/operator.hpp"
#include "math/point.hpp"
#include "math/vector.hpp"

namespace pbpt::core {

template<typename T>
class XYZ : public math::Vector<T, 3> {
public:
    XYZ() : math::Vector<T, 3>(0, 0, 0) {}
    XYZ(T x, T y, T z) : math::Vector<T, 3>(x, y, z) {}
    XYZ(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    static constexpr XYZ from_xyY(const math::Point<T, 2>& xy, T Y = T{1}) {
        if (xy.y() == 0) {
            return XYZ(0, 0, 0);
        }
        T X = (xy.x() * Y) / xy.y();
        T Z = ((1 - xy.x() - xy.y()) * Y) / xy.y();
        return XYZ(X, Y, Z);
    }

    template<typename IlluminantSpectrumType>
    static XYZ<T> from_illuminant(const IlluminantSpectrumType& I) {
        T Xn = inner_product(I, CIE_X<T>);
        T Yn = inner_product(I, CIE_Y<T>);
        T Zn = inner_product(I, CIE_Z<T>);
        T denom = Yn; // = inner_product(I, CIE_Y<T>);
        // 归一化使理想白(单位反射率)在该光源下 Y=1
        return XYZ<T>(Xn / denom, Yn / denom, Zn / denom);
    }

    template<typename ReflectanceSpectrumType, typename IlluminantSpectrumType>
    static XYZ<T> from_reflectance_under_illuminant(const ReflectanceSpectrumType& R,
                                                    const IlluminantSpectrumType& I) {
        // 分子：R(λ)*I(λ) 与 CIE CMF 的内积
        T Xn = inner_product(R * I, CIE_X<T>);
        T Yn = inner_product(R * I, CIE_Y<T>);
        T Zn = inner_product(R * I, CIE_Z<T>);
        // 分母：I(λ) 与 ȳ(λ) 的内积（归一化，使理想白 Y=1）
        T denom = inner_product(I, CIE_Y<T>);
        return XYZ<T>(Xn / denom, Yn / denom, Zn / denom);
    }

    template<int N>
    static constexpr XYZ from_sampled_spectrum(
        const SampledSpectrum<T, N>& spectrum, 
        const SampledWavelength<T, N>& wavelengths, 
        const SampledPdf<T, N>& pdf) 
    {
        SampledSpectrum<T, N> X = CIE_X<T>.sample(wavelengths);
        SampledSpectrum<T, N> Y = CIE_Y<T>.sample(wavelengths);
        SampledSpectrum<T, N> Z = CIE_Z<T>.sample(wavelengths);
        return XYZ(
            (X * spectrum * pdf.inv()).average(), 
            (Y * spectrum * pdf.inv()).average(), 
            (Z * spectrum * pdf.inv()).average()
        );
    }

    T x() const { return this->operator[](0); }
    T y() const { return this->operator[](1); }
    T z() const { return this->operator[](2); }

    math::Point<T, 2> to_xy() const {
        return math::Point<T, 2>(x() / (x() + y() + z()), y() / (x() + y() + z()));
    }

    XYZ<T>& normalize_to_y(T target_y = T{100}) {
        T current_y = y();
        if (current_y != 0) {
            T scale = target_y / current_y;
            *this *= scale;
        }
        return *this;
    }

    XYZ<T> normalized_to_y(T target_y = T{100}) const {
        return XYZ<T>(*this).normalize_to_y(target_y);
    }
};

template<typename T>
std::string to_string(const XYZ<T>& xyz) {
    return std::format("XYZ<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       xyz.x(), 
                       xyz.y(), 
                       xyz.z());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const XYZ<T>& xyz) {
    return os << to_string(xyz);
}

template<typename T>
class LAB : public math::Vector<T, 3> {
public:
    LAB() : math::Vector<T, 3>(0, 0, 0) {}
    LAB(T L, T a, T b) : math::Vector<T, 3>(L, a, b) {}
    LAB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    static LAB<T> from_xyz(const XYZ<T>& xyz, const XYZ<T>& white_point) {
        auto f = [](T t) {
            const T delta = T(6) / T(29);
            if (t > delta * delta * delta) {
                return std::cbrt(t);
            } else {
                return t / (3 * delta * delta) + T(4) / T(29);
            }
        };

        T fx = f(xyz.x() / white_point.x());
        T fy = f(xyz.y() / white_point.y());
        T fz = f(xyz.z() / white_point.z());

        T L = T(116) * fy - T(16);
        T a = T(500) * (fx - fy);
        T b = T(200) * (fy - fz);

        return LAB<T>(L, a, b);
    }

    T L() const { return this->operator[](0); }
    T a() const { return this->operator[](1); }
    T b() const { return this->operator[](2); }
};

template<typename T>
std::string to_string(const LAB<T>& lab) {
    return std::format("LAB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       lab.L(), 
                       lab.a(), 
                       lab.b());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const LAB<T>& lab) {
    return os << to_string(lab);
}

template<typename T>
class RGB : public math::Vector<T, 3> {
public:
    RGB() : math::Vector<T, 3>(0, 0, 0) {}
    RGB(T r, T g, T b) : math::Vector<T, 3>(r, g, b) {}
    RGB(const math::Vector<T, 3>& vec) : math::Vector<T, 3>(vec) {}

    T r() const { return this->operator[](0); }
    T g() const { return this->operator[](1); }
    T b() const { return this->operator[](2); }
};

template<typename T>
std::string to_string(const RGB<T>& rgb) {
    return std::format("RGB<{}>({}, {}, {})", 
                       typeid(T).name(), 
                       rgb.r(), 
                       rgb.g(), 
                       rgb.b());
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const RGB<T>& rgb) {
    return os << to_string(rgb);
}

template<typename T>
class RGBColorSpace {
private:
    math::Point<T, 2> m_r_xy;
    math::Point<T, 2> m_g_xy;
    math::Point<T, 2> m_b_xy;

    XYZ<T> m_white_point;

    math::Matrix<T, 3, 3> m_rgb_to_xyz;
    math::Matrix<T, 3, 3> m_xyz_to_rgb;

public:
    RGBColorSpace(
        const math::Point<T, 2>& r_xy, 
        const math::Point<T, 2>& g_xy, 
        const math::Point<T, 2>& b_xy, 
        const XYZ<T>& m_white_point
    ) : m_r_xy(r_xy), m_g_xy(g_xy), m_b_xy(b_xy), m_white_point(m_white_point) {
        XYZ<T> r_chroma = XYZ<T>::from_xyY(r_xy);
        XYZ<T> g_chroma = XYZ<T>::from_xyY(g_xy);
        XYZ<T> b_chroma = XYZ<T>::from_xyY(b_xy);

        math::Matrix<T, 3, 3> M{
            r_chroma, g_chroma, b_chroma
        };

        math::Vector<T, 3> s = M.inversed() * m_white_point;

        m_rgb_to_xyz = math::Matrix<T, 3, 3>{
            s.x() * r_chroma, s.y() * g_chroma, s.z() * b_chroma
        };

        m_xyz_to_rgb = m_rgb_to_xyz.inversed();
    }

    const math::Point<T, 2>& r_xy() const { return m_r_xy; }
    const math::Point<T, 2>& g_xy() const { return m_g_xy; }
    const math::Point<T, 2>& b_xy() const { return m_b_xy; }
    const XYZ<T>& white_point() const { return m_white_point; }

    const math::Matrix<T, 3, 3>& rgb_to_xyz_matrix() const { return m_rgb_to_xyz; }
    const math::Matrix<T, 3, 3>& xyz_to_rgb_matrix() const { return m_xyz_to_rgb; }

    XYZ<T> to_xyz(const RGB<T>& rgb) const {
        return m_rgb_to_xyz * rgb;
    }

    RGB<T> to_rgb(const XYZ<T>& xyz) const {
        return m_xyz_to_rgb * xyz;
    }

};

template<typename T>
struct RGBSigmoidPolynomialOptimizationResult {
    math::Vector<T, 3> error;
    std::array<T, 3> coeffs;
};

template<typename T, typename LuminantSpectrumType>
inline auto optimize_albedo_rgb_sigmoid_polynomial(
    const RGB<T>& target_rgb, 
    const RGBColorSpace<T>& color_space,
    const LuminantSpectrumType& reference_luminant_spectrum,
    int max_iterations = 15, 
    double error_threshold = 1e-4, 
    const std::array<T, 3>& initial_guess = {0.0, 0.0, 0.0}
) {
    
    std::cout << "Starting optimization for target RGB: " << target_rgb << std::endl;
    XYZ<T> target_xyz = color_space.to_xyz(target_rgb);
    std::cout << "Target XYZ: " << target_xyz << std::endl;
    LAB<T> target_lab = LAB<T>::from_xyz(target_xyz, color_space.white_point());
    std::cout << "Target LAB: " << target_lab << std::endl;

    auto eval_lab = [&](const std::array<T,3>& c) -> LAB<T> {
        RGBAlbedoSpectrumDistribution<T, RGBSigmoidPolynomialNormalized> albedo({c[0], c[1], c[2]});
        XYZ<T> xyz = XYZ<T>::from_reflectance_under_illuminant(albedo, reference_luminant_spectrum);
        return LAB<T>::from_xyz(xyz, color_space.white_point());
    };

    auto eval_error = [&](const LAB<T>& lab) -> math::Vector<T, 3> {
        T dL = target_lab.L() - lab.L();
        T da = target_lab.a() - lab.a();
        T db = target_lab.b() - lab.b();
        return math::Vector<T, 3>{dL, da, db};
    };

    auto eval_jacobian = [&](const std::array<T,3>& c) -> math::Matrix<T, 3, 3> {
        const T epsilon = 1e-4;
        math::Matrix<T, 3, 3> J;
        for (int i = 0; i < 3; i++) {
            std::array<T, 3> c_plus = c;
            std::array<T, 3> c_minus = c;
            c_plus[i] += epsilon;
            c_minus[i] -= epsilon;
            LAB<T> lab_plus = eval_lab(c_plus);
            auto err_plus = eval_error(lab_plus);
            LAB<T> lab_minus = eval_lab(c_minus);
            auto err_minus = eval_error(lab_minus);
            J[0][i] = (err_plus[0] - err_minus[0]) / (2 * epsilon);
            J[1][i] = (err_plus[1] - err_minus[1]) / (2 * epsilon);
            J[2][i] = (err_plus[2] - err_minus[2]) / (2 * epsilon);
        }
        return J;
    };


    std::array<T, 3> coeffs = initial_guess;
    auto error = math::Vector<T, 3>::filled(std::numeric_limits<double>::max());
    for (int i = 0; i < max_iterations; i ++) {
        auto object_lab = eval_lab(coeffs);
        //std::cout << "Object LAB: " << object_lab << std::endl;
        error = eval_error(object_lab);

        std::cout << "Iteration " << i << ": Error = " << error 
                  << ", Coeffs = (" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ")\n";

        if ([&]() -> bool {
            for (int j = 0; j < 3; j++) {
                if (!math::is_less_equal(error[j], error_threshold)) {
                    return false;
                }
            }
            return true;
        }()) {
            return RGBSigmoidPolynomialOptimizationResult<T>{error, coeffs};
        }

        auto J = eval_jacobian(coeffs);

        std::cout << "Jacobian:\n" << J << std::endl;   

        // Solve the linear system J * delta = error and update coeffs
        auto delta = J.inversed() * error;
        coeffs[0] -= delta[0];
        coeffs[1] -= delta[1];
        coeffs[2] -= delta[2];
    }

    return RGBSigmoidPolynomialOptimizationResult<T>{error, coeffs};
}



};
