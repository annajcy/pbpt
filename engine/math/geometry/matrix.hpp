#pragma once

#include "vector.hpp" 

#include <array>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <functional>


/**
 * @file matrix.hpp
 * @brief Defines a generic, RxC-dimensional, constexpr-friendly matrix class and its views.
 */

namespace pbpt::math {
//Forward declarations
template<typename T, int R, int C>
requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix;

/**
 * @class VectorView
 * @brief A non-owning, mutable view/proxy of vector-like data.
 * @details Extends ConstVectorView to allow modification of the underlying data.
 */
template<typename T, int N>
class VectorView {
protected:
    const T* m_start_ptr;
    int m_stride;

public:

    constexpr VectorView(const T* start, int stride) : m_start_ptr(start), m_stride(stride) {}

    constexpr const T& operator[](int i) const { return m_start_ptr[i * m_stride]; }
    constexpr T& operator[](int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }
    constexpr const T& at(int i) const { return m_start_ptr[i * m_stride]; }
    constexpr T& at(int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    constexpr int dims() const noexcept { return N; }

    constexpr T dot(const Vector<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    constexpr T dot(const VectorView<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    constexpr Vector<T, N> to_vector() const {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i) result[i] = (*this)[i];
        return result;
    }

    constexpr VectorView& operator=(const Vector<T, N>& other) {
        for (int i = 0; i < N; ++i) (*this)[i] = other[i];
        return *this;
    }
    
    constexpr VectorView& operator=(const VectorView<T, N>& other) {
        for (int i = 0; i < N; ++i) (*this)[i] = other[i];
        return *this;
    }

    void apply(const std::function<void(T&, int)>& func) {
        for (int i = 0; i < N; ++i) func((*this)[i], i);
    }
};

/**
 * @class MatrixView
 * @brief A non-owning, mutable view/proxy into a sub-region of another Matrix.
 */
template<typename T, int R, int C, int ViewR, int ViewC>
class MatrixView {
protected:

    const Matrix<T, R, C>& m_original;
    int m_row_start{};
    int m_col_start{};

public:
    using RowView = VectorView<T, ViewC>;
    using ColView = VectorView<T, ViewR>;

    constexpr MatrixView(const Matrix<T, R, C>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    constexpr const T& at(int r, int c) const {
        return m_original.at(m_row_start + r, m_col_start + c);
    }

    constexpr const RowView operator[](int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }

    constexpr RowView operator[](int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    constexpr const RowView row(int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    constexpr ColView col(int c) {
        return ColView(&(*this).at(0, c), ViewC);
    }
    
    constexpr Matrix<T, ViewR, ViewC> to_matrix() const {
        Matrix<T, ViewR, ViewC> result;
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                result[i][j] = (*this)[i][j];
            }
        }
        return result;
    }

    constexpr T& at(int r, int c) {
        return const_cast<T&>(this->m_original.at(this->m_row_start + r, this->m_col_start + c));
    }

    constexpr MatrixView& operator=(const Matrix<T, ViewR, ViewC>& other) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                (*this).at(i, j) = other.at(i, j);
            }
        }
        return *this;
    }

    void apply(const std::function<void(T&, int, int)>& func) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                func((*this).at(i, j), i, j);
            }
        }
    }
};

/**
 * @class Matrix
 * @brief A template class for RxC dimensional mathematical matrices.
 * @details This class provides a generic, fixed-size matrix implementation using
 * row-major memory layout. It's designed for high performance and flexibility,
 * with extensive use of `constexpr` for compile-time computations.
 * @tparam T The underlying floating-point type.
 * @tparam R The number of rows.
 * @tparam C The number of columns.
 */
template<typename T, int R, int C>
requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix {

private:
    std::array<T, R * C> m_data{};

public:

    template<int ViewR, int ViewC>
    using MatView = MatrixView<T, R, C, ViewR, ViewC>;
    using RowView = VectorView<T, C>;
    using ColView = VectorView<T, R>;

    static constexpr Matrix zeros() noexcept { return Matrix(); }
    static constexpr Matrix identity() noexcept requires(R == C) {
        Matrix result{};
        for (int i = 0; i < R; ++i) {
            result[i][i] = 1.0;
        }
        return result;
    }

    constexpr Matrix() noexcept = default;

    template<typename... Vecs>
    requires(sizeof...(Vecs) == C && (std::is_same_v<Vector<T, R>, std::remove_cvref_t<Vecs>> && ...))
    constexpr explicit Matrix(Vecs&&... col_vecs) noexcept {
        int col_index = 0;
        (([&] {
            for(int row_index = 0; row_index < R; ++row_index) {
                (*this)[row_index][col_index] = col_vecs[row_index];
            }
            ++col_index;
        })(), ...);
    }

    template<std::convertible_to<T>... Vals>
    requires(sizeof...(Vals) == R * C && (std::is_arithmetic_v<std::remove_cvref_t<Vals>> && ...))
    constexpr Matrix(Vals&&... vals) noexcept : m_data{static_cast<T>(std::forward<Vals>(vals))...} {}

    constexpr const T& at(int r, int c) const {
        if (r < 0 || r >= R || c < 0 || c >= C) {
            if (std::is_constant_evaluated()) {
                 throw "Compile-time error: Matrix index out of range";
            } else {
                 throw std::out_of_range("Matrix index out of range");
            }
        }
        return m_data[r * C + c];
    }

    constexpr RowView operator[](int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    constexpr const RowView operator[](int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    constexpr RowView row(int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    constexpr const RowView row(int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    constexpr ColView col(int c) {
        return ColView(&(*this).at(0, c), C);
    }

    constexpr const ColView col(int c) const {
        return ColView(&(*this).at(0, c), C);
    }

    constexpr int row_dims() const noexcept { return R; }
    constexpr int col_dims() const noexcept { return C; }

    const T* data() const noexcept { return m_data.data(); }
    T* data() noexcept { return m_data.data(); }

    constexpr bool operator==(const Matrix& rhs) const noexcept {
        for (int i = 0; i < R * C; ++i) {
            if (!m_data[i] == rhs.m_data[i]) return false;
        }
        return true;
    }

    constexpr bool operator!=(const Matrix& rhs) const noexcept {
        return !(*this == rhs);
    }

    constexpr Matrix& operator+=(const Matrix& rhs) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] += rhs.m_data[i];
        return *this;
    }

    constexpr Matrix& operator-=(const Matrix& rhs) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] -= rhs.m_data[i];
        return *this;
    }

    constexpr Matrix& operator*=(T scalar) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] *= scalar;
        return *this;
    }

    constexpr Matrix<T, C, R> transposed() const noexcept {
        Matrix<T, C, R> result{};
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result[j][i] = (*this)[i][j];
            }
        }
        return result;
    }

    constexpr Matrix<T, C, R>& transpose() noexcept requires(R == C) {
        Matrix<T, C, R> result{};
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result[j][i] = (*this)[i][j];
            }
        }
        *this = result;
        return *this;
    }

    constexpr T determinant() const requires(R == C) {
        if constexpr (R == 1) {
            return (*this).at(0, 0);
        } else if constexpr (R == 2) {
            return (*this).at(0, 0) * (*this).at(1, 1) - (*this).at(0, 1) * (*this).at(1, 0);
        } else if constexpr (R == 3) {
            return (*this).at(0, 0) * ((*this).at(1, 1) * (*this).at(2, 2) - (*this).at(1, 2) * (*this).at(2, 1)) -
                   (*this).at(0, 1) * ((*this).at(1, 0) * (*this).at(2, 2) - (*this).at(1, 2) * (*this).at(2, 0)) +
                   (*this).at(0, 2) * ((*this).at(1, 0) * (*this).at(2, 1) - (*this).at(1, 1) * (*this).at(2, 0));
        } else {
            // General case: Laplace expansion (less efficient, but works for any size)
            T det = 0;
            for (int c = 0; c < C; ++c) {
                T sign = (c % 2 == 0) ? 1 : -1;
                det += sign * (*this).at(0, c) * submatrix(0, c).determinant();
            }
            return det;
        }
    }

    constexpr Matrix inversed() const requires(R == C) {
        T det = determinant();
        if (det == 0) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Cannot invert a singular matrix";
            } else {
                throw std::runtime_error("Cannot invert a singular matrix");
            }
        }

        Matrix cofactor_matrix{};
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                T sign = ((r + c) % 2 == 0) ? 1 : -1;
                cofactor_matrix[r][c] = sign * submatrix(r, c).determinant();
            }
        }

        Matrix adjugate_matrix = cofactor_matrix.transposed();
        return adjugate_matrix * (static_cast<T>(1.0) / det);
    }

    constexpr Matrix& inverse() requires(R == C) {
        T det = determinant();
        if (det == 0) {
            if (std::is_constant_evaluated()) {
                throw "Compile-time error: Cannot invert a singular matrix";
            } else {
                throw std::runtime_error("Cannot invert a singular matrix");
            }
        }

        Matrix cofactor_matrix{};
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                T sign = ((r + c) % 2 == 0) ? 1 : -1;
                cofactor_matrix[r][c] = sign * submatrix(r, c).determinant();
            }
        }

        Matrix adjugate_matrix = cofactor_matrix.transposed();
        *this = adjugate_matrix * (static_cast<T>(1.0) / det);
        return *this;
    }
    
    template <int ViewR, int ViewC>
    constexpr Matrix::MatView<ViewR, ViewC> view(int row_start, int col_start) {
        // Basic bounds check to ensure the view doesn't immediately go out of bounds
        if (std::is_constant_evaluated()) {
            if (row_start + ViewR > R || col_start + ViewC > C) {
                throw "Compile-time error: View dimensions exceed original matrix bounds";
            }
        }
        return Matrix::MatView<ViewR, ViewC>(*this, row_start, col_start);
    }
    
    template <int ViewR, int ViewC>
    constexpr const Matrix::MatView<ViewR, ViewC> view(int row_start, int col_start) const {
        // Basic bounds check to ensure the view doesn't immediately go out of bounds
        if (std::is_constant_evaluated()) {
            if (row_start + ViewR > R || col_start + ViewC > C) {
                throw "Compile-time error: View dimensions exceed original matrix bounds";
            }
        }
        return Matrix::MatView<ViewR, ViewC>(*this, row_start, col_start);
    }
    
    constexpr auto submatrix(int row_to_remove, int col_to_remove) const requires(R == C) {
        Matrix<T, R - 1, C - 1> sub{};
        int sub_r = 0;
        for (int r = 0; r < R; ++r) {
            if (r == row_to_remove) continue;
            int sub_c = 0;
            for (int c = 0; c < C; ++c) {
                if (c == col_to_remove) continue;
                sub[sub_r][sub_c] = (*this)[r][c];
                sub_c++;
            }
            sub_r++;
        }
        return sub;
    }

    constexpr Matrix operator+(const Matrix& rhs) const noexcept {
        auto result = *this;
        return result += rhs;
    }

    constexpr Matrix operator-(const Matrix& rhs) const noexcept {
        auto result = *this;
        return result -= rhs;
    }

    template<typename U>
    requires std::convertible_to<U, T>
    constexpr Matrix operator*(U scalar) const noexcept {
        Matrix result = *this;
        return result *= static_cast<T>(scalar);
    }

    template<typename U>
    requires std::convertible_to<U, T>
    friend constexpr Matrix operator*(U scalar, const Matrix<T, R, C>& mat) noexcept {
        auto result = mat;
        return result *= static_cast<T>(scalar);
    }

    constexpr Vector<T, R> operator*(const Vector<T, R>& rhs) const noexcept {
        Vector<T, R> result{};
        for (int r = 0; r < R; ++r) {
            result[r] = row(r).dot(rhs);
        }
        return result;
    }

    template<int M>
    constexpr Matrix<T, R, M> operator*(const Matrix<T, C, M>& rhs) const noexcept {
        Matrix<T, R, M> result{};
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < M; ++c) {
                result[r][c] = row(r).dot(rhs.col(c));
            }
        }
        return result;
    }

    friend std::ostream& operator<<(std::ostream& os, const Matrix<T, R, C>& mat) {
        os << "Matrix " << R << "x" << C << "[\n";
        for (int i = 0; i < R; ++i) {
            os << "  ";
            for (int j = 0; j < C; ++j) {
                os << mat.at(i, j) << (j == C - 1 ? "" : ",\t");
            }
            os << "\n";
        }
        os << "]";
        return os;
    }

    constexpr bool is_zero() const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if ((*this)[r][c] != 0) return false;
            }
        }
        return true;
    }

    constexpr bool is_identity() const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (r == c && (*this)[r][c] != 1) return false;
                if (r != c && (*this)[r][c] != 0) return false;
            }
        }
        return true;
    }

    constexpr bool has_nan() const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (std::isnan((*this)[r][c])) return true;
            }
        }
        return false;
    }
};


using Mat2 = Matrix<Float, 2, 2>;
using Mat3 = Matrix<Float, 3, 3>;
using Mat4 = Matrix<Float, 4, 4>;
using Mat3x4 = Matrix<Float, 3, 4>;
using Mat4x3 = Matrix<Float, 4, 3>;

} // namespace math
