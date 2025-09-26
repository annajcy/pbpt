#pragma once

#include <array>
#include <concepts>
#include <algorithm>
#include <type_traits>
#include <utility>

#include "homogeneous.hpp"
#include "operator.hpp"
#include "utils.hpp"
#include "vector.hpp"

namespace pbpt::math {
template <typename T, int R, int C>
    requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix;

template <typename T, int N>
class VectorView {
protected:
    const T* m_start_ptr;
    int      m_stride;

public:
    /// @brief Creates a non-owning view of data elements with specified stride.
    /// @warning The caller must ensure that the referenced data remains valid for the lifetime of this view.
    /// @param start Pointer to the first element (must not be nullptr)
    /// @param stride Distance between consecutive elements in the view
    [[nodiscard]] constexpr VectorView(const T* start, int stride) 
        : m_start_ptr(start), m_stride(stride) {
        assert_if_ex<std::invalid_argument>([&]() { return start == nullptr; }, 
                                           "VectorView requires non-null data pointer");
    }

    constexpr const T& operator[](int i) const { return m_start_ptr[i * m_stride]; }
    constexpr T&       operator[](int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    constexpr const T& at(int i) const { return m_start_ptr[i * m_stride]; }
    constexpr T&       at(int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    constexpr int dims() const noexcept { return N; }

    constexpr T dot(const Vector<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i)
            result += (*this)[i] * rhs[i];
        return result;
    }

    constexpr T dot(const VectorView<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i)
            result += (*this)[i] * rhs[i];
        return result;
    }

    constexpr Vector<T, N> to_vector() const {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = (*this)[i];
        return result;
    }

    constexpr VectorView& operator=(const Vector<T, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    constexpr VectorView& operator=(const VectorView<T, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    template <std::invocable<const T&, int> F>
    void visit(F func) const {
        for (int i = 0; i < N; ++i)
            func((*this)[i], i);
    }

    template <std::invocable<T&, int> F>
    void visit(F func) {
        for (int i = 0; i < N; ++i)
            func((*this)[i], i);
    }
};

template <typename T, int R, int C, int ViewR, int ViewC>
class MatrixView {
protected:
    const Matrix<T, R, C>& m_original;
    int                    m_row_start{};
    int                    m_col_start{};

public:
    using RowView = VectorView<T, ViewC>;
    using ColView = VectorView<T, ViewR>;

    constexpr MatrixView(const Matrix<T, R, C>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    constexpr const T& at(int r, int c) const { return m_original.at(m_row_start + r, m_col_start + c); }

    constexpr const RowView operator[](int r) const { return RowView(&(*this).at(r, 0), 1); }

    constexpr RowView operator[](int r) { return RowView(&(*this).at(r, 0), 1); }

    constexpr const RowView row(int r) const { return RowView(&(*this).at(r, 0), 1); }

    constexpr ColView col(int c) { return ColView(&(*this).at(0, c), ViewC); }

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

    template<std::invocable<T&, int, int> F>
    void visit(F func) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                func((*this).at(i, j), i, j);
            }
        }
    }

    template<std::invocable<const T&, int, int> F>
    void visit(F func) const {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                func((*this).at(i, j), i, j);
            }
        }
    }
};

template <typename T, int R, int C>
    requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix {
private:
    std::array<T, R * C> m_data{};

public:
    template <int ViewR, int ViewC>
    using MatView = MatrixView<T, R, C, ViewR, ViewC>;
    using RowView = VectorView<T, C>;
    using ColView = VectorView<T, R>;

    // Static factory methods (unified with Vector design)
    static constexpr Matrix zeros() noexcept { return Matrix{}; }
    
    static constexpr Matrix filled(T value) noexcept {
        Matrix result{};
        for (int i = 0; i < R * C; ++i)
            result.m_data[i] = value;
        return result;
    }
    
    static constexpr Matrix ones() noexcept { return filled(T(1)); }
    
    static constexpr Matrix identity() noexcept
        requires(R == C)
    {
        Matrix result{};
        for (int i = 0; i < R; ++i) {
            result[i][i] = T(1);
        }
        return result;
    }

    static constexpr Matrix from_array(const std::array<T, R * C>& arr) noexcept {
        Matrix result{};
        result.m_data = arr;
        return result;
    }

    static auto random() {
        Matrix result{};
        for (int i = 0; i < R * C; ++i) {
            result.m_data[i] = rand<T>();
        }
        return result;
    }

    // Constructors
    constexpr Matrix() noexcept = default;

    template <typename... Vecs>
        requires(sizeof...(Vecs) == C && (std::is_convertible_v<Vecs, Vector<T, R>> && ...))
    constexpr explicit Matrix(Vecs&&... col_vecs) noexcept {
        int col_index = 0;
        (([&] {
             for (int row_index = 0; row_index < R; ++row_index) {
                 (*this)[row_index][col_index] = col_vecs[row_index];
             }
             ++col_index;
         })(),
         ...);
    }

    template <std::convertible_to<T>... Vals>
        requires(sizeof...(Vals) == R * C && (std::is_arithmetic_v<std::remove_cvref_t<Vals>> && ...))
    constexpr Matrix(Vals&&... vals) noexcept : m_data{static_cast<T>(std::forward<Vals>(vals))...} {}

    // Copy/conversion constructor from different matrix types
    template <typename U>
        requires std::convertible_to<U, T>
    constexpr Matrix(const Matrix<U, R, C>& other) noexcept {
        for (int i = 0; i < R * C; ++i) {
            m_data[i] = static_cast<T>(other.data()[i]);
        }
    }

    // Accessors with improved error handling
    constexpr const T& at(int x, int y) const {
        assert_if_ex<std::out_of_range>([&]() { return x < 0 || x >= R || y < 0 || y >= C; }, 
                                       "Matrix index out of range");
        return m_data[x * C + y];
    }

    constexpr T& at(int x, int y) {
        assert_if_ex<std::out_of_range>([&]() { return x < 0 || x >= R || y < 0 || y >= C; }, 
                                       "Matrix index out of range");
        return m_data[x * C + y];
    }

    constexpr RowView operator[](int r) { return RowView(&(*this).at(r, 0), 1); }

    constexpr const RowView operator[](int r) const { return RowView(&(*this).at(r, 0), 1); }

    constexpr RowView row(int r) { return RowView(&(*this).at(r, 0), 1); }

    constexpr const RowView row(int r) const { return RowView(&(*this).at(r, 0), 1); }

    constexpr ColView col(int c) { return ColView(&(*this).at(0, c), C); }

    constexpr const ColView col(int c) const { return ColView(&(*this).at(0, c), C); }

    constexpr int row_dims() const noexcept { return R; }
    constexpr int col_dims() const noexcept { return C; }
    constexpr int dims() const noexcept { return R * C; }  // Total elements (unified with Vector)

    const T* data() const noexcept { return m_data.data(); }
    T*       data() noexcept { return m_data.data(); }
    
    constexpr std::array<T, R * C> to_array() const noexcept { return m_data; }

    // Unified comparison operators
    template <typename U>
    constexpr bool operator==(const Matrix<U, R, C>& rhs) const noexcept {
        // Use std::ranges::equal with custom comparator for floating-point comparison
        return std::ranges::equal(m_data, rhs.to_array(), 
                                 [](const T& a, const U& b) { 
                                     return is_equal(a, static_cast<T>(b)); 
                                 });
    }

    template <typename U>
    constexpr bool operator!=(const Matrix<U, R, C>& rhs) const noexcept { 
        return !(*this == rhs); 
    }

    /// @brief Check if any element in the matrix is NaN
    /// @return true if any element is NaN, false otherwise
    [[nodiscard]] constexpr bool has_nan() const noexcept 
        requires std::is_floating_point_v<T>
    {
        return std::ranges::any_of(m_data, [](T val) { return std::isnan(val); });
    }

    // Assignment operators with type safety
    template <typename U>
    constexpr Matrix& operator+=(const Matrix<U, R, C>& rhs) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] += static_cast<T>(rhs.data()[i]);
        return *this;
    }

    template <typename U>
    constexpr Matrix& operator-=(const Matrix<U, R, C>& rhs) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] -= static_cast<T>(rhs.data()[i]);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Matrix& operator*=(const U& scalar) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] *= static_cast<T>(scalar);
        return *this;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Matrix& operator/=(const U& scalar) noexcept {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in matrix division");
        for (int i = 0; i < R * C; ++i)
            m_data[i] /= static_cast<T>(scalar);
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

    constexpr Matrix& transpose() noexcept
        requires(R == C)
    {
        // True in-place transpose using symmetric swapping
        for (int i = 0; i < R; ++i) {
            for (int j = i + 1; j < C; ++j) {
                std::swap((*this)[i][j], (*this)[j][i]);
            }
        }
        return *this;
    }

    constexpr T determinant() const
        requires(R == C)
    {
        if constexpr (R == 1) {
            return (*this).at(0, 0);
        } else if constexpr (R == 2) {
            return (*this).at(0, 0) * (*this).at(1, 1) - (*this).at(0, 1) * (*this).at(1, 0);
        } else if constexpr (R == 3) {
            return (*this).at(0, 0) * ((*this).at(1, 1) * (*this).at(2, 2) - (*this).at(1, 2) * (*this).at(2, 1)) -
                   (*this).at(0, 1) * ((*this).at(1, 0) * (*this).at(2, 2) - (*this).at(1, 2) * (*this).at(2, 0)) +
                   (*this).at(0, 2) * ((*this).at(1, 0) * (*this).at(2, 1) - (*this).at(1, 1) * (*this).at(2, 0));
        } else if constexpr (R == 4) {
            // Optimized 4x4 determinant using cofactor expansion along first row
            return (*this).at(0, 0) * minor_matrix(0, 0).determinant() -
                   (*this).at(0, 1) * minor_matrix(0, 1).determinant() +
                   (*this).at(0, 2) * minor_matrix(0, 2).determinant() -
                   (*this).at(0, 3) * minor_matrix(0, 3).determinant();
        } else {
            // General case: Laplace expansion (less efficient, but works for
            // any size)
            T det = 0;
            for (int c = 0; c < C; ++c) {
                T sign = (c % 2 == 0) ? 1 : -1;
                det += sign * (*this).at(0, c) * minor_matrix(0, c).determinant();
            }
            return det;
        }
    }

    constexpr Matrix inversed() const
        requires(R == C)
    {
        T det = determinant();
        assert_if_ex<std::domain_error>([&det]() { return is_equal(det, T(0)); }, 
                                       "Cannot invert a singular matrix");

        // Optimized hardcoded versions for small matrices
        if constexpr (R == 1) {
            Matrix result{};
            result.at(0, 0) = T(1) / det;
            return result;
        } else if constexpr (R == 2) {
            return inverse_2x2_optimized();
        } else if constexpr (R == 3) {
            return inverse_3x3_optimized();
        } else if constexpr (R == 4) {
            return inverse_4x4_optimized();
        } else {
            // General cofactor method for larger matrices
            Matrix cofactor_matrix{};
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    T sign                = ((r + c) % 2 == 0) ? T(1) : T(-1);
                    cofactor_matrix[r][c] = sign * minor_matrix(r, c).determinant();
                }
            }

            Matrix adjugate_matrix = cofactor_matrix.transposed();
            return adjugate_matrix * (T(1) / det);
        }
    }

    constexpr Matrix& inverse()
        requires(R == C)
    {
        T det = determinant();
        assert_if_ex<std::domain_error>([&det]() { return is_equal(det, T(0)); }, 
                                       "Cannot invert a singular matrix");

        Matrix cofactor_matrix{};
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                T sign                = ((r + c) % 2 == 0) ? T(1) : T(-1);
                cofactor_matrix[r][c] = sign * minor_matrix(r, c).determinant();
            }
        }

        Matrix adjugate_matrix = cofactor_matrix.transposed();
        *this                  = adjugate_matrix * (T(1) / det);
        return *this;
    }

    template <int ViewR, int ViewC>
    constexpr Matrix::MatView<ViewR, ViewC> view(int row_start, int col_start) {
        assert_if_ex<std::out_of_range>([&]() { return row_start + ViewR > R || col_start + ViewC > C; }, 
                                       "View dimensions exceed matrix bounds");
        return Matrix::MatView<ViewR, ViewC>(*this, row_start, col_start);
    }

    template <int ViewR, int ViewC>
    constexpr const Matrix::MatView<ViewR, ViewC> view(int row_start, int col_start) const {
        assert_if_ex<std::out_of_range>([&]() { return row_start + ViewR > R || col_start + ViewC > C; }, 
                                       "View dimensions exceed matrix bounds");
        return Matrix::MatView<ViewR, ViewC>(*this, row_start, col_start);
    }

    /// @brief Creates a minor matrix by removing specified row and column.
    /// @param row_to_remove The row index to remove (0-based)
    /// @param col_to_remove The column index to remove (0-based) 
    /// @return A (R-1)×(C-1) matrix with the specified row and column removed
    constexpr auto minor_matrix(int row_to_remove, int col_to_remove) const
        requires(R == C)
    {
        Matrix<T, R - 1, C - 1> minor{};
        int                     minor_r = 0;
        for (int r = 0; r < R; ++r) {
            if (r == row_to_remove)
                continue;
            int minor_c = 0;
            for (int c = 0; c < C; ++c) {
                if (c == col_to_remove)
                    continue;
                minor[minor_r][minor_c] = (*this)[r][c];
                minor_c++;
            }
            minor_r++;
        }
        return minor;
    }

private:
    /// @brief Optimized 2x2 matrix inversion using analytical formula
    /// @param det Pre-computed determinant value
    /// @return Inverted 2x2 matrix
    constexpr Matrix inverse_2x2_optimized() const
        requires(R == 2 && C == 2)
    {
        T det = (*this).determinant();
        T inv_det = T(1) / det;
        Matrix result{};
        
        // For 2x2 matrix [[a, b], [c, d]], inverse is (1/det) * [[d, -b], [-c, a]]
        result.at(0, 0) =  (*this).at(1, 1) * inv_det;  // d
        result.at(0, 1) = -(*this).at(0, 1) * inv_det;  // -b
        result.at(1, 0) = -(*this).at(1, 0) * inv_det;  // -c
        result.at(1, 1) =  (*this).at(0, 0) * inv_det;  // a
        
        return result;
    }

    /// @brief Optimized 3x3 matrix inversion using analytical formula
    /// @param det Pre-computed determinant value
    /// @return Inverted 3x3 matrix
    constexpr Matrix inverse_3x3_optimized() const
        requires(R == 3 && C == 3)
    {
        T det = (*this).determinant();
        T inv_det = T(1) / det;
        Matrix result{};
        
        // Compute cofactor matrix elements directly
        // Row 0
        result.at(0, 0) = ((*this).at(1, 1) * (*this).at(2, 2) - (*this).at(1, 2) * (*this).at(2, 1)) * inv_det;
        result.at(0, 1) = ((*this).at(0, 2) * (*this).at(2, 1) - (*this).at(0, 1) * (*this).at(2, 2)) * inv_det;
        result.at(0, 2) = ((*this).at(0, 1) * (*this).at(1, 2) - (*this).at(0, 2) * (*this).at(1, 1)) * inv_det;
        
        // Row 1
        result.at(1, 0) = ((*this).at(1, 2) * (*this).at(2, 0) - (*this).at(1, 0) * (*this).at(2, 2)) * inv_det;
        result.at(1, 1) = ((*this).at(0, 0) * (*this).at(2, 2) - (*this).at(0, 2) * (*this).at(2, 0)) * inv_det;
        result.at(1, 2) = ((*this).at(0, 2) * (*this).at(1, 0) - (*this).at(0, 0) * (*this).at(1, 2)) * inv_det;
        
        // Row 2
        result.at(2, 0) = ((*this).at(1, 0) * (*this).at(2, 1) - (*this).at(1, 1) * (*this).at(2, 0)) * inv_det;
        result.at(2, 1) = ((*this).at(0, 1) * (*this).at(2, 0) - (*this).at(0, 0) * (*this).at(2, 1)) * inv_det;
        result.at(2, 2) = ((*this).at(0, 0) * (*this).at(1, 1) - (*this).at(0, 1) * (*this).at(1, 0)) * inv_det;
        
        return result;
    }

    /// @brief Optimized 4x4 matrix inversion using direct formula
    /// @return Inverted 4x4 matrix
    constexpr Matrix inverse_4x4_optimized() const
        requires(R == 4 && C == 4)
    {
        // Use direct analytical formula for 4x4 matrix inversion
        // More efficient than cofactor expansion
        const T a00 = (*this).at(0, 0), a01 = (*this).at(0, 1), a02 = (*this).at(0, 2), a03 = (*this).at(0, 3);
        const T a10 = (*this).at(1, 0), a11 = (*this).at(1, 1), a12 = (*this).at(1, 2), a13 = (*this).at(1, 3);
        const T a20 = (*this).at(2, 0), a21 = (*this).at(2, 1), a22 = (*this).at(2, 2), a23 = (*this).at(2, 3);
        const T a30 = (*this).at(3, 0), a31 = (*this).at(3, 1), a32 = (*this).at(3, 2), a33 = (*this).at(3, 3);

        // Calculate 2x2 determinants for cofactor computation
        const T det2_01_01 = a00 * a11 - a01 * a10;
        const T det2_01_02 = a00 * a12 - a02 * a10;
        const T det2_01_03 = a00 * a13 - a03 * a10;
        const T det2_01_12 = a01 * a12 - a02 * a11;
        const T det2_01_13 = a01 * a13 - a03 * a11;
        const T det2_01_23 = a02 * a13 - a03 * a12;

        const T det2_23_01 = a20 * a31 - a21 * a30;
        const T det2_23_02 = a20 * a32 - a22 * a30;
        const T det2_23_03 = a20 * a33 - a23 * a30;
        const T det2_23_12 = a21 * a32 - a22 * a31;
        const T det2_23_13 = a21 * a33 - a23 * a31;
        const T det2_23_23 = a22 * a33 - a23 * a32;

        // Calculate 3x3 cofactors for adjugate matrix
        const T cof00 = +(a11 * det2_23_23 - a12 * det2_23_13 + a13 * det2_23_12);
        const T cof01 = -(a10 * det2_23_23 - a12 * det2_23_03 + a13 * det2_23_02);
        const T cof02 = +(a10 * det2_23_13 - a11 * det2_23_03 + a13 * det2_23_01);
        const T cof03 = -(a10 * det2_23_12 - a11 * det2_23_02 + a12 * det2_23_01);

        const T cof10 = -(a01 * det2_23_23 - a02 * det2_23_13 + a03 * det2_23_12);
        const T cof11 = +(a00 * det2_23_23 - a02 * det2_23_03 + a03 * det2_23_02);
        const T cof12 = -(a00 * det2_23_13 - a01 * det2_23_03 + a03 * det2_23_01);
        const T cof13 = +(a00 * det2_23_12 - a01 * det2_23_02 + a02 * det2_23_01);

        const T cof20 = +(a31 * det2_01_23 - a32 * det2_01_13 + a33 * det2_01_12);
        const T cof21 = -(a30 * det2_01_23 - a32 * det2_01_03 + a33 * det2_01_02);
        const T cof22 = +(a30 * det2_01_13 - a31 * det2_01_03 + a33 * det2_01_01);
        const T cof23 = -(a30 * det2_01_12 - a31 * det2_01_02 + a32 * det2_01_01);

        const T cof30 = -(a21 * det2_01_23 - a22 * det2_01_13 + a23 * det2_01_12);
        const T cof31 = +(a20 * det2_01_23 - a22 * det2_01_03 + a23 * det2_01_02);
        const T cof32 = -(a20 * det2_01_13 - a21 * det2_01_03 + a23 * det2_01_01);
        const T cof33 = +(a20 * det2_01_12 - a21 * det2_01_02 + a22 * det2_01_01);

        // Calculate determinant using first row
        const T det = a00 * cof00 + a01 * cof01 + a02 * cof02 + a03 * cof03;
        
        assert_if_ex<std::domain_error>([&det]() { return is_equal(det, T(0)); }, 
                                       "Cannot invert a singular 4x4 matrix");
        
        const T inv_det = T(1) / det;

        // Build result matrix (transposed cofactor matrix / determinant)
        Matrix result{};
        result.at(0, 0) = cof00 * inv_det; result.at(0, 1) = cof10 * inv_det; 
        result.at(0, 2) = cof20 * inv_det; result.at(0, 3) = cof30 * inv_det;
        result.at(1, 0) = cof01 * inv_det; result.at(1, 1) = cof11 * inv_det;
        result.at(1, 2) = cof21 * inv_det; result.at(1, 3) = cof31 * inv_det;
        result.at(2, 0) = cof02 * inv_det; result.at(2, 1) = cof12 * inv_det;
        result.at(2, 2) = cof22 * inv_det; result.at(2, 3) = cof32 * inv_det;
        result.at(3, 0) = cof03 * inv_det; result.at(3, 1) = cof13 * inv_det;
        result.at(3, 2) = cof23 * inv_det; result.at(3, 3) = cof33 * inv_det;

        return result;
    }

public:

    // Arithmetic operators with type promotion
    template <typename U>
    constexpr auto operator+(const Matrix<U, R, C>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Matrix<ResultType, R, C> result{};
        for (int i = 0; i < R * C; ++i) {
            result.data()[i] = static_cast<ResultType>(m_data[i]) + 
                              static_cast<ResultType>(rhs.data()[i]);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator-(const Matrix<U, R, C>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Matrix<ResultType, R, C> result{};
        for (int i = 0; i < R * C; ++i) {
            result.data()[i] = static_cast<ResultType>(m_data[i]) - 
                              static_cast<ResultType>(rhs.data()[i]);
        }
        return result;
    }

    constexpr Matrix operator-() const noexcept {
        Matrix result{};
        for (int i = 0; i < R * C; ++i)
            result.m_data[i] = -m_data[i];
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator*(const U& scalar) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Matrix<ResultType, R, C> result{};
        for (int i = 0; i < R * C; ++i) {
            result.data()[i] = static_cast<ResultType>(m_data[i]) * 
                              static_cast<ResultType>(scalar);
        }
        return result;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(const U& scalar, const Matrix<T, R, C>& mat) noexcept {
        return mat * scalar;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr auto operator/(const U& scalar) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in matrix division");
        Matrix<ResultType, R, C> result{};
        for (int i = 0; i < R * C; ++i) {
            result.data()[i] = static_cast<ResultType>(m_data[i]) / 
                              static_cast<ResultType>(scalar);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Vector<U, C>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, R> result{};
        for (int r = 0; r < R; ++r) {
            result[r] = row(r).dot(rhs);
        }
        return result;
    }

    template <typename U>
    constexpr auto operator*(const Homogeneous<U, C - 1>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, R - 1> result{};
        for (int r = 0; r < R; ++r) {
            result[r] = row(r).dot(rhs.to_vector_raw());
        }
        return result;
    }

    template <typename U, int M>
    constexpr auto operator*(const Matrix<U, C, M>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Matrix<ResultType, R, M> result{};
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < M; ++c) {
                result[r][c] = row(r).dot(rhs.col(c));
            }
        }
        return result;
    }

    constexpr bool is_all_zero() const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (!pbpt::math::is_zero((*this).at(r, c)))
                    return false;
            }
        }
        return true;
    }

    constexpr bool is_identity() const
        requires(R == C)
    {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (r == c && !is_equal((*this).at(r, c), T(1)))
                    return false;
                if (r != c && !pbpt::math::is_zero((*this).at(r, c)))
                    return false;
            }
        }
        return true;
    }

    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    if (std::isnan((*this).at(r, c)))
                        return true;
                }
            }
        }
        return false;
    }

    // Apply function to each element (unified with Vector design)
    template <std::invocable<T&, int, int> F>
    constexpr void visit(F&& f) {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                f((*this).at(r, c), r, c);
            }
        }
    }

    template <std::invocable<const T&, int, int> F>
    constexpr void visit(F&& f) const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                f((*this).at(r, c), r, c);
            }
        }
    }
};

using Mat2   = Matrix<Float, 2, 2>;
using Mat3   = Matrix<Float, 3, 3>;
using Mat4   = Matrix<Float, 4, 4>;
using Mat3x4 = Matrix<Float, 3, 4>;
using Mat4x3 = Matrix<Float, 4, 3>;

}  // namespace pbpt::math
