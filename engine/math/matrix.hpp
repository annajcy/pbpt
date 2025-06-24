#pragma once

#include <array>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <utility> // For std::forward and std::move
#include <functional>

#include "vector.hpp" // Assumed to be available from the user's code.


/**
 * @file matrix.hpp
 * @brief Defines a generic, RxC-dimensional, constexpr-friendly matrix class and its views.
 */

namespace pbpt {
namespace math {

// Forward declarations
template<typename T, int R, int C>
class Matrix;

template<typename T, int N>
class VectorView;

template<typename T, int N>
class ConstVectorView;

template<typename T, int OrigR, int OrigC, int R, int C>
class MatrixView;

template<typename T, int OrigR, int OrigC, int R, int C>
class ConstMatrixView;


/**
 * @class ConstVectorView
 * @brief A non-owning, read-only view/proxy of vector-like data (e.g., a matrix row/column).
 * @tparam T The underlying floating-point type.
 * @tparam N The number of elements in the view.
 */
template<typename T, int N>
class ConstVectorView {
protected:
    /** @brief Pointer to the first element of the viewed data. */
    const T* m_start_ptr;
    /** @brief The distance (in number of elements) between consecutive elements in the view. */
    int m_stride;

public:
    /**
     * @brief Constructs a read-only vector view.
     * @param start Pointer to the first element.
     * @param stride The step size to move between elements. For a row view, this is 1. For a column view, this is the column count of the source matrix.
     */
    constexpr ConstVectorView(const T* start, int stride) : m_start_ptr(start), m_stride(stride) {}

    /**
     * @brief Provides read-only access to an element of the view.
     * @param i The index of the element.
     * @return A const reference to the element in the original data source.
     */
    constexpr T operator[](int i) const { return m_start_ptr[i * m_stride]; }
    
    /** @brief Returns the number of dimensions of the view. */
    constexpr int dims() const noexcept { return N; }

    /**
     * @brief Calculates the dot product with an owning vector.
     * @param rhs The other vector.
     * @return The scalar dot product.
     */
    constexpr T dot(const Vec<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    /**
     * @brief Calculates the dot product with another read-only view.
     * @param rhs The other vector view.
     * @return The scalar dot product.
     */

    constexpr T dot(const ConstVectorView<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    /**
     * @brief Calculates the dot product with a mutable view.
     * @param rhs The other vector view.
     * @return The scalar dot product.
     */

    constexpr T dot(const VectorView<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    /**
     * @brief Converts the view into a new, owning Vec instance.
     * @details This performs a copy of the elements from the viewed region.
     */
    constexpr operator Vec<T, N>() const {
        Vec<T, N> result;
        for (int i = 0; i < N; ++i) result[i] = (*this)[i];
        return result;
    }
};

/**
 * @class VectorView
 * @brief A non-owning, mutable view/proxy of vector-like data.
 * @details Extends ConstVectorView to allow modification of the underlying data.
 */
template<typename T, int N>
class VectorView : public ConstVectorView<T, N> {
public:
    /** @brief Inherits constructors from ConstVectorView. */
    using ConstVectorView<T, N>::ConstVectorView;

    /**
     * @brief Provides mutable access to an element of the view.
     * @param i The index of the element.
     * @return A mutable reference to the element in the original data source.
     */
    constexpr T& operator[](int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

     /**
     * @brief Assigns an owning vector's content to the region represented by this view.
     * @param other The vector to copy from.
     * @return A reference to this view.
     */
    constexpr VectorView& operator=(const Vec<T, N>& other) {
        for (int i = 0; i < N; ++i) (*this)[i] = other[i];
        return *this;
    }
    
    /**
     * @brief Assigns another vector view's content to the region represented by this view.
     * @param other The vector view to copy from.
     * @return A reference to this view.
     */
    constexpr VectorView& operator=(const ConstVectorView<T, N>& other) {
        for (int i = 0; i < N; ++i) (*this)[i] = other[i];
        return *this;
    }

    /**
     * @brief Applies a function to each element of the view.
     * @param func The function to apply.
     */
    void apply(const std::function<void(T&, int)>& func) {
        for (int i = 0; i < N; ++i) func((*this)[i], i);
    }
};


/**
 * @class ConstMatrixView
 * @brief A non-owning, read-only view/proxy into a sub-region of another Matrix.
 */
template<typename T, int OrigR, int OrigC, int R, int C>
class ConstMatrixView {
protected:
    /** @brief A const reference to the original matrix. */
    const Matrix<T, OrigR, OrigC>& m_original;
    /** @brief The starting row of the view in the original matrix's coordinate system. */
    int m_row_start{};
    /** @brief The starting column of the view in the original matrix's coordinate system. */
    int m_col_start{};

public:
    /**
     * @brief Constructs a view from a Matrix instance.
     * @param original The matrix to view.
     * @param row_start The starting row index of the view.
     * @param col_start The starting column index of the view.
     */
    constexpr ConstMatrixView(const Matrix<T, OrigR, OrigC>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    /**
     * @brief Provides read-only access to an element of the view.
     * @param r The row index relative to the view.
     * @param c The column index relative to the view.
     * @return A const reference to the element in the original matrix.
     */
    constexpr const T& operator()(int r, int c) const {
        return m_original(m_row_start + r, m_col_start + c);
    }
    
    /**
     * @brief Converts the view into a new, owning Matrix instance.
     * @details This performs a copy of the elements from the viewed region.
     */
    constexpr operator Matrix<T, R, C>() const {
        Matrix<T, R, C> result;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result(i, j) = (*this)(i, j);
            }
        }
        return result;
    }
};

/**
 * @class MatrixView
 * @brief A non-owning, mutable view/proxy into a sub-region of another Matrix.
 */
template<typename T, int OrigR, int OrigC, int R, int C>
class MatrixView : public ConstMatrixView<T, OrigR, OrigC, R, C> {
public:

    /** @brief Inherits constructors from ConstMatrixView. */
    using ConstMatrixView<T, OrigR, OrigC, R, C>::ConstMatrixView;

    /**
     * @brief Provides mutable access to an element of the view.
     * @param r The row index relative to the view.
     * @param c The column index relative to the view.
     * @return A mutable reference to the element in the original matrix.
     */
    constexpr T& operator()(int r, int c) {
        return const_cast<Matrix<T, OrigR, OrigC>&>(this->m_original)(this->m_row_start + r, this->m_col_start + c);
    }

    /**
     * @brief Assigns another matrix's content to the region represented by this view.
     * @param other The matrix to copy from.
     * @return A reference to this view.
     */
    constexpr MatrixView& operator=(const Matrix<T, R, C>& other) {
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                (*this)(i, j) = other(i, j);
            }
        }
        return *this;
    }

    /**
     * @brief Applies a function to each element of the matrix.
     * @param func The function to apply.
     */
    void apply(const std::function<void(T&, int, int)>& func) {
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                func((*this)(i, j), i, j);
            }
        }
    }
    
};


// --- 类型别名 (Type Aliases) ---
/*@breif

*/
// --- 类型别名 (Type Aliases) ---
/** @brief A 2x2 matrix of type `Float`. */
using Mat2 = Matrix<Float, 2, 2>;
/** @brief A 3x3 matrix of type `Float`. */
using Mat3 = Matrix<Float, 3, 3>;
/** @brief A 4x4 matrix of type `Float`. */
using Mat4 = Matrix<Float, 4, 4>;
/** @brief A 3x4 matrix of type `Float`. */
using Mat3x4 = Matrix<Float, 3, 4>;
/** @brief A 4x3 matrix of type `Float`. */
using Mat4x3 = Matrix<Float, 4, 3>;

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
class Matrix {
    static_assert(R > 0 && C > 0, "Matrix dimensions must be positive");
    static_assert(std::is_floating_point_v<T>, "Matrix type must be floating point");

private:
    // Row-major storage for cache efficiency and intuitive C++ access.
    std::array<T, R * C> m_data{};

public:

    /**
     * @brief Provides a view into a sub-region of the matrix.
     * @tparam R1 The number of rows in the view.
     * @tparam C1 The number of columns in the view.
     * @param r The starting row index of the view.
     * @param c The starting column index of the view.
     * @return A MatrixView instance.
     */
    template<int R1, int C1>
    using View = MatrixView<T, R, C, R1, C1>;

    /**
     * @brief Provides a const view into a sub-region of the matrix.
     * @tparam R1 The number of rows in the view.
     * @tparam C1 The number of columns in the view.
     * @param r The starting row index of the view.
     * @param c The starting column index of the view.
     * @return A ConstMatrixView instance.
     */
    template<int R1, int C1>
    using ConstView = ConstMatrixView<T, R, C, R1, C1>;





    // --- 静态工厂函数 (Static Factory Functions) ---

    /**
     * @brief Creates a matrix with all components set to zero.
     */
    static constexpr Matrix zeros() noexcept { return Matrix(); }

    /**
     * @brief Creates an identity matrix.
     * @note Only available for square matrices (R == C).
     */
    static constexpr Matrix identity() noexcept requires(R == C) {
        Matrix result{};
        for (int i = 0; i < R; ++i) {
            result(i, i) = 1.0;
        }
        return result;
    }

    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor. Initializes a zero matrix.
     */
    constexpr Matrix() noexcept = default;

    /**
     * @brief Constructs a matrix from a list of row vectors.
     * @details The number of vectors must match the number of rows (R).
     * Each vector's dimension must match the number of columns (C).
     * @param rows A parameter pack of Vec<T, C> objects.
     */
    template<typename... Vecs>
    constexpr explicit Matrix(Vecs&&... rows) noexcept requires(sizeof...(Vecs) == R) {
        std::array<Vec<T, C>, R> row_vectors = {std::forward<Vecs>(rows)...};
        for(int i = 0; i < R; ++i) {
            for(int j = 0; j < C; ++j) {
                m_data[i * C + j] = row_vectors[i][j];
            }
        }
    }


    // --- 访问器 (Accessors) ---

    /**
     * @brief Provides const access to matrix components by row and column.
     * @param r The zero-based row index.
     * @param c The zero-based column index.
     * @return A const reference to the component.
     * @throw std::out_of_range If index is out of bounds at runtime.
     */
    constexpr const T& operator()(int r, int c) const {
        if (r < 0 || r >= R || c < 0 || c >= C) {
            if (std::is_constant_evaluated()) {
                 throw "Compile-time error: Matrix index out of range";
            } else {
                 throw std::out_of_range("Matrix index out of range");
            }
        }
        return m_data[r * C + c];
    }

    /**
     * @brief Provides mutable access to matrix components by row and column.
     * @param r The zero-based row index.
     * @param c The zero-based column index.
     * @return A mutable reference to the component.
     * @throw std::out_of_range If index is out of bounds at runtime.
     */
    constexpr T& operator()(int r, int c) {
        if (r < 0 || r >= R || c < 0 || c >= C) {
            if (std::is_constant_evaluated()) {
                 throw "Compile-time error: Matrix index out of range";
            } else {
                 throw std::out_of_range("Matrix index out of range");
            }
        }
        return m_data[r * C + c];
    }
    
    /**
     * @brief Returns a non-owning, mutable view of a row.
     */
    constexpr VectorView<T, C> row(int r) {
        return VectorView<T, C>(&(*this)(r, 0), 1);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr ConstVectorView<T, C> row(int r) const {
        return ConstVectorView<T, C>(&(*this)(r, 0), 1);
    }
    
    /**
     * @brief Returns a non-owning, mutable view of a column.
     */
    constexpr VectorView<T, R> col(int c) {
        return VectorView<T, R>(&(*this)(0, c), C);
    }

    /**
     * @brief Returns a non-owning, read-only view of a column.
     */
    constexpr ConstVectorView<T, R> col(int c) const {
        return ConstVectorView<T, R>(&(*this)(0, c), C);
    }


    /** @brief Returns the number of rows. */
    constexpr int row_dims() const noexcept { return R; }

    /** @brief Returns the number of columns. */
    constexpr int col_dims() const noexcept { return C; }

    /**
     * @brief Returns a pointer to the underlying contiguous data array.
     * @details Useful for interoperability with graphics APIs like OpenGL.
     * The data is in row-major order.
     */
    const T* data() const noexcept { return m_data.data(); }

    /**
     * @brief Returns a pointer to the underlying contiguous data array.
     * @details Useful for interoperability with graphics APIs like OpenGL.
     * The data is in row-major order.
     */
    T* data() noexcept { return m_data.data(); }


    // --- 复合赋值运算符 (Compound Assignment Operators) ---
    
    /**
     * @brief Adds the elements of another matrix to this matrix.
     * @param rhs The matrix to add.
     * @return A reference to this matrix.
     */
    constexpr Matrix& operator+=(const Matrix& rhs) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] += rhs.m_data[i];
        return *this;
    }

    /**
     * @brief Subtracts the elements of another matrix from this matrix.
     * @param rhs The matrix to subtract.
     * @return A reference to this matrix.
     */
    constexpr Matrix& operator-=(const Matrix& rhs) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] -= rhs.m_data[i];
        return *this;
    }

    /**
     * @brief Multiplies this matrix by a scalar.
     * @param scalar The scalar to multiply by.
     * @return A reference to this matrix.
     */
    constexpr Matrix& operator*=(T scalar) noexcept {
        for (int i = 0; i < R * C; ++i) m_data[i] *= scalar;
        return *this;
    }

    // --- 矩阵数学运算 (Matrix Math Operations) ---

    /**
     * @brief Computes the transpose of this matrix.
     * @return A new Matrix<T, C, R> that is the transpose.
     */
    constexpr Matrix<T, C, R> transpose() const noexcept {
        Matrix<T, C, R> result{};
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    /**
     * @brief Computes the determinant of the matrix.
     * @note Only available for square matrices.
     */
    constexpr T determinant() const requires(R == C) {
        if constexpr (R == 1) {
            return (*this)(0, 0);
        } else if constexpr (R == 2) {
            return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
        } else if constexpr (R == 3) {
            return (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) -
                   (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 0)) +
                   (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0));
        } else {
            // General case: Laplace expansion (less efficient, but works for any size)
            T det = 0;
            for (int c = 0; c < C; ++c) {
                T sign = (c % 2 == 0) ? 1 : -1;
                det += sign * (*this)(0, c) * submatrix(0, c).determinant();
            }
            return det;
        }
    }

    /**
     * @brief Computes the inverse of the matrix.
     * @note Only available for square matrices.
     * @throws std::runtime_error if the matrix is singular (determinant is zero).
     */
    constexpr Matrix inverse() const requires(R == C) {
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
                cofactor_matrix(r, c) = sign * submatrix(r, c).determinant();
            }
        }

        Matrix adjugate_matrix = cofactor_matrix.transpose();
        return adjugate_matrix * (static_cast<T>(1.0) / det);
    }
    
    /**
     * @brief Creates a mutable, non-owning view into a sub-region of this matrix.
     * @tparam NewR The number of rows in the view.
     * @tparam NewC The number of columns in the view.
     * @param row_start The starting row index for the view.
     * @param col_start The starting column index for the view.
     * @return A MatrixView object that refers to a part of this matrix.
     */
    template <int NewR, int NewC>
    constexpr MatrixView<T, R, C, NewR, NewC> view(int row_start, int col_start) {
        // Basic bounds check to ensure the view doesn't immediately go out of bounds
        if (std::is_constant_evaluated()) {
            if (row_start + NewR > R || col_start + NewC > C) {
                throw "Compile-time error: View dimensions exceed original matrix bounds";
            }
        }
        return MatrixView<T, R, C, NewR, NewC>(*this, row_start, col_start);
    }
    
    /**
     * @brief Creates a non-owning, read-only view into a sub-region of this const matrix.
     * @tparam NewR The number of rows in the view.
     * @tparam NewC The number of columns in the view.
     * @param row_start The starting row index for the view.
     * @param col_start The starting column index for the view.
     * @return A ConstMatrixView object that refers to a part of this matrix.
     */
    template <int NewR, int NewC>
    constexpr ConstMatrixView<T, R, C, NewR, NewC> view(int row_start, int col_start) const {
        // Basic bounds check to ensure the view doesn't immediately go out of bounds
        if (std::is_constant_evaluated()) {
            if (row_start + NewR > R || col_start + NewC > C) {
                throw "Compile-time error: View dimensions exceed original matrix bounds";
            }
        }
        return ConstMatrixView<T, R, C, NewR, NewC>(*this, row_start, col_start);
    }
    
    /**
     * @brief Extracts a submatrix by specifying which rows and columns to keep. Returns a copy.
     * @tparam NewR The number of rows in the new submatrix.
     * @tparam NewC The number of columns in the new submatrix.
     * @param rows_to_keep An array of row indices to include in the submatrix.
     * @param cols_to_keep An array of column indices to include in the submatrix.
     * @return A new Matrix<T, NewR, NewC> containing the specified elements.
     */
    template <int NewR, int NewC>
    constexpr Matrix<T, NewR, NewC> submatrix(const std::array<int, NewR>& rows_to_keep, const std::array<int, NewC>& cols_to_keep) const {
        Matrix<T, NewR, NewC> result{};
        for (int i = 0; i < NewR; ++i) {
            for (int j = 0; j < NewC; ++j) {
                // The underlying operator() will perform bounds checking.
                result(i, j) = (*this)(rows_to_keep[i], cols_to_keep[j]);
            }
        }
        return result;
    }

    /**
     * @brief Extracts a contiguous submatrix of size NewR x NewC. Returns a copy.
     * @tparam NewR The number of rows in the new submatrix.
     * @tparam NewC The number of columns in the new submatrix.
     * @param row_start The starting row index of the submatrix block.
     * @param col_start The starting column index of the submatrix block.
     * @return A new Matrix<T, NewR, NewC> containing the specified block.
     */
    template <int NewR, int NewC>
    constexpr Matrix<T, NewR, NewC> submatrix(int row_start, int col_start) const {
        Matrix<T, NewR, NewC> result{};
        for (int i = 0; i < NewR; ++i) {
            for (int j = 0; j < NewC; ++j) {
                // The underlying operator() will perform bounds checking for (row_start + i) and (col_start + j).
                result(i, j) = (*this)(row_start + i, col_start + j);
            }
        }
        return result;
    }
    
private:
    /**
     * @brief Creates a submatrix by removing a specified row and column.
     * @note Helper for determinant and inverse calculations.
     */
    constexpr Matrix<T, R - 1, C - 1> submatrix(int row_to_remove, int col_to_remove) const {
        Matrix<T, R - 1, C - 1> sub{};
        int sub_r = 0;
        for (int r = 0; r < R; ++r) {
            if (r == row_to_remove) continue;
            int sub_c = 0;
            for (int c = 0; c < C; ++c) {
                if (c == col_to_remove) continue;
                sub(sub_r, sub_c) = (*this)(r, c);
                sub_c++;
            }
            sub_r++;
        }
        return sub;
    }
};

// --- 全局运算符 (Global Operators) ---

// Matrix-Matrix Addition
/**
 * @brief Matrix addition.
 * @details Element-wise addition of two matrices.
 * @return A new Matrix<T, R, C>
 */
template<typename T, int R, int C>
constexpr Matrix<T, R, C> operator+(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) noexcept {
    auto result = lhs;
    return result += rhs;
}


// Matrix-Matrix Subtraction
/**
 * @brief Matrix subtraction.
 * @details Element-wise subtraction of two matrices.
 * @return A new Matrix<T, R, C>
 */
template<typename T, int R, int C>
constexpr Matrix<T, R, C> operator-(const Matrix<T, R, C>& lhs, const Matrix<T, R, C>& rhs) noexcept {
    auto result = lhs;
    return result -= rhs;
}

// Scalar-Matrix Multiplication
/**
 * @brief Matrix multiplication by a scalar.
 * @details Multiplies each element of the matrix by the scalar.
 * @return A new Matrix<T, R, C>
 */
template<typename T, int R, int C, std::convertible_to<T> U>
constexpr Matrix<T, R, C> operator*(U scalar, const Matrix<T, R, C>& mat) noexcept {
    auto result = mat;
    return result *= static_cast<T>(scalar);
}

// Matrix-Scalar Multiplication
/**
 * @brief Matrix multiplication by a scalar.
 * @details Multiplies each element of the matrix by the scalar.
 * @return A new Matrix<T, R, C>
 */
template<typename T, int R, int C, std::convertible_to<T> U>
constexpr Matrix<T, R, C> operator*(const Matrix<T, R, C>& mat, U scalar) noexcept {
    return scalar * mat;
}

// --- 核心乘法运算 (Core Multiplication) ---

/**
 * @brief Matrix-Vector multiplication.
 * @details Transforms a column vector by a matrix.
 * Result is a new column vector.
 * @return A new Vec<T, R>
 */
template<typename T, int R, int C>
constexpr Vec<T, R> operator*(const Matrix<T, R, C>& lhs, const Vec<T, C>& rhs) noexcept {
    Vec<T, R> result{};
    for (int i = 0; i < R; ++i) {
        result[i] = lhs.row(i).dot(rhs);
    }
    return result;
}

/**
 * @brief Matrix-Matrix multiplication.
 * @details The number of columns in the left matrix must equal the number of rows
 * in the right matrix.
 * @return A new Matrix<T, R1, C2>
 */
template<typename T, int R1, int C1, int C2>
constexpr Matrix<T, R1, C2> operator*(const Matrix<T, R1, C1>& lhs, const Matrix<T, C1, C2>& rhs) noexcept {
    Matrix<T, R1, C2> result{};
    for (int r = 0; r < R1; ++r) {
        for (int c = 0; c < C2; ++c) {
            result(r, c) = lhs.row(r).dot(rhs.col(c));
        }
    }
    return result;
}

/**
 * @brief Matrix stream output.
 * 
 * @tparam T The matrix element type.
 * @tparam R The number of rows in the matrix.
 * @tparam C The number of columns in the matrix.
 * @param os The output stream.
 * @param mat The matrix to output.
 * @return std::ostream& The output stream.
 */
template<typename T, int R, int C>
std::ostream& operator<<(std::ostream& os, const Matrix<T, R, C>& mat) {
    os << "Matrix " << R << "x" << C << "[\n";
    for (int i = 0; i < R; ++i) {
        os << "  ";
        for (int j = 0; j < C; ++j) {
            os << mat(i, j) << (j == C - 1 ? "" : ",\t");
        }
        os << "\n";
    }
    os << "]";
    return os;
}

} // namespace math
} // namespace pbpt
