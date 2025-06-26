#pragma once

#include <array>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <functional>

#include "vector.hpp" 
#include "homogeneous.hpp" 

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
    constexpr VectorView(const T* start, int stride) : m_start_ptr(start), m_stride(stride) {}

    /**
     * @brief Provides read-only access to an element of the view.
     * @param i The index of the element.
     * @return A const reference to the element in the original data source.
     */
    constexpr const T& operator[](int i) const { return m_start_ptr[i * m_stride]; }

    /**
     * @brief Provides read-only access to an element of the view.
     * @param i The index of the element.
     * @return A const reference to the element in the original data source.
     */
    constexpr const T& at(int i) const { return m_start_ptr[i * m_stride]; }

    /** @brief Returns the number of dimensions of the view. */
    constexpr int dims() const noexcept { return N; }

    /**
     * @brief Calculates the dot product with an owning vector.
     * @param rhs The other vector.
     * @return The scalar dot product.
     */
    constexpr T dot(const Vector<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i) result += (*this)[i] * rhs[i];
        return result;
    }

    /**
     * @brief Calculates the dot product with another vector view.
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
    constexpr Vector<T, N> to_vector() const {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i) result[i] = (*this)[i];
        return result;
    }

    /**
     * @brief Provides mutable access to an element of the view.
     * @param i The index of the element.
     * @return A mutable reference to the element in the original data source.
     */
    constexpr T& operator[](int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    /**
     * @brief Provides mutable access to an element of the view.
     * @param i The index of the element.
     * @return A mutable reference to the element in the original data source.
     */
    constexpr T& at(int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

     /**
     * @brief Assigns an owning vector's content to the region represented by this view.
     * @param other The vector to copy from.
     * @return A reference to this view.
     */
    constexpr VectorView& operator=(const Vector<T, N>& other) {
        for (int i = 0; i < N; ++i) (*this)[i] = other[i];
        return *this;
    }
    
    /**
     * @brief Assigns another vector view's content to the region represented by this view.
     * @param other The vector view to copy from.
     * @return A reference to this view.
     */
    constexpr VectorView& operator=(const VectorView<T, N>& other) {
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
 * @class MatrixView
 * @brief A non-owning, mutable view/proxy into a sub-region of another Matrix.
 */
template<typename T, int R, int C, int ViewR, int ViewC>
class MatrixView {
protected:
    /** @brief A const reference to the original matrix. */
    const Matrix<T, R, C>& m_original;
    /** @brief The starting row of the view in the original matrix's coordinate system. */
    int m_row_start{};
    /** @brief The starting column of the view in the original matrix's coordinate system. */
    int m_col_start{};

public:
    using RowView = VectorView<T, ViewC>;
    using ColView = VectorView<T, ViewR>;
    /**
     * @brief Constructs a view from a Matrix instance.
     * @param original The matrix to view.
     * @param row_start The starting row index of the view.
     * @param col_start The starting column index of the view.
     */
    constexpr MatrixView(const Matrix<T, R, C>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    /**
     * @brief Provides read-only access to an element of the view.
     * @param r The row index relative to the view.
     * @param c The column index relative to the view.
     * @return A const reference to the element in the original matrix.
     */
    constexpr const T& at(int r, int c) const {
        return m_original.at(m_row_start + r, m_col_start + c);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr const RowView operator[](int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr RowView operator[](int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr const RowView row(int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    /**
     * @brief Returns a non-owning, mutable view of a column.
     */
    constexpr ColView col(int c) {
        return ColView(&(*this).at(0, c), ViewC);
    }
    
    /**
     * @brief Converts the view into a new, owning Matrix instance.
     * @details This performs a copy of the elements from the viewed region.
     */
    constexpr Matrix<T, ViewR, ViewC> to_matrix() const {
        Matrix<T, ViewR, ViewC> result;
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                result[i][j] = (*this)[i][j];
            }
        }
        return result;
    }

    /**
     * @brief Provides mutable access to an element of the view.
     * @param r The row index relative to the view.
     * @param c The column index relative to the view.
     * @return A mutable reference to the element in the original matrix.
     */
    constexpr T& at(int r, int c) {
        return const_cast<T&>(this->m_original.at(this->m_row_start + r, this->m_col_start + c));
    }

    /**
     * @brief Assigns another matrix's content to the region represented by this view.
     * @param other The matrix to copy from.
     * @return A reference to this view.
     */
    constexpr MatrixView& operator=(const Matrix<T, ViewR, ViewC>& other) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                (*this).at(i, j) = other.at(i, j);
            }
        }
        return *this;
    }

    /**
     * @brief Applies a function to each element of the matrix.
     * @param func The function to apply.
     */
    void apply(const std::function<void(T&, int, int)>& func) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                func((*this).at(i, j), i, j);
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
requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix {

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
    template<int ViewR, int ViewC>
    using MatView = MatrixView<T, R, C, ViewR, ViewC>;
    using RowView = VectorView<T, C>;
    using ColView = VectorView<T, R>;

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
            result[i][i] = 1.0;
        }
        return result;
    }

    // --- 构造函数 (Constructors) ---

    /**
     * @brief Default constructor. Initializes a zero matrix.
     */
    constexpr Matrix() noexcept = default;

    /**
    * @brief Constructs a matrix from a list of col vectors.
    * @details The number of vectors must match the number of columns (C).
    * Each vector's dimension must match the number of rows (R).
    * The requires clause ensures this constructor is only enabled for Vector types.
    * @param col_vecs A parameter pack of column vectors.
    */
    template<typename... Vecs>
    // 将类型检查 (std::is_same_v<...>) 加入 requires 子句
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

    /**
     * @brief Constructs a matrix from a list of values.
     * @details The number of values must match the number of elements (R * C).
     * @tparam Vals The parameter pack of values.
     */

    template<std::convertible_to<T>... Vals>
    // 使用折叠表达式 (&& ...) 来展开参数包
    requires(sizeof...(Vals) == R * C && (std::is_arithmetic_v<std::remove_cvref_t<Vals>> && ...))
    constexpr Matrix(Vals&&... vals) noexcept : m_data{static_cast<T>(std::forward<Vals>(vals))...} {}

    // --- 访问器 (Accessors) ---

    /**
     * @brief Provides const access to matrix components by row and column.
     * @param r The zero-based row index.
     * @param c The zero-based column index.
     * @return A const reference to the component.
     * @throw std::out_of_range If index is out of bounds at runtime.
     */
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

    /**
     * @brief Returns a non-owning, mutable view of a row.
     */
    constexpr RowView operator[](int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr const RowView operator[](int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    /**
     * @brief Returns a non-owning, mutable view of a row.
     */
    constexpr RowView row(int r) {
        return RowView(&(*this).at(r, 0), 1);
    }

    /**
     * @brief Returns a non-owning, read-only view of a row.
     */
    constexpr const RowView row(int r) const {
        return RowView(&(*this).at(r, 0), 1);
    }
    
    /**
     * @brief Returns a non-owning, mutable view of a column.
     */
    constexpr ColView col(int c) {
        return ColView(&(*this).at(0, c), C);
    }

    /**
     * @brief Returns a non-owning, read-only view of a column.
     */
    constexpr const ColView col(int c) const {
        return ColView(&(*this).at(0, c), C);
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

    // 比较运算符

    /**
     * @brief Compares two matrices for equality.
     * @param rhs The matrix to compare with.
     * @return true if the matrices are equal, false otherwise.
     */
    constexpr bool operator==(const Matrix& rhs) const noexcept {
        for (int i = 0; i < R * C; ++i) {
            if (!math::is_equal(m_data[i], rhs.m_data[i])) return false;
        }
        return true;
    }

    /**
     * @brief Compares two matrices for inequality.
     * @param rhs The matrix to compare with.
     * @return true if the matrices are not equal, false otherwise.
     */
    constexpr bool operator!=(const Matrix& rhs) const noexcept {
        return !(*this == rhs);
    }

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
                result[j][i] = (*this)[i][j];
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

    /**
     * @brief Computes the inverse of the matrix.
     * @note Only available for square matrices.
     * @throws std::runtime_error if the matrix is singular (determinant is zero).
     */
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
    
    /**
     * @brief Creates a non-owning, read-only view into a sub-region of this const matrix.
     * @tparam ViewR The number of rows in the view.
     * @tparam ViewC The number of columns in the view.
     * @param row_start The starting row index for the view.
     * @param col_start The starting column index for the view.
     * @return A ConstMatrixView object that refers to a part of this matrix.
     */
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
    
    /**
     * @brief Creates a submatrix by removing a specified row and column.
     * @note Helper for determinant and inverse calculations.
     */
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
constexpr Vector<T, R> operator*(const Matrix<T, R, C>& lhs, const Vector<T, C>& rhs) noexcept {
    Vector<T, R> result{};
    for (int i = 0; i < R; ++i) {
        result[i] = lhs.row(i).dot(rhs);
    }
    return result;
}

// In a file where both Matrix<T,R,C> and Homo<T,N> are visible
// (e.g., at the end of homogeneous_coord.hpp, after including matrix.hpp)

/**
 * @brief Transforms a homogeneous coordinate by a matrix.
 * @details This is the core operation for applying geometric transformations.
 * The (N+1)x(N+1) matrix `lhs` is multiplied by the (N+1)-dimensional
 * homogeneous coordinate vector represented by `rhs`.
 * * @tparam T The underlying floating-point type.
 * @tparam N The original dimension of the space (e.g., 3 for 3D graphics).
 * @param lhs The (N+1)x(N+1) transformation matrix.
 * @param rhs The homogeneous coordinate to be transformed.
 * @return A new Homo<T, N> representing the transformed coordinate.
 */
template<typename T, int N>
constexpr Homogeneous<T, N> operator*(const Matrix<T, N + 1, N + 1>& lhs, const Homogeneous<T, N>& rhs) noexcept {
    // Reuse the existing Matrix * Vec operator.
    // It multiplies the matrix with the Homo's internal (N+1) vector.
    Vector<T, N + 1> result_coords = lhs * rhs.raw();
    
    // Construct a new Homo object from the resulting coordinates.
    return Homogeneous<T, N>(result_coords);
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
            result[r][c] = lhs.row(r).dot(rhs.col(c));
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
            os << mat.at(i, j) << (j == C - 1 ? "" : ",\t");
        }
        os << "\n";
    }
    os << "]";
    return os;
}

} // namespace math
