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

/**
 * @brief Forward declaration of a fixed-size dense matrix.
 *
 * See the full `Matrix` definition later in this file for details.
 */
template <typename T, int R, int C>
    requires std::is_floating_point_v<T> && (R > 0) && (C > 0)
class Matrix;

template <typename T, int N>
/**
 * @brief Non-owning strided view over a contiguous vector of elements.
 *
 * A `VectorView` behaves like a fixed-size `Vector<T,N>` but only
 * stores a pointer and a stride; it never allocates or owns memory.
 * It is typically used to expose rows or columns of a matrix.
 *
 * @tparam T Element type.
 * @tparam N Number of elements visible through the view.
 */
class VectorView {
protected:
    /// Pointer to the first element in the underlying storage.
    const T* m_start_ptr;
    /// Stride (in elements) between consecutive entries.
    int      m_stride;

public:
    /// @brief Creates a non-owning view of data elements with specified stride.
    /// @warning The caller must ensure that the referenced data remains valid for the lifetime of this view.
    /// @param start Pointer to the first element (must not be nullptr)
    /// @param stride Distance between consecutive elements in the view
    /**
     * @brief Constructs a view from a start pointer and stride.
     *
     * The caller is responsible for ensuring the lifetime of the
     * underlying data, and that at least N elements are addressable
     * with the given stride.
     */
    [[nodiscard]] constexpr VectorView(const T* start, int stride) 
        : m_start_ptr(start), m_stride(stride) {
        assert_if_ex<std::invalid_argument>([&]() { return start == nullptr; }, 
                                           "VectorView requires non-null data pointer");
    }

    /// Returns element @p i without bounds checking.
    constexpr const T& operator[](int i) const { return m_start_ptr[i * m_stride]; }
    /// Returns element @p i without bounds checking (mutable).
    constexpr T&       operator[](int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    /// Returns element @p i (same as operator[]).
    constexpr const T& at(int i) const { return m_start_ptr[i * m_stride]; }
    /// Returns element @p i (same as operator[]; mutable).
    constexpr T&       at(int i) { return const_cast<T&>(this->m_start_ptr[i * this->m_stride]); }

    /// Number of elements in the view.
    constexpr int dims() const noexcept { return N; }

    /// Dot product between this view and a full vector.
    constexpr T dot(const Vector<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i)
            result += (*this)[i] * rhs[i];
        return result;
    }

    /// Dot product between two views.
    constexpr T dot(const VectorView<T, N>& rhs) const {
        T result = 0;
        for (int i = 0; i < N; ++i)
            result += (*this)[i] * rhs[i];
        return result;
    }

    /// Copies the view contents into an owning `Vector<T,N>`.
    constexpr Vector<T, N> to_vector() const {
        Vector<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = (*this)[i];
        return result;
    }

    /// Assigns from an owning vector.
    constexpr VectorView& operator=(const Vector<T, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    /// Assigns from another view with the same layout.
    constexpr VectorView& operator=(const VectorView<T, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    /// Visits each element with its index (const).
    template <std::invocable<const T&, int> F>
    void visit(F func) const {
        for (int i = 0; i < N; ++i)
            func((*this)[i], i);
    }

    /// Visits each element with its index (mutable).
    template <std::invocable<T&, int> F>
    void visit(F func) {
        for (int i = 0; i < N; ++i)
            func((*this)[i], i);
    }
};

template <typename T, int R, int C, int ViewR, int ViewC>
/**
 * @brief Non-owning rectangular view into a matrix.
 *
 * Provides a window of size ViewR x ViewC into an existing
 * `Matrix<T,R,C>`, exposing rows and columns as `VectorView`s.
 */
class MatrixView {
protected:
    /// Reference to the original matrix that owns the data.
    const Matrix<T, R, C>& m_original;
    /// Row index of the top-left element of the view.
    int                    m_row_start{};
    /// Column index of the top-left element of the view.
    int                    m_col_start{};

public:
    /// View type for a row inside the submatrix.
    using RowView = VectorView<T, ViewC>;
    /// View type for a column inside the submatrix.
    using ColView = VectorView<T, ViewR>;

    /// Constructs a view at the given row/column offset in the original.
    constexpr MatrixView(const Matrix<T, R, C>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    /// Returns the element at (r, c) inside the view (const).
    constexpr const T& at(int r, int c) const { return m_original.at(m_row_start + r, m_col_start + c); }

    /// Returns a row-view for row r (const).
    constexpr const RowView operator[](int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a row-view for row r (mutable).
    constexpr RowView operator[](int r) { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a row-view for row r (const).
    constexpr const RowView row(int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a column-view for column c (mutable).
    constexpr ColView col(int c) { return ColView(&(*this).at(0, c), ViewC); }

    /// Copies the view contents into an owning matrix of size ViewR x ViewC.
    constexpr Matrix<T, ViewR, ViewC> to_matrix() const {
        Matrix<T, ViewR, ViewC> result;
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                result[i][j] = (*this)[i][j];
            }
        }
        return result;
    }

    /// Returns a mutable reference to the element at (r, c).
    constexpr T& at(int r, int c) {
        return const_cast<T&>(this->m_original.at(this->m_row_start + r, this->m_col_start + c));
    }

    /// Assigns from another matrix of the same view size.
    constexpr MatrixView& operator=(const Matrix<T, ViewR, ViewC>& other) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                (*this).at(i, j) = other.at(i, j);
            }
        }
        return *this;
    }

    /// Visits all elements in the view (mutable).
    template<std::invocable<T&, int, int> F>
    void visit(F func) {
        for (int i = 0; i < ViewR; ++i) {
            for (int j = 0; j < ViewC; ++j) {
                func((*this).at(i, j), i, j);
            }
        }
    }

    /// Visits all elements in the view (const).
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
/**
 * @brief Dense column-major matrix with fixed compile-time size.
 *
 * Provides basic linear algebra operations such as addition,
 * scalar and matrix multiplication, determinants, inversion and
 * Gaussian elimination routines. The matrix elements are stored
 * in a contiguous `std::array`.
 *
 * @tparam T Floating-point element type.
 * @tparam R Number of rows.
 * @tparam C Number of columns.
 */
class Matrix {
private:
    std::array<T, R * C> m_data{};

public:
    /// Alias for a rectangular submatrix view.
    template <int ViewR, int ViewC>
    using MatView = MatrixView<T, R, C, ViewR, ViewC>;
    /// Alias for a non-owning row view.
    using RowView = VectorView<T, C>;
    /// Alias for a non-owning column view.
    using ColView = VectorView<T, R>;

    // Static factory methods (unified with Vector design)
    /// Returns a matrix with all entries initialized to zero.
    static constexpr Matrix zeros() noexcept { return Matrix{}; }
    
    /// Returns a matrix with all entries initialized to @p value.
    static constexpr Matrix filled(T value) noexcept {
        Matrix result{};
        for (int i = 0; i < R * C; ++i)
            result.m_data[i] = value;
        return result;
    }
    
    /// Returns a matrix with all entries equal to one.
    static constexpr Matrix ones() noexcept { return filled(T(1)); }
    
    /**
     * @brief Returns the identity matrix (only defined for square matrices).
     */
    static constexpr Matrix identity() noexcept
        requires(R == C)
    {
        Matrix result{};
        for (int i = 0; i < R; ++i) {
            result[i][i] = T(1);
        }
        return result;
    }

    /**
     * @brief Constructs a matrix from a flat array in row-major order.
     */
    static constexpr Matrix from_array(const std::array<T, R * C>& arr) noexcept {
        Matrix result{};
        result.m_data = arr;
        return result;
    }

    /**
     * @brief Constructs a matrix with random entries using `rand<T>()`.
     */
    static auto random() {
        Matrix result{};
        for (int i = 0; i < R * C; ++i) {
            result.m_data[i] = rand<T>();
        }
        return result;
    }

    // Constructors
    constexpr Matrix() noexcept = default;

    /**
     * @brief Constructs a matrix from column vectors.
     *
     * Expects exactly C column vectors of type `Vector<T,R>`.
     */
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

    /**
     * @brief Constructs a matrix from a flat list of R*C scalar values.
     *
     * Values are stored in row-major order.
     */
    template <std::convertible_to<T>... Vals>
        requires(sizeof...(Vals) == R * C && (std::is_arithmetic_v<std::remove_cvref_t<Vals>> && ...))
    constexpr Matrix(Vals&&... vals) noexcept : m_data{static_cast<T>(std::forward<Vals>(vals))...} {}

    // Copy/conversion constructor from different matrix types
    /**
     * @brief Converting copy-constructor from another Matrix type.
     *
     * Copies all elements from @p other, converting them to type @p T.
     */
    template <typename U>
        requires std::convertible_to<U, T>
    constexpr Matrix(const Matrix<U, R, C>& other) noexcept {
        for (int i = 0; i < R * C; ++i) {
            m_data[i] = static_cast<T>(other.data()[i]);
        }
    }

    // Accessors with improved error handling
    /// Returns the element at (row, column) with bounds checking (const).
    constexpr const T& at(int x, int y) const {
        assert_if_ex<std::out_of_range>([&]() { return x < 0 || x >= R || y < 0 || y >= C; }, 
                                       "Matrix index out of range");
        return m_data[x * C + y];
    }

    /// Returns the element at (row, column) with bounds checking (mutable).
    constexpr T& at(int x, int y) {
        assert_if_ex<std::out_of_range>([&]() { return x < 0 || x >= R || y < 0 || y >= C; }, 
                                       "Matrix index out of range");
        return m_data[x * C + y];
    }

    /// Returns a view of the r-th row.
    constexpr RowView operator[](int r) { return RowView(&(*this).at(r, 0), 1); }
    /// Returns a view of the r-th row (const).
    constexpr const RowView operator[](int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a view of the r-th row.
    constexpr RowView row(int r) { return RowView(&(*this).at(r, 0), 1); }
    /// Returns a view of the r-th row (const).
    constexpr const RowView row(int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a view of the c-th column.
    constexpr ColView col(int c) { return ColView(&(*this).at(0, c), C); }
    /// Returns a view of the c-th column (const).
    constexpr const ColView col(int c) const { return ColView(&(*this).at(0, c), C); }

    /// Number of rows.
    constexpr int row_dims() const noexcept { return R; }
    /// Number of columns.
    constexpr int col_dims() const noexcept { return C; }
    /// Total number of stored elements.
    constexpr int dims() const noexcept { return R * C; }  // Total elements (unified with Vector)

    /// Pointer to the underlying contiguous storage (const).
    const T* data() const noexcept { return m_data.data(); }
    /// Pointer to the underlying contiguous storage (mutable).
    T*       data() noexcept { return m_data.data(); }
    
    /// Returns a copy of the underlying array storage.
    constexpr std::array<T, R * C> to_array() const noexcept { return m_data; }

    // Unified comparison operators
    /**
     * @brief Element-wise equality comparison with another matrix.
     *
     * Uses a floating-point aware comparison for each entry.
     */
    template <typename U>
    constexpr bool operator==(const Matrix<U, R, C>& rhs) const noexcept {
        // Use std::ranges::equal with custom comparator for floating-point comparison
        return std::ranges::equal(m_data, rhs.to_array(), 
                                 [](const T& a, const U& b) { 
                                     return is_equal(a, static_cast<T>(b)); 
                                 });
    }

    /// Logical negation of operator==.
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
    /// Adds another matrix to this one in-place.
    template <typename U>
    constexpr Matrix& operator+=(const Matrix<U, R, C>& rhs) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] += static_cast<T>(rhs.data()[i]);
        return *this;
    }

    /// Subtracts another matrix from this one in-place.
    template <typename U>
    constexpr Matrix& operator-=(const Matrix<U, R, C>& rhs) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] -= static_cast<T>(rhs.data()[i]);
        return *this;
    }

    /// Multiplies all entries by a scalar in-place.
    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Matrix& operator*=(const U& scalar) noexcept {
        for (int i = 0; i < R * C; ++i)
            m_data[i] *= static_cast<T>(scalar);
        return *this;
    }

    /// Divides all entries by a scalar in-place (asserting non-zero divisor).
    template <typename U>
        requires std::is_arithmetic_v<U>
    constexpr Matrix& operator/=(const U& scalar) noexcept {
        assert_if_ex<std::domain_error>([&scalar]() { return is_equal(scalar, U(0)); }, 
                                       "Division by zero in matrix division");
        for (int i = 0; i < R * C; ++i)
            m_data[i] /= static_cast<T>(scalar);
        return *this;
    }

    /// Returns the transposed copy of this matrix.
    constexpr Matrix<T, C, R> transposed() const noexcept {
        Matrix<T, C, R> result{};
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result[j][i] = (*this)[i][j];
            }
        }
        return result;
    }

    /**
     * @brief Transposes this matrix in-place (square matrices only).
     */
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

    /**
     * @brief Determinant of a square matrix.
     *
     * Uses specialized formulas for small sizes and a Laplace
     * expansion in the general case.
     */
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

    /**
     * @brief Returns the inverse of this matrix (square matrices only).
     *
     * Uses hard-coded fast paths for 1x1, 2x2, 3x3 and 4x4 matrices
     * and a general cofactor-based method otherwise.
     *
     * Throws if the matrix is singular.
     */
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

    /**
     * @brief In-place inversion via the adjugate matrix (square matrices only).
     */
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

    /// Returns a view of a submatrix starting at (row_start, col_start).
    template <int ViewR, int ViewC>
    constexpr Matrix::MatView<ViewR, ViewC> view(int row_start, int col_start) {
        assert_if_ex<std::out_of_range>([&]() { return row_start + ViewR > R || col_start + ViewC > C; }, 
                                       "View dimensions exceed matrix bounds");
        return Matrix::MatView<ViewR, ViewC>(*this, row_start, col_start);
    }

    /// Returns a const view of a submatrix starting at (row_start, col_start).
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
    /// Matrix addition with type promotion.
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

    /// Matrix subtraction with type promotion.
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

    /// Unary minus: returns a matrix with all entries negated.
    constexpr Matrix operator-() const noexcept {
        Matrix result{};
        for (int i = 0; i < R * C; ++i)
            result.m_data[i] = -m_data[i];
        return result;
    }

    /// Scalar multiplication with type promotion.
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

    /// Scalar multiplication from the left (friend, commutative with `operator*` above).
    template <typename U>
        requires std::is_arithmetic_v<U>
    friend constexpr auto operator*(const U& scalar, const Matrix<T, R, C>& mat) noexcept {
        return mat * scalar;
    }

    /// Scalar division with type promotion.
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

    /// Matrix–vector product with type promotion.
    template <typename U>
    constexpr auto operator*(const Vector<U, C>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Vector<ResultType, R> result{};
        for (int r = 0; r < R; ++r) {
            result[r] = row(r).dot(rhs);
        }
        return result;
    }

    /// Matrix–homogeneous-vector product with type promotion.
    template <typename U>
    constexpr auto operator*(const Homogeneous<U, C>& rhs) const noexcept {
        using ResultType = std::common_type_t<T, U>;
        Homogeneous<ResultType, R> result{};
        for (int r = 0; r < R; ++r) {
            result[r] = row(r).dot(rhs.to_vector_raw());
        }
        return result;
    }

    /// Matrix–matrix product with type promotion.
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

    /// Returns true if all entries are (approximately) zero.
    constexpr bool is_all_zero() const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                if (!pbpt::math::is_zero((*this).at(r, c)))
                    return false;
            }
        }
        return true;
    }

    /// Returns true if the matrix is (approximately) the identity.
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

    /// Returns true if any entry is NaN (floating-point matrices only).
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
    /// Visits each matrix element with its (row, col) indices (mutable).
    template <std::invocable<T&, int, int> F>
    constexpr void visit(F&& f) {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                f((*this).at(r, c), r, c);
            }
        }
    }

    /// Visits each matrix element with its (row, col) indices (const).
    template <std::invocable<const T&, int, int> F>
    constexpr void visit(F&& f) const {
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                f((*this).at(r, c), r, c);
            }
        }
    }

    // elementary row operations
    /// Swaps two rows in-place; returns *this.
    constexpr Matrix<T, R, C>& swap_rows(int r1, int r2) {
        assert_if_ex<std::out_of_range>(
            r1 < 0 || r1 >= R || r2 < 0 || r2 >= R, 
            "Row index out of range in swap_rows"
        );

        for (int c = 0; c < C; ++c) {
            std::swap((*this).at(r1, c), (*this).at(r2, c));
        }
        return *this;
    }

    /// Scales a row by the given scalar; returns *this.
    constexpr Matrix<T, R, C>& scale_row(int r, T scalar) {
        assert_if_ex<std::out_of_range>(
            r < 0 || r >= R, 
            "Row index out of range in scale_row"
        );

        for (int c = 0; c < C; ++c) {
            (*this).at(r, c) *= scalar;
        }
        return *this;
    }

    /// Adds a scaled source row to a destination row; returns *this.
    constexpr Matrix<T, R, C>& add_scaled_row(int dest_r, int src_r, T scalar) {
        assert_if_ex<std::out_of_range>(
            dest_r < 0 || dest_r >= R || src_r < 0 || src_r >= R, 
            "Row index out of range in add_scaled_row"
        );

        for (int c = 0; c < C; ++c) {
            (*this).at(dest_r, c) += scalar * (*this).at(src_r, c);
        }
        return *this;
    }

    /// Swaps two columns in-place; returns *this.
    constexpr Matrix<T, R, C>& swap_cols(int c1, int c2) {
        assert_if_ex<std::out_of_range>(
            c1 < 0 || c1 >= C || c2 < 0 || c2 >= C, 
            "Column index out of range in swap_cols"
        );
        for (int r = 0; r < R; ++r) {
            std::swap((*this).at(r, c1), (*this).at(r, c2));
        }
        return *this;
    }

    /// Scales a column by the given scalar; returns *this.
    constexpr Matrix<T, R, C>& scale_col(int c, T scalar) {
        assert_if_ex<std::out_of_range>(
            c < 0 || c >= C, 
            "Column index out of range in scale_col"
        );
        for (int r = 0; r < R; ++r) {
            (*this).at(r, c) *= scalar;
        }
        return *this;
    }

    /// Adds a scaled source column to a destination column; returns *this.
    constexpr Matrix<T, R, C>& add_scaled_col(int dest_c, int src_c, T scalar) {
        assert_if_ex<std::out_of_range>(
            dest_c < 0 || dest_c >= C || src_c < 0 || src_c >= C, 
            "Column index out of range in add_scaled_col"
        );
        for (int r = 0; r < R; ++r) {
            (*this).at(r, dest_c) += scalar * (*this).at(r, src_c);
        }
        return *this;
    }

    //helpers: find index of max abs value in row/col
    /// Returns index and value of maximum absolute entry in a column slice.
    constexpr std::pair<int, T> argmax_abs_in_col(int c, int start_row = 0, int end_row = R) const {
        assert_if_ex<std::out_of_range>(
            c < 0 || c >= C, 
            "Column index out of range in argmax_abs_in_col"
        );

        int max_index = start_row;
        T max_value = (*this).at(start_row, c);
        T max_abs_value = std::abs((*this).at(start_row, c));
        for (int r = start_row + 1; r < end_row; ++r) {
            T abs_value = std::abs((*this).at(r, c));
            if (abs_value > max_abs_value) {
                max_abs_value = abs_value;
                max_value = (*this).at(r, c);
                max_index = r;
            }
        }
        return {max_index, max_value};
    }

    /// Returns index and value of maximum absolute entry in a row slice.
    constexpr std::pair<int, T> argmax_abs_in_row(int r, int start_col = 0, int end_col = C) const {
        assert_if_ex<std::out_of_range>(
            r < 0 || r >= R, 
            "Row index out of range in argmax_abs_in_row"
        );

        int max_index = start_col;
        T max_value = (*this).at(r, start_col);
        T max_abs_value = std::abs((*this).at(r, start_col));
        for (int c = start_col + 1; c < end_col; ++c) {
            T abs_value = std::abs((*this).at(r, c));
            if (abs_value > max_abs_value) {
                max_abs_value = abs_value;
                max_value = (*this).at(r, c);
                max_index = c;
            }
        }
        return {max_index, max_value};
    }
    
    /**
     * @brief Result of Gaussian elimination in row echelon form.
     *
     * Contains the transformed matrix, its rank, the number of row
     * swaps performed and the accumulated inverse scale factor used
     * during elimination (useful for determinant computation).
     */
    struct EchelonResult {
        /// Matrix in (reduced) row echelon form.
        Matrix<T, R, C> matrix;
        /// Rank of the original matrix.
        int rank{};
        /// Number of row swaps performed (for determinant sign).
        int row_swaps{};
        /// Inverse of the product of all scaling factors used.
        T total_scale_inv{1}; // the inverse of the product of all scaling factors used
    };

    //transform to row echelon form and reduced row echelon form, return rank and number of row swaps
    /**
     * @brief Transforms the matrix to row echelon form in-place.
     *
     * Uses Gaussian elimination with partial pivoting and returns
     * the resulting echelon matrix together with its rank and the
     * accumulated scaling information.
     */
    constexpr EchelonResult ref_inplace() {
        int r = 0;
        int swaps = 0;
        T total_scale_inv = T(1);
        for (int c = 0; c < C && r < R; c ++) {
            auto [pr, pv] = argmax_abs_in_col(c, r);
            if (is_zero(pv)) { continue; }
            if (pr != r) swap_rows(r, pr), swaps ++; 
            scale_row(r, T(1) / pv), total_scale_inv *= pv;
            for (int rr = r + 1; rr < R; rr ++) {
                T s = -(*this).at(rr, c);
                add_scaled_row(rr, r, s);
            }
            r ++;
        }
        visit([&](T& x, int, int){ if (is_zero(x)) x = T(0); });
        return {(*this), r, swaps, total_scale_inv};
    } 

    /// Returns a copy of this matrix in row echelon form.
    constexpr EchelonResult ref() const {
        Matrix<T, R, C> temp = *this;
        return temp.ref_inplace();
    }

    /**
     * @brief Transforms the matrix to reduced row echelon form in-place.
     */
    constexpr EchelonResult rref_inplace() {
        int r = 0;
        int swaps = 0;
        T total_scale_inv = T(1);
        for (int c = 0; c < C && r < R; c ++) {
            auto [pr, pv] = argmax_abs_in_col(c, r);
            if (is_zero(pv)) {continue; }
            if (pr != r) swap_rows(r, pr), swaps ++;
            scale_row(r, T(1) / pv), total_scale_inv *= pv;
            for (int rr = 0; rr < R; rr ++) {
                if (rr == r) continue;
                T s = -(*this).at(rr, c);
                add_scaled_row(rr, r, s);
            }
            r ++;
        }
        visit([&](T& x, int, int){ if (is_zero(x)) x = T(0); });
        return {(*this), r, swaps, total_scale_inv};
    }

    /// Returns a copy of this matrix in reduced row echelon form.
    constexpr EchelonResult rref() const {
        Matrix<T, R, C> temp = *this;
        return temp.rref_inplace();
    }

    // functions
    /// Returns the rank of the matrix computed via row echelon form.
    int rank() const {
        Matrix<T, R, C> temp = *this;
        return temp.ref_inplace().rank;
    }

    /// Returns true if the matrix is invertible (square matrices only).
    bool is_invertible() const requires(R == C) {
        Matrix<T, R, C> temp = *this;
        auto [mat, rank, swaps, _] = temp.ref_inplace();
        return rank == R;
    }

    /**
     * @brief Inverse computed via augmented RREF (returned as a new matrix).
     */
    Matrix<T, R, C> inversed_rref() const requires(R == C) {
        Matrix<T, R, 2 * C> augmented{};
        // Create augmented matrix [A | I]
        for (int r = 0; r < R; r ++) {
            for (int c = 0; c < C; c ++) {
                augmented.at(r, c) = (*this).at(r, c);  
                augmented.at(r, c + C) = (r == c) ? T(1) : T(0);
            }
        }
        augmented.rref_inplace();
        Matrix<T, R, C> result{};
        for (int r = 0; r < R; r ++) {
            for (int c = 0; c < C; c ++) {
                result.at(r, c) = augmented.at(r, c + C);
            }
        }
        return result;
    }

    /**
     * @brief In-place inversion via the augmented RREF method.
     */
    Matrix<T, R, C>& inverse_rref() requires(R == C) {
        Matrix<T, R, 2 * C> augmented{};
        // Create augmented matrix [A | I]
        for (int r = 0; r < R; r ++) {
            for (int c = 0; c < C; c ++) {
                augmented.at(r, c) = (*this).at(r, c);  
                augmented.at(r, c + C) = (r == c) ? T(1) : T(0);
            }
        }
        augmented.rref_inplace();
        for (int r = 0; r < R; r ++) {
            for (int c = 0; c < C; c ++) {
                (*this).at(r, c) = augmented.at(r, c + C);
            }
        }
        return *this;
    }

    /**
     * @brief Determinant computed via row echelon form.
     *
     * Uses ref_inplace() and accounts for row swaps and scaling to
     * reconstruct the determinant.
     */
    T determinant_ref() const requires(R == C) {
        Matrix<T, R, C> temp = *this;
        auto [mat, rank, swaps, total_scale_inv] = temp.ref_inplace();
        T det = (swaps % 2 == 0) ? T(1) : T(-1);
        for (int i = 0; i < R; i ++) {
            det *= temp.at(i, i);
        }
        return det * total_scale_inv;
    }
    
};

using Mat2   = Matrix<Float, 2, 2>;
using Mat3   = Matrix<Float, 3, 3>;
using Mat4   = Matrix<Float, 4, 4>;
using Mat3x4 = Matrix<Float, 3, 4>;
using Mat4x3 = Matrix<Float, 4, 3>;

// slove least mean square problem: find M that minimizes ||M * A - B||
template <typename T, int N, int ConstraintsCount>
inline constexpr Matrix<T, N, N> solve_LMS(
    const Matrix<T, N, ConstraintsCount>& A, 
    const Matrix<T, N, ConstraintsCount>& B
) {
    Matrix<T, N, N> result = Matrix<T, N, N>::zeros();
    auto At = A.transposed();
    auto AAt = A * At;
    auto BAt = B * At;
    auto AAt_inv = AAt.inversed_rref();
    result = BAt * AAt_inv;
    return result;
}

}  // namespace pbpt::math
