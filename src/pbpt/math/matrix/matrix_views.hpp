/**
 * @file
 * @brief Non-owning view types over matrix storage (VectorView, MatrixView).
 */
#pragma once

#include <concepts>
#include <stdexcept>
#include <type_traits>

#include "pbpt/math/basic/utils.hpp"
#include "pbpt/math/spatial/vector.hpp"

namespace pbpt::math {

// Forward declaration required by MatrixView.
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
class ConstVectorView {
protected:
    /// Pointer to the first element in the underlying storage.
    const T* m_start_ptr;
    /// Stride (in elements) between consecutive entries.
    int m_stride;

public:
    /**
     * @brief Constructs a view from a start pointer and stride.
     *
     * The caller is responsible for ensuring the lifetime of the
     * underlying data, and that at least N elements are addressable
     * with the given stride.
     */
    [[nodiscard]] constexpr ConstVectorView(const T* start, int stride) : m_start_ptr(start), m_stride(stride) {
        assert_if_ex<std::invalid_argument>([&]() { return start == nullptr; },
                                            "VectorView requires non-null data pointer");
    }

    /// Returns element @p i without bounds checking.
    constexpr const T& operator[](int i) const { return m_start_ptr[i * m_stride]; }

    /// Returns element @p i (same as operator[]).
    constexpr const T& at(int i) const { return m_start_ptr[i * m_stride]; }

    /// Number of elements in the view.
    constexpr int dims() const noexcept { return N; }

    /// Dot product between this view and a full vector.
    template <typename U>
    constexpr auto dot(const Vector<U, N>& rhs) const {
        using ResultType = std::common_type_t<T, U>;
        ResultType result = 0;
        for (int i = 0; i < N; ++i)
            result += static_cast<ResultType>((*this)[i]) * static_cast<ResultType>(rhs[i]);
        return result;
    }

    /// Dot product between two views.
    constexpr T dot(const ConstVectorView<T, N>& rhs) const {
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

    /// Visits each element with its index (const).
    template <std::invocable<const T&, int> F>
    void visit(F func) const {
        for (int i = 0; i < N; ++i)
            func((*this)[i], i);
    }
};

template <typename T, int N>
class VectorView : public ConstVectorView<T, N> {
private:
    using Base = ConstVectorView<T, N>;

public:
    using Base::Base;
    using Base::dims;
    using Base::dot;
    using Base::to_vector;
    using Base::visit;

    /// Returns element @p i without bounds checking (mutable).
    constexpr T& operator[](int i) { return const_cast<T&>(Base::operator[](i)); }

    /// Returns element @p i (same as operator[]; mutable).
    constexpr T& at(int i) { return const_cast<T&>(Base::at(i)); }

    /// Assigns from an owning vector.
    template <typename U>
        requires std::is_convertible_v<U, T>
    constexpr VectorView& operator=(const Vector<U, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = static_cast<T>(other[i]);
        return *this;
    }

    /// Assigns from another view with the same layout.
    constexpr VectorView& operator=(const ConstVectorView<T, N>& other) {
        for (int i = 0; i < N; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    /// Assigns from another mutable view.
    constexpr VectorView& operator=(const VectorView<T, N>& other) {
        return (*this) = static_cast<const ConstVectorView<T, N>&>(other);
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
class ConstMatrixView {
protected:
    /// Reference to the original matrix that owns the data.
    const Matrix<T, R, C>& m_original;
    /// Row index of the top-left element of the view.
    int m_row_start{};
    /// Column index of the top-left element of the view.
    int m_col_start{};

public:
    /// View type for a row inside the submatrix.
    using RowView = ConstVectorView<T, ViewC>;
    /// View type for a column inside the submatrix.
    using ColView = ConstVectorView<T, ViewR>;

    /// Constructs a view at the given row/column offset in the original.
    constexpr ConstMatrixView(const Matrix<T, R, C>& original, int row_start, int col_start)
        : m_original(original), m_row_start(row_start), m_col_start(col_start) {}

    /// Returns the element at (r, c) inside the view (const).
    constexpr const T& at(int r, int c) const { return m_original.at(m_row_start + r, m_col_start + c); }

    /// Returns a row-view for row r (const).
    constexpr RowView operator[](int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a row-view for row r (const).
    constexpr RowView row(int r) const { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a column-view for column c (const).
    constexpr ColView col(int c) const { return ColView(&(*this).at(0, c), C); }

    /// Copies the view contents into an owning matrix of size ViewR x ViewC.
    constexpr Matrix<T, ViewR, ViewC> to_matrix() const {
        Matrix<T, ViewR, ViewC> result;
        for (int i = 0; i < ViewR; ++i)
            for (int j = 0; j < ViewC; ++j)
                result[i][j] = (*this)[i][j];
        return result;
    }

    /// Visits all elements in the view (const).
    template <std::invocable<const T&, int, int> F>
    void visit(F func) const {
        for (int i = 0; i < ViewR; ++i)
            for (int j = 0; j < ViewC; ++j)
                func((*this).at(i, j), i, j);
    }
};

template <typename T, int R, int C, int ViewR, int ViewC>
class MatrixView : public ConstMatrixView<T, R, C, ViewR, ViewC> {
private:
    using Base = ConstMatrixView<T, R, C, ViewR, ViewC>;

public:
    using Base::to_matrix;
    using Base::visit;

    using RowView = VectorView<T, ViewC>;
    using ColView = VectorView<T, ViewR>;

    /// Constructs a mutable view at the given row/column offset in the original.
    constexpr MatrixView(Matrix<T, R, C>& original, int row_start, int col_start)
        : Base(original, row_start, col_start) {}

    /// Returns a mutable reference to the element at (r, c).
    constexpr T& at(int r, int c) {
        return const_cast<T&>(Base::at(r, c));
    }

    /// Returns a row-view for row r (mutable).
    constexpr RowView operator[](int r) { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a row-view for row r (mutable).
    constexpr RowView row(int r) { return RowView(&(*this).at(r, 0), 1); }

    /// Returns a column-view for column c (mutable).
    constexpr ColView col(int c) { return ColView(&(*this).at(0, c), C); }

    /// Assigns from another matrix of the same view size.
    constexpr MatrixView& operator=(const Matrix<T, ViewR, ViewC>& other) {
        for (int i = 0; i < ViewR; ++i)
            for (int j = 0; j < ViewC; ++j)
                (*this).at(i, j) = other.at(i, j);
        return *this;
    }

    /// Visits all elements in the view (mutable).
    template <std::invocable<T&, int, int> F>
    void visit(F func) {
        for (int i = 0; i < ViewR; ++i)
            for (int j = 0; j < ViewC; ++j)
                func((*this).at(i, j), i, j);
    }
};

}  // namespace pbpt::math
