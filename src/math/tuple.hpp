#pragma once

#include <array>
#include <type_traits>

#include "operator.hpp"
#include "utils.hpp"

namespace pbpt::math {

template <template <typename, int> typename Derived, typename T, int N>
    requires(N > 0)
class Tuple {
protected:
    std::array<T, N> m_data{};

public:
    
    static constexpr auto filled(T value) {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = value;
        return t;
    }

    static constexpr auto zeros() { return filled(T(0)); }
    static constexpr auto ones() { return filled(T(1)); }

    static auto random() {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = rand<T>();
        return t;
    }

    static constexpr auto from_array(const std::array<T, N>& arr) {
        Derived<T, N> t;
        for (int i = 0; i < N; ++i)
            t[i] = arr[i];
        return t;
    }

    // -- Constructors --
    constexpr Tuple() = default;

    template<typename ...Args>
        requires(sizeof...(Args) == N)
    explicit constexpr Tuple(Args&&... args) : m_data{ static_cast<T>(args)... } {}

    template<template <typename, int> typename OtherDerived, typename U>
    constexpr Tuple(const Tuple<OtherDerived, U, N>& other) {
        for (int i = 0; i < N; ++i) {
            m_data[i] = static_cast<T>(other[i]);
        }
    }

    constexpr std::array<T, N>& to_array() { return m_data; }
    constexpr std::array<T, N> to_array() const { return m_data; }
    constexpr int dims() const { return N; }

    constexpr bool has_nan() const {
        if constexpr (std::is_floating_point_v<T>) {
            for (int i = 0; i < N; ++i)
                if (std::isnan(m_data[i]))
                    return true;
        }
        return false;
    }

    // -- Accessors --
    constexpr T& operator[](int i) {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }

    constexpr const T& operator[](int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::operator[] index out of range");
        return m_data[i];
    }

    constexpr const T& at(int i) const {
        assert_if([&]() { return i < 0 || i >= N; }, "Tuple::at index out of range");
        return m_data[i];
    }

    constexpr T& x() requires (N > 0) { return m_data[0]; }
    constexpr T& y() requires (N > 1) { return m_data[1]; }
    constexpr T& z() requires (N > 2) { return m_data[2]; }
    constexpr T& w() requires (N > 3) { return m_data[3]; }

    constexpr const T& x() const requires (N > 0) { return m_data[0]; }
    constexpr const T& y() const requires (N > 1) { return m_data[1]; }
    constexpr const T& z() const requires (N > 2) { return m_data[2]; }
    constexpr const T& w() const requires (N > 3) { return m_data[3]; }

    // -- Equality --
    template <typename U>
    constexpr bool operator==(const Tuple<Derived, U, N>& rhs) const {
        for (int i = 0; i < N; ++i)
            if (is_not_equal(m_data[i], rhs[i]))
                return false;
        return true;
    }

    template <typename U>
    constexpr bool operator!=(const Tuple<Derived, U, N>& rhs) const {
        return !(*this == rhs);
    }

    constexpr Derived<T, N> operator-() const {
        Derived<T, N> result;
        for (int i = 0; i < N; ++i)
            result[i] = -(*this)[i];
        return result;
    }
   
    // -- Type Conversion --
    template <typename U, int M>
        requires(M > 0) && std::is_convertible_v<T, U>
    constexpr auto cast() const {
        Derived<U, M> result;
        int k = std::min(N, M);
        for (int i = 0; i < k; ++i)
            result[i] = static_cast<U>(m_data[i]);
        for (int i = k; i < M; ++i)
            result[i] = U(0);
        return result;
    }

    template <typename U>
        requires std::is_convertible_v<T, U>
    constexpr auto type_cast() const {
        return cast<U, N>();
    }

    template <int M>
        requires(M > 0)
    constexpr auto dim_cast() const {
        return cast<T, M>();
    }

    template <std::invocable<T&, int> F>
    constexpr void visit(F&& f) {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    template <std::invocable<const T&, int> F>
    constexpr void visit(F&& f) const {
        for (int i = 0; i < N; ++i)
            f((*this)[i], i);
    }

    int max_dim() const {
        int max_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], (*this)[max_i]))
                max_i = i;
        return max_i;
    }

    T max() const {
        T max_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_greater((*this)[i], max_v))
                max_v = (*this)[i];
        return max_v;
    }

    int min_dim() const {
        int min_i = 0;
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], (*this)[min_i]))
                min_i = i;
        return min_i;
    }

    T min() const {
        T min_v = (*this)[0];
        for (int i = 1; i < N; ++i)
            if (is_less((*this)[i], min_v))
                min_v = (*this)[i];
        return min_v;
    }

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr auto permuted(Args... args) const {
        Derived<T, N> result;
        int          i = 0;
        ((result[i++] = (*this)[args]), ...);
        return result;
    }

    template <typename... Args>
        requires(sizeof...(Args) == N)
    constexpr auto& permute(Args... args) {
        *this = permuted(args...);
        return *this;
    }

    constexpr bool is_all_zero() const {
        for (int i = 0; i < N; i++) {
            if (!is_zero(m_data[i]))
                return false;
        }
        return true;
    }

    constexpr auto abs() const {
        Derived<T, N> result{};
        for (int i = 0; i < N; i++) {
            result[i] = pbpt::math::abs((*this)[i]);
        }
        return result;
    }

    constexpr auto sum() const {
        T result = T(0);
        for (int i = 0; i < N; i++) {
            result += (*this)[i];
        }
        return result;
    }

    constexpr auto average() const {
        T result = T(0);
        for (int i = 0; i < N; i++) {
            result += (*this)[i];
        }
        return result / N;
    }

    constexpr const Derived<T, N>& as_derived() const {
        return *static_cast<const Derived<T, N>&>(*this);
    }

    constexpr Derived<T, N>& as_derived() {
        return *static_cast<Derived<T, N>&>(*this);
    }
};

}  // namespace pbpt::math
