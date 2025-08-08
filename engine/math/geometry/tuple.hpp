#pragma once

#include "../global/function.hpp"
#include "../global/operator.hpp"
#include "../global/type_alias.hpp"
#include "../global/utils.hpp"

#include <array>
#include <concepts>
#include <type_traits>
#include <iostream>

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

    template<template <typename, int> typename OtherDerived, typename U, int M>
    constexpr Tuple(const Tuple<OtherDerived, U, M>& other) {
        constexpr int min_size = (N < M) ? N : M;
        for (int i = 0; i < min_size; ++i) {
            m_data[i] = static_cast<T>(other[i]);
        }
        for (int i = min_size; i < N; ++i) {
            m_data[i] = T{};
        }
    }

    constexpr std::array<T, N> to_array() const { return m_data; }
    constexpr int dims() const { return N; }

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

    // -- Output --
    friend std::ostream& operator<<(std::ostream& os, const Tuple& t) {
        os << Derived<T, N>::name() << "(";
        for (int i = 0; i < N; ++i)
            os << t[i] << (i == N - 1 ? "" : ", ");
        os << ")";
        return os;
    }
};

}  // namespace pbpt::math
