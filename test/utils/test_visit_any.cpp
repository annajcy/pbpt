#include <gtest/gtest.h>
#include <variant>
#include <string>
#include "utils/visit_any.hpp"

using namespace pbpt::utils;

// Test fixture or just simple tests
TEST(VisitAnyTest, VisitOneVariant) {
    std::variant<int, double, std::string> v = 42;
    
    // Visitor that returns a string description
    auto visitor = [](auto&& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int>) {
            return "int: " + std::to_string(arg);
        } else if constexpr (std::is_same_v<T, double>) {
            return "double: " + std::to_string(arg);
        } else if constexpr (std::is_same_v<T, std::string>) {
            return "string: " + arg;
        } else {
            return "unknown";
        }
    };

    EXPECT_EQ(visit_one(visitor, v), "int: 42");

    v = 3.14;
    EXPECT_EQ(visit_one(visitor, v), "double: 3.140000"); // std::to_string default precision

    v = "hello";
    EXPECT_EQ(visit_one(visitor, v), "string: hello");
}

TEST(VisitAnyTest, VisitOneDirectType) {
    int i = 100;
    double d = 1.23;
    std::string s = "world";

    auto visitor = [](auto&& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int>) {
            return "int";
        } else if constexpr (std::is_same_v<T, double>) {
            return "double";
        } else if constexpr (std::is_same_v<T, std::string>) {
            return "string";
        }
        return "unknown";
    };

    EXPECT_EQ(visit_one(visitor, i), "int");
    EXPECT_EQ(visit_one(visitor, d), "double");
    EXPECT_EQ(visit_one(visitor, s), "string");
}

// Ensure it modifies reference correctly
TEST(VisitAnyTest, VisitOneModification) {
    std::variant<int, float> v = 10;
    
    visit_one([](auto& arg) {
        arg += 5;
    }, v);

    EXPECT_EQ(std::get<int>(v), 15);

    int direct = 20;
    visit_one([](auto& arg) {
        arg += 5;
    }, direct);

    EXPECT_EQ(direct, 25);
}

TEST(VisitAnyTest, VisitAnyAllVariant) {
    std::variant<int, double> v1 = 1;
    std::variant<int, double> v2 = 2.0;

    auto result = visit_any([](auto a, auto b) {
        return static_cast<double>(a) + static_cast<double>(b);
    }, v1, v2);

    EXPECT_DOUBLE_EQ(result, 3.0);
}

TEST(VisitAnyTest, VisitAnyAllDirect) {
    int a = 10;
    double b = 20.5;

    auto result = visit_any([](auto x, auto y) {
        return x + y;
    }, a, b);

    EXPECT_DOUBLE_EQ(result, 30.5);
}
