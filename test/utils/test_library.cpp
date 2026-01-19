#include <gtest/gtest.h>
#include <variant>
#include <string>
#include "utils/library.hpp"

using namespace pbpt::utils;

// --- 测试普通类型 ---
TEST(LibraryTest, NonVariantType) {
    Library<double, std::string> lib;
    
    // Add items
    int id1 = lib.add_item("Hello");
    int id2 = lib.add_item("World");

    ASSERT_EQ(lib.size(), 2);
    EXPECT_EQ(lib.get(id1), "Hello");
    EXPECT_EQ(lib.get(id2), "World");

    // Modify item
    lib.get(id1) = "Aloha";
    EXPECT_EQ(lib.get(id1), "Aloha");

    // Remove item
    lib.remove(id1);
    ASSERT_EQ(lib.size(), 1);
    
    // Check stability of other IDs
    EXPECT_EQ(lib.get(id2), "World");
    
    // Check removed access throws
    EXPECT_THROW(lib.get(id1), std::out_of_range);
}

// --- 测试 Variant 类型 ---
using TestVariant = std::variant<int, double, std::string>;

TEST(LibraryTest, VariantType) {
    Library<double, TestVariant> lib;

    int id_int = lib.add_item(42);
    int id_dbl = lib.add_item(3.14);
    int id_str = lib.add_item(std::string("Variant"));

    ASSERT_EQ(lib.size(), 3);

    // Verify types and values
    TestVariant& v1 = lib.get(id_int);
    EXPECT_TRUE(std::holds_alternative<int>(v1));
    EXPECT_EQ(std::get<int>(v1), 42);

    TestVariant& v2 = lib.get(id_dbl);
    EXPECT_TRUE(std::holds_alternative<double>(v2));
    EXPECT_DOUBLE_EQ(std::get<double>(v2), 3.14);

    // Remove one
    lib.remove(id_dbl);
    ASSERT_EQ(lib.size(), 2);
    EXPECT_THROW(lib.get(id_dbl), std::out_of_range);
    
    // Others remain valid
    EXPECT_EQ(std::get<int>(lib.get(id_int)), 42);
    EXPECT_EQ(std::get<std::string>(lib.get(id_str)), "Variant");
}

// --- 测试 NamedLibrary (非 Variant) ---
TEST(NamedLibraryTest, BasicUsage) {
    NamedLibrary<double, int> named_lib;

    int id_a = named_lib.add_item("A", 100);
    int id_b = named_lib.add_item("B", 200);

    // Access by Name
    EXPECT_EQ(named_lib.get("A"), 100);
    EXPECT_EQ(named_lib.get("B"), 200);

    // Access by ID
    EXPECT_EQ(named_lib.get(id_a), 100);

    // Remove by Name
    named_lib.remove("A");
    EXPECT_THROW(named_lib.get("A"), std::out_of_range);
    EXPECT_THROW(named_lib.get(id_a), std::out_of_range);
    
    EXPECT_EQ(named_lib.get("B"), 200);

    // Remove by ID
    named_lib.remove(id_b);
    EXPECT_THROW(named_lib.get("B"), std::out_of_range);
}

// --- 测试 NamedLibrary 索引一致性 (删除中间元素) ---
TEST(NamedLibraryTest, IndexStability) {
    NamedLibrary<double, std::string> lib;
    
    int id1 = lib.add_item("First", "1");
    int id2 = lib.add_item("Second", "2");
    int id3 = lib.add_item("Third", "3");

    // Remove middle element
    lib.remove("Second");

    // "Third" should still be accessible by its original ID and Name
    EXPECT_EQ(lib.get("Third"), "3");
    EXPECT_EQ(lib.get(id3), "3"); 
    
    // "First" should still be accessible
    EXPECT_EQ(lib.get("First"), "1");
    EXPECT_EQ(lib.get(id1), "1");
}
