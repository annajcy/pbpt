#pragma once

#include <map>
#include <string>
#include <stdexcept>
#include <unordered_map>

namespace pbpt::utils {
template <typename T, typename ItemType>
class Library {
private:
    std::map<int, ItemType> m_items;
    int m_next_id = 0;

public:
    int add_item(ItemType&& item) {
        int id = m_next_id++;
        m_items.emplace(id, std::move(item));
        return id;
    }

    const ItemType& get(int id) const {
        auto it = m_items.find(id);
        if (it == m_items.end())
            throw std::out_of_range("Library: Invalid item ID");
        return it->second;
    }

    ItemType& get(int id) {
        auto it = m_items.find(id);
        if (it == m_items.end())
            throw std::out_of_range("Library: Invalid item ID");
        return it->second;
    }

    void remove(int id) {
        if (m_items.erase(id) == 0)
            throw std::out_of_range("Library: Invalid item ID");
    }

    void clear() {
        m_items.clear();
        m_next_id = 0;
    }

    std::size_t size() const { return m_items.size(); }
};

template <typename T, typename ItemType>
class NamedLibrary {
private:
    std::unordered_map<std::string, int> m_name_to_id;
    std::unordered_map<int, std::string> m_id_to_name;
    Library<T, ItemType> m_library;

public:
    int add_item(const std::string& name, ItemType&& item) {
        if (m_name_to_id.contains(name)) {
            throw std::runtime_error("NamedLibrary: Duplicate item name: " + name);
        }
        int id = m_library.add_item(std::move(item));
        m_name_to_id[name] = id;
        m_id_to_name[id] = name;
        return id;
    }

    const ItemType& get(int id) const { return m_library.get(id); }

    ItemType& get(int id) { return m_library.get(id); }

    const ItemType& get(const std::string& name) const {
        auto it = m_name_to_id.find(name);
        if (it == m_name_to_id.end())
            throw std::out_of_range("NamedLibrary: Invalid item name");
        return m_library.get(it->second);
    }

    ItemType& get(const std::string& name) {
        auto it = m_name_to_id.find(name);
        if (it == m_name_to_id.end())
            throw std::out_of_range("NamedLibrary: Invalid item name");
        return m_library.get(it->second);
    }

    void remove(int id) {
        auto it = m_id_to_name.find(id);
        if (it == m_id_to_name.end())
            throw std::out_of_range("NamedLibrary: Invalid item ID");
        m_name_to_id.erase(it->second);
        m_id_to_name.erase(it);
        m_library.remove(id);
    }

    void remove(const std::string& name) {
        auto it = m_name_to_id.find(name);
        if (it == m_name_to_id.end())
            throw std::out_of_range("NamedLibrary: Invalid item name");
        int id = it->second;
        m_name_to_id.erase(it);
        m_id_to_name.erase(id);
        m_library.remove(id);
    }

    const std::unordered_map<std::string, int>& name_to_id() const { return m_name_to_id; }

    const std::unordered_map<int, std::string>& id_to_name() const { return m_id_to_name; }

    void clear() {
        m_name_to_id.clear();
        m_id_to_name.clear();
        m_library = Library<T, ItemType>();
    }

    std::size_t size() const { return m_library.size(); }
};

};  // namespace pbpt::utils