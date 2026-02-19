#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <pugixml.hpp>

#include "pbpt/loader/loader_context.hpp"

namespace pbpt::loader {

template <typename T, typename ProductType>
class LoaderRegistry {
public:
    using FactoryFunction = std::function<ProductType(const pugi::xml_node&, LoaderContext<T>&)>;

    static void register_loader(const std::string& name, FactoryFunction factory) { get_registry()[name] = factory; }

    static ProductType create(const std::string& name, const pugi::xml_node& node, LoaderContext<T>& ctx) {
        auto& reg = get_registry();
        if (reg.find(name) != reg.end()) {
            return reg[name](node, ctx);
        }
        throw std::runtime_error("Unknown type for loader: " + name);
    }

    static bool has_loader(const std::string& name) {
        auto& reg = get_registry();
        return reg.find(name) != reg.end();
    }

private:
    static std::map<std::string, FactoryFunction>& get_registry() {
        static std::map<std::string, FactoryFunction> registry;
        return registry;
    }
};

}  // namespace pbpt::loader
