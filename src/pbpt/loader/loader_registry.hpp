#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <pugixml.hpp>

namespace pbpt::loader {

template <typename T>
struct LoaderContext;

template <typename T, typename ProductType>
class LoaderRegistry {
public:
    using FactoryFunction = std::function<ProductType(const pugi::xml_node&, LoaderContext<T>&)>;

    void register_loader(const std::string& name, FactoryFunction factory) { registry_[name] = factory; }

    ProductType create(const std::string& name, const pugi::xml_node& node, LoaderContext<T>& ctx) const {
        if (registry_.find(name) != registry_.end()) {
            return registry_.at(name)(node, ctx);
        }
        throw std::runtime_error("Unknown type for loader: " + name);
    }

    bool has_loader(const std::string& name) const { return registry_.find(name) != registry_.end(); }

private:
    std::map<std::string, FactoryFunction> registry_;
};

}  // namespace pbpt::loader
