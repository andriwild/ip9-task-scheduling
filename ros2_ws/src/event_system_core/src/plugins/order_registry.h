#pragma once


#include <memory>
#include <stdexcept>
#include <unordered_map>

#include "i_order_plugin.h"

class OrderRegistry {
    std::unordered_map<std::string, std::unique_ptr<IOrderPlugin>> m_byType;
    std::vector<IOrderPlugin*> m_order;

public:
    static OrderRegistry& instance() {
        static OrderRegistry r;
        return r;
    }

    void registerPlugin(std::unique_ptr<IOrderPlugin> plugin) {
        const auto name = plugin->typeName();
        auto* raw = plugin.get();
        m_byType.emplace(name, std::move(plugin));
        m_order.push_back(raw);
    }

    IOrderPlugin& get(const std::string& typeName) const {
        auto it = m_byType.find(typeName);
        if (it == m_byType.end()) {
            throw std::runtime_error("Unknow order type: " + typeName);
        }
        return *it->second;
    }
    const std::vector<IOrderPlugin*>& all() const { return m_order; };
};
