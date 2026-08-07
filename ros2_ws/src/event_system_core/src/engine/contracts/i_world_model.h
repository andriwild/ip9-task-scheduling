#pragma once

#include <optional>
#include <string>
#include <vector>

#include "util/types.h"

namespace des {

class IWorldModel {
public:
    virtual ~IWorldModel() = default;
    virtual const Room& room(const std::string& name) const = 0;
    virtual std::vector<std::string> roomNames() const = 0;
    virtual std::optional<int> lastServiced(const std::string& room, const std::string& type) const = 0;
    virtual void recordServiced(const std::string& room, const std::string& type, int time) = 0;
};

}  // namespace des
