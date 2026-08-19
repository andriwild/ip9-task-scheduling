#pragma once

#include <memory>

#include "util/types.h"

namespace des {

struct SimConfig;

class IConfigAccess {
public:
    virtual ~IConfigAccess() = default;
    virtual std::shared_ptr<SimConfig> getConfig() const = 0;
};

}  // namespace des
