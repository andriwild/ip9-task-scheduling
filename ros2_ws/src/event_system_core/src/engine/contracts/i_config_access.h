#pragma once

#include <memory>

#include "util/types.h"

class IConfigAccess {
public:
    virtual ~IConfigAccess() = default;
    virtual std::shared_ptr<des::SimConfig> getConfig() const = 0;
};
