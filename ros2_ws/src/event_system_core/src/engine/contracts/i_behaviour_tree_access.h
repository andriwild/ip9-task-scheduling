#pragma once

#include <string>

namespace des {

class IBehaviorTreeAccess {
public:
    virtual ~IBehaviorTreeAccess() = default;
    virtual void tickBT() = 0;
    virtual void setBTBlackboard(const std::string& key, const std::string& value) = 0;
};

}  // namespace des
