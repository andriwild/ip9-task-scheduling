#pragma once

#include <string>

namespace des {

class ITimelineSink {
public:
    virtual ~ITimelineSink() = default;

    virtual void publishMeeting(int id,
                                int startTime,
                                int scheduledTime,
                                int state,
                                const std::string& orderType,
                                const std::string& personName,
                                const std::string& roomName,
                                const std::string& description,
                                int executionMode) = 0;
};

}  // namespace des
