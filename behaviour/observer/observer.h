#include <string>

class IObserver {
public:
    virtual ~IObserver() = default;

    virtual void onLog(double time, const std::string& message) = 0;
};
