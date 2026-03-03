#include "init/des_application.h"

int main(const int argc, char *argv[]) {
    DesApplication app(argc, argv);
    app.setupApplication();

    auto _ = std::thread([&] {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Start Simulation Thread");
        while (rclcpp::ok()) {
            app.updateConfig();

            switch (app.loadAppState()) {
                case SystemState::Request::RESET:
                    app.reset();
                    app.enterPause();
                    break;

                case SystemState::Request::PAUSE:
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break;

                case SystemState::Request::RUN:
                    if (!app.m_eventQueue.empty()) {
                        const auto e = app.m_eventQueue.top();
                        app.m_eventQueue.pop();
                        RCLCPP_INFO(rclcpp::get_logger("main"), "-> Event Execute: %s %s", e->getName().c_str(), des::toHumanReadableTime(e->time).c_str());
                        app.m_ctx->advanceTime(e->time);
                        e->execute(*app.m_ctx);
                    } else {
                        RCLCPP_DEBUG(rclcpp::get_logger("main"), "Simulation complete. Event Queue empty.");
                        app.enterPause();
                    }
                    break;
                default:
                        RCLCPP_WARN(rclcpp::get_logger("main"), "Unrecognized Simulation State!");
                    break;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("main"), "Simulation complete");
    });
}