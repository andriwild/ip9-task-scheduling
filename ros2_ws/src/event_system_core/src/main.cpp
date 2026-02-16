#include "init/des_application.h"

int main(const int argc, char *argv[]) {
    DesApplication app(argc, argv);
    app.setupApplication();

    auto _ = std::thread([&] {
        RCLCPP_INFO(app.m_node->get_logger(), "Start Simulation Thread");
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
                        RCLCPP_DEBUG(app.m_node->get_logger(), "Queue Event: %s %s", e->getName().c_str(), des::toHumanReadableTime(e->time).c_str());
                        app.m_ctx->setTime(e->time);
                        e->execute(*app.m_ctx);
                    } else {
                        RCLCPP_DEBUG(app.m_node->get_logger(), "Simulation complete. Event Queue empty.");
                        app.enterPause();
                    }
                    break;
                default:
                        RCLCPP_WARN(app.m_node->get_logger(), "Unrecognized Simulation State!");
                    break;
            }
        }

        RCLCPP_INFO(app.m_node->get_logger(), "Simulation complete");

        QCoreApplication::quit();
        QApplication::quit();
    });

    return QApplication::exec();
}
