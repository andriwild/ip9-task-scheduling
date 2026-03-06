#include <string>
#include <memory>
#include "runner/runner.h"
#include "runner/impl/headless_runner.h"
#include "runner/impl/sim_runner.h"

int main(const int argc, char *argv[]) {
    std::unique_ptr<IAppRunner> app;

    // ros parameter requires searching the flag (stability)
    bool headless = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--mode") {
            if (i + 1 < argc && std::string(argv[i+1]) == "headless") {
                headless = true;
            }
        }
    }

    if (headless) {
        app = HeadlessRunner::create(argc, argv);
    } else {
        app = SimRunner::create(argc, argv);
    }

    app->setupApplication();

    bool running = true;
    auto sim_loop = [&] {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Start Simulation Loop (Headless Mode: %d)", headless);
        while (running && rclcpp::ok()) {
            app->updateConfig();

            switch (app->loadAppState()) {
                case SystemState::Request::RESET:
                    app->reset();
                    app->enterPause();
                    break;

                case SystemState::Request::PAUSE:
                    if (!headless) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    break;

                case SystemState::Request::EXIT:
                    running = false;
                    break;

                case SystemState::Request::RUN:
                    if (!app->m_eventQueue.empty()) {
                        const auto e = app->m_eventQueue.top();
                        app->m_eventQueue.pop();
                        RCLCPP_INFO(rclcpp::get_logger("main"), "-> Event Execute: %s %s", e->getName().c_str(), des::toHumanReadableTime(e->time).c_str());

                        app->m_ctx->advanceTime(e->time);
                        e->execute(*app->m_ctx);
                    } else {
                        RCLCPP_DEBUG(rclcpp::get_logger("main"), "Simulation complete. Event Queue empty.");
                        app->enterPause();
                        if (headless) {
                            RCLCPP_INFO(rclcpp::get_logger("main"), "Fast simulation complete. Quitting...");
                            running = false;
                        }
                    }
                    break;
                default:
                        RCLCPP_WARN(rclcpp::get_logger("main"), "Unrecognized Simulation State!");
                    break;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("main"), "Simulation complete");
    };

    auto _ = std::thread(sim_loop);
    _.join();

    // Stop ROS Executor and ROS-Thread
    app->shutdown();
    app.reset();

    return 0;
}