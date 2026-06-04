#include <cstdlib>
#include <string>
#include <memory>

#include "plugins/order_registry.h"
#include "plugins/accompany/accompany_plugin.h"
#include "plugins/data_acquisition/data_acquisition_plugin.h"
#include "plugins/clean/clean_plugin.h"
#include "plugins/information/information_plugin.h"
#include "runner/runner.h"
#include "runner/impl/headless_runner.h"
#include "runner/impl/sim_runner.h"
#include "runner/impl/snapshot_builder.h"
#include "util/log.h"

const std::string APPOINTMENT_FILES = "/home/andri/repos/ip9-task-scheduling/ros2_ws/config/scenarios/test/";

int main(const int argc, char *argv[]) {
    std::unique_ptr<IAppRunner> app;
    OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<DataAcquisition>());
    OrderRegistry::instance().registerPlugin(std::make_unique<CleanPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<InformationPlugin>());

    // ros parameter requires searching the flag (stability)
    std::string mode = "full";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--mode") {
            mode = argv[i + 1];
        }
    }

    // Offline building-snapshot generation: build the JSON from DB + Nav2, exit.
    if (mode == "build") {
        rclcpp::init(argc, argv);
        des::log::installOutputHandler();
        DES_LOG_INFO(rclcpp::get_logger("des.main"), "\n----- Building Snapshot Generation -----");
        SnapshotBuilder builder;
        const int rc = builder.run();
        rclcpp::shutdown();
        return rc;
    }

    const bool headless = (mode == "headless");

    // Initialize application runner
    if (headless) {
        app = HeadlessRunner::create(argc, argv);
        app->setupApplication(APPOINTMENT_FILES);
    } else {
        app = SimRunner::create(argc, argv);
        app->setupApplication("");
    }

    des::log::installOutputHandler();  // after rclcpp::init() from create()
    

    bool running = true;
    auto sim_loop = [&] {
        DES_LOG_INFO(rclcpp::get_logger("des.main"), "Start Simulation Loop (Headless Mode: %d)", headless);
        app->m_eventQueue.print();
        while (running && rclcpp::ok()) {
            app->updateConfig();

            switch (app->loadAppState()) {
                case SystemState::Request::RESET:
                    app->reset();
                    app->enterPause();
                    break;

                case SystemState::Request::PAUSE:
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break;

                case SystemState::Request::EXIT:
                    running = false;
                    break;

                case SystemState::Request::RUN: {
                    auto handleSimComplete = [&] {
                        DES_LOG_DEBUG(rclcpp::get_logger("des.main"), "Simulation complete.");
                        if (headless) {
                            auto* headlessApp = static_cast<HeadlessRunner*>(app.get());
                            headlessApp->onSimulationComplete();
                        } else {
                            app->enterPause();
                        }
                    };
                    if (app->m_eventQueue.empty()) {
                        handleSimComplete();
                        break;
                    }
                    const auto e = app->m_eventQueue.top();
                    app->m_eventQueue.pop();
                    app->m_ctx->advanceTime(e->time);
                    app->m_ctx->executeEvent(e);
                    DES_LOG_INFO(rclcpp::get_logger("des.main"), "-> Event Execute: %s %s", e->getName().c_str(), des::toHumanReadableTime(e->time).c_str());
                    if (e->getType() == des::EventType::SIMULATION_END) {
                        app->m_eventQueue.clear();
                        handleSimComplete();
                    }
                    break;
                }
                default:
                        DES_LOG_WARN(rclcpp::get_logger("des.main"), "Unrecognized Simulation State!");
                    break;
            }
        }
        DES_LOG_INFO(rclcpp::get_logger("des.main"), "Simulation complete");
    };

    std::thread t(sim_loop); 
    t.join();

    // Stop ROS Executor and ROS-Thread
    app->shutdown();
    app.reset();

    return 0;
}
