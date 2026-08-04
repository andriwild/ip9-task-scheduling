#include <algorithm>
#include <cstdlib>
#include <string>
#include <memory>

#include "plugins/order_registry.h"
#include "plugins/accompany/accompany_plugin.h"
#include "plugins/data_acquisition/data_acquisition_plugin.h"
#include "plugins/clean/clean_plugin.h"
#include "plugins/charge/charge_plugin.h"
#include "plugins/information/information_plugin.h"
#include "runner/runner.h"
#include "runner/impl/headless_runner.h"
#include "runner/impl/sim_runner.h"
#include "runner/impl/snapshot_builder.h"
#include "util/log.h"

int main(const int argc, char *argv[]) {
    des::log::setConsoleDefaults();

    std::unique_ptr<IAppRunner> app;
    
    // load plugins
    OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<DataAcquisition>());
    OrderRegistry::instance().registerPlugin(std::make_unique<CleanPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<ChargePlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<InformationPlugin>());

    // ros parameter requires searching the flag (stability). ros2 launch drops
    // empty-string arguments, so a flag may be followed by the next flag.
    std::string mode = "full";
    auto valueOf = [&](const int i) -> std::string {
        if (i + 1 >= argc) {
            return "";
        }
        const std::string next = argv[i + 1];
        return next.rfind("--", 0) == 0 ? "" : next;
    };

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--mode" && !valueOf(i).empty()) {
            mode = valueOf(i);
        }
        if (flag == "--config") {
            ConfigLoader::s_overridePath = valueOf(i);
        }
        if (flag == "--base-config") {
            ConfigLoader::s_baseConfigPath = valueOf(i);
        }
        if (flag == "--out-dir") {
            IAppRunner::s_outDir = valueOf(i);
        }
        if (flag == "--run-id") {
            IAppRunner::s_runId = valueOf(i);
        }
    }

    // Offline building-snapshot generation: build the JSON from DB + Nav2, exit.
    if (mode == "build") {
        rclcpp::init(argc, argv);
        DES_LOG_DEBUG(rclcpp::get_logger("des.main"), "\n----- Building Snapshot Generation -----");
        SnapshotBuilder builder;
        const int rc = builder.run();
        rclcpp::shutdown();
        return rc;
    }

    const bool headless = (mode == "headless");

    // Initialize application runner
    if (headless) {
        app = HeadlessRunner::create(argc, argv);
    } else {
        app = SimRunner::create(argc, argv);
    }
    try {
        app->setupApplication();
    } catch (const std::exception& e) {
        DES_LOG_ERROR(rclcpp::get_logger("des.main"), "Setup failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }


    // main loop of the application
    // System State (RUN, PAUSE, RESET, EXIT) are used for the UI (Rviz)
    bool running = true;
    bool batteryDepleted = false;
    auto sim_loop = [&] {
        DES_LOG_DEBUG(rclcpp::get_logger("des.main"), "Start Simulation Loop (Headless Mode: %d)", headless);
        int lastEventTime = -1;
        while (running && rclcpp::ok()) {
            app->updateConfig();

            const int state = app->loadAppState();
            switch (state) {
                case SystemState::Request::RESET:
                    app->reset();
                    app->enterPause();
                    lastEventTime = -1;
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

                    if (e->cancelled) {
                        break;
                    }

                    const double speedFactor = app->m_ctx->getConfig()->simSpeedFactor;
                    if (!headless && speedFactor > 0.0 && lastEventTime >= 0 && e->time > lastEventTime) {
                        const double waitMs = std::min(10000.0, (e->time - lastEventTime) * 1000.0 / speedFactor);
                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(waitMs)));
                    }
                    lastEventTime = e->time;

                    app->m_ctx->advanceTime(e->time);
                    app->m_ctx->executeEvent(e);
                    DES_LOG_DEBUG(rclcpp::get_logger("des.main"), "-> Event Execute: %s %s", e->getName().c_str(), des::toHumanReadableTime(e->time).c_str());

                    // stop if the battery is depleted -> simulation failed
                    if (app->m_ctx->getRobot()->isBatteryDepleted()) {
                        DES_LOG_ERROR(rclcpp::get_logger("des.main"), "Robot battery reached SoC 0 at %s — aborting simulation", des::toHumanReadableTime(e->time).c_str());
                        batteryDepleted = true;
                        running = false;
                        if (const auto metrics = app->metricsNode()) {
                            metrics->setTerminatedReason("battery_depleted");
                            metrics->publishReport();
                        }
                        break;
                    }
                    if (e->getType() == des::EventType::SIMULATION_END) {
                        app->m_eventQueue.clear();
                        handleSimComplete();
                    }
                    break;
                }
                default:
                        DES_LOG_ERROR(rclcpp::get_logger("des.main"), "Unrecognized Simulation State!");
                    break;
            }
        }
        DES_LOG_DEBUG(rclcpp::get_logger("des.main"), "Simulation complete");
    };

    std::thread t(sim_loop); 
    t.join();

    // Stop ROS Executor and ROS-Thread
    app->shutdown();
    app.reset();

    return batteryDepleted ? EXIT_FAILURE : 0;
}
