#pragma once

#include <memory>
#include "../runner.h"
#include "../../observer/metrics.h"

class MetricsNode;

class HeadlessRunner : public IAppRunner {

public:
    HeadlessRunner() = default;

    ~HeadlessRunner() override {
        if (rclcpp::ok()) {
            HeadlessRunner::shutdown();
            rclcpp::shutdown();
        }

        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
    }

    static std::unique_ptr<IAppRunner> create(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "\n----- Descrete Event Sytem: Headless Mode -----");
        RCLCPP_INFO(rclcpp::get_logger("des_application"), "C++ Version: %ld", __cplusplus);
        return std::make_unique<HeadlessRunner>();
    }

    void setupApplication() override;
    void updateConfig() override;
    int loadAppState() const override;
    void enterPause() const override;
    void setupObservers(bool headless, bool verbose) override;
    void reset() override;

    void shutdown() override {
        if (m_executor) {
            m_executor->cancel(); // stop spinning
        }
    }

private:
    void initROS();
    void setupQueue(const std::shared_ptr<des::SimConfig> &config);

    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_rosThread;
};

