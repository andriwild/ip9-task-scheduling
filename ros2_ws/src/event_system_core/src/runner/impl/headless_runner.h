#pragma once

#include <memory>
#include <queue>
#include <string>
#include "../runner.h"
#include "../../observer/metrics.h"

class MetricsNode;

class HeadlessRunner final : public IAppRunner {
    std::queue<std::string> m_appointmentFiles;
    bool m_batchComplete = false;

public:
    explicit HeadlessRunner() {
        m_locationMap = IAppRunner::loadPointsOfInterest({"wsr_user", "wsr_password"});

        m_plannerNode = std::make_shared<PathPlannerNode>(m_locationMap);
        m_metricsNode = std::make_shared<MetricsNode>();

        IAppRunner::initROS({ m_plannerNode, m_metricsNode });
    }

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

    void setupApplication(const std::string& path) override;
    void updateConfig() override;
    int loadAppState() const override;
    void enterPause() const override;
    void reset() override;
    void onSimulationComplete();

    void shutdown() override {
        if (m_executor) {
            m_executor->cancel();
        }
    }

private:
    bool loadNextBatch();
};
