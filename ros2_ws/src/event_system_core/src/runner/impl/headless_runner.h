#pragma once

#include <memory>
#include "../runner.h"
#include "../../observer/metrics.h"

class MetricsNode;

class HeadlessRunner final : public IAppRunner {

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

    void shutdown() override {
        if (m_executor) {
            m_executor->cancel(); // stop spinning
        }
    }

private:
    SortedEventQueue setupQueue(
        const std::shared_ptr<des::SimConfig> &config,
        std::vector<std::shared_ptr<des::Appointment> > &appointments,
        Scheduler &scheduler
    ) const;
};
