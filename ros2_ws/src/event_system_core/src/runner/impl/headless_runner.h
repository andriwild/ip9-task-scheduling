#pragma once

#include <memory>
#include "../../util/log.h"
#include <queue>
#include <string>
#include <vector>
#include "../runner.h"
#include "../../observer/metrics.h"

class MetricsNode;

class HeadlessRunner final : public IAppRunner {
    std::vector<std::string> m_orderFilePaths;
    std::queue<std::string> m_orderFiles;
    int m_totalRounds = 1;
    int m_currentRound = 0;
    bool m_batchComplete = false;

    void rebuildFileQueue() {
        m_orderFiles = {};
        for (const auto& path : m_orderFilePaths) {
            m_orderFiles.push(path);
        }
    }

public:
    explicit HeadlessRunner() {
        m_locationMap = loadLocations();

        createPlanner();
        m_metricsNode = std::make_shared<MetricsNode>();

        std::vector<std::shared_ptr<rclcpp::Node>> nodes = { m_metricsNode };
        if (m_plannerNode) nodes.push_back(m_plannerNode);
        IAppRunner::initROS(nodes);
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
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "\n----- Descrete Event Sytem: Headless Mode -----");
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "C++ Version: %ld", __cplusplus);

        auto runner = std::make_unique<HeadlessRunner>();
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--rounds" && i + 1 < argc) {
                int rounds = std::atoi(argv[i + 1]);
                runner->m_totalRounds = std::max(1, rounds);
            }
        }
        return runner;
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
    std::vector<std::shared_ptr<des::Person>> m_allPeople;
};
