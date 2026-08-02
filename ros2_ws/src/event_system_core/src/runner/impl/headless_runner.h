#pragma once

#include <memory>
#include "../../util/log.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <queue>
#include <string>
#include <vector>
#include "../runner.h"
#include "../../observer/metrics.h"

class MetricsNode;

class HeadlessRunner final : public IAppRunner {
    std::queue<std::string> m_orderFiles;
    int m_totalRounds = 1;
    int m_currentRound = 0;
    bool m_batchComplete = false;

    void rebuildFileQueue() {
        m_orderFiles = {};
        m_orderFiles.push(m_config->appointmentsPath);
    }

    unsigned int roundSeed(const int round) const {
        return m_config->seed + des::ROUND_SEED_STRIDE * static_cast<unsigned int>(round);
    }

    void writeEffectiveConfig(const std::string& path) const {
        std::error_code ec;
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
        }
        std::ofstream out(path);
        if (!out.is_open()) {
            DES_LOG_WARN(rclcpp::get_logger("des.runner"), "Could not write effective config: %s", path.c_str());
            return;
        }
        out << std::setw(4) << ConfigLoader::configToJson(*m_config) << std::endl;
        DES_LOG_INFO(rclcpp::get_logger("des.runner"), "Effective config written: %s", path.c_str());
    }

    static std::string outputPath(const std::string& stem, const std::string& extension) {
        if (!s_outDir.empty()) {
            const std::string dir = s_outDir.back() == '/' ? s_outDir : s_outDir + "/";
            return dir + stem + extension;
        }
        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
        return CONFIG_PATH + "../results/" + stem + "_" + stamp + extension;
    }

public:
    explicit HeadlessRunner() {
        m_rooms = loadRooms();

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
        for (int i = 1; i + 1 < argc; ++i) {
            if (std::string(argv[i]) == "--rounds" && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                int rounds = std::atoi(argv[i + 1]);
                runner->m_totalRounds = std::max(1, rounds);
            }
        }
        return runner;
    }

    void setupApplication() override;
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
