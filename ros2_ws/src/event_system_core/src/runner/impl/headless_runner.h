#pragma once

#include <memory>
#include "../../util/log.h"
#include <filesystem>
#include <string>
#include <vector>
#include "../runner.h"

namespace des {

class HeadlessRunner final : public IAppRunner {
    static constexpr int kRounds = 1;

    int m_currentRound = 0;
    bool m_runComplete = false;

    unsigned int roundSeed(const int round) const {
        return m_config->seed + ROUND_SEED_STRIDE * static_cast<unsigned int>(round);
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
    }

    ~HeadlessRunner() override = default;

    static std::unique_ptr<IAppRunner> create(int, char**) {
        DES_LOG_INFO("des.runner", "\n----- Descrete Event Sytem: Headless Mode -----");
        DES_LOG_INFO("des.runner", "C++ Version: %ld", __cplusplus);

        return std::make_unique<HeadlessRunner>();
    }

    void setupApplication() override;
    void updateConfig() override;
    RunState loadAppState() const override;
    void enterPause() const override;
    void reset() override;
    void onSimulationComplete() override;

    void shutdown() override {}

private:
    bool loadNextRound();
};

}  // namespace des
