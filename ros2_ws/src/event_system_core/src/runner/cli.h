/*
 * Argument handling shared by the frontends.
 * ros2 launch drops empty-string arguments, so a flag may be followed
 * by the next flag rather than by its value.
 *
 */

#pragma once

#include <memory>
#include <string>

#include "init/config_loader.h"
#include "plugins/accompany/accompany_plugin.h"
#include "plugins/charge/charge_plugin.h"
#include "plugins/clean/clean_plugin.h"
#include "plugins/data_acquisition/data_acquisition_plugin.h"
#include "plugins/information/information_plugin.h"
#include "plugins/order_registry.h"
#include "runner/runner.h"
#include "util/log.h"
#include "util/stop.h"
#include "runner/sim_loop.h"

namespace des::cli {

inline void registerPlugins() {
    OrderRegistry::instance().registerPlugin(std::make_unique<AccompanyOrderPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<DataAcquisition>());
    OrderRegistry::instance().registerPlugin(std::make_unique<CleanPlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<ChargePlugin>());
    OrderRegistry::instance().registerPlugin(std::make_unique<InformationPlugin>());
}

// Returns the value of --mode, everything else lands where it belongs.
inline std::string applyArgs(const int argc, char* argv[]) {
    log::applyArgs(argc, argv);
    stop::installHandlers();
    registerPlugins();

    auto valueOf = [&](const int i) -> std::string {
        if (i + 1 >= argc) {
            return "";
        }
        const std::string next = argv[i + 1];
        return next.rfind("--", 0) == 0 ? "" : next;
    };

    std::string mode = "full";
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
        if (flag == "--rounds" && !valueOf(i).empty()) {
            ConfigLoader::s_rounds = std::stoi(valueOf(i));
        }
    }
    return mode;
}

inline int setupAndRun(IAppRunner& app) {
    try {
        app.setupApplication();
    } catch (const std::exception& e) {
        DES_LOG_ERROR("des.main", "Setup failed: %s", e.what());
        return EXIT_FAILURE;
    }
    return runSimulation(app);
}

}  // namespace des::cli
