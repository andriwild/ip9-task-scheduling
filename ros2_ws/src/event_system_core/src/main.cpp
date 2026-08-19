#include <cstdlib>
#include <memory>
#include <string>

#include "runner/cli.h"
#include "runner/impl/headless_runner.h"
#include "runner/impl/sim_runner.h"
#include "io/snapshot_builder.h"
#include "util/log.h"

int main(const int argc, char* argv[]) {
    const std::string mode = des::cli::applyArgs(argc, argv);

    // Offline building-snapshot generation: build the JSON from DB + Nav2, exit.
    if (mode == "build") {
        DES_LOG_DEBUG("des.main", "\n----- Building Snapshot Generation -----");
        return des::SnapshotBuilder::build(argc, argv);
    }

    if (mode == "build_rooms") {
        DES_LOG_DEBUG("des.main", "\n----- Building Snapshot Generation (rooms only) -----");
        return des::SnapshotBuilder::buildRooms(argc, argv);
    }

    const bool headless = (mode == "headless");
    const auto app = headless ? des::HeadlessRunner::create(argc, argv)
                              : des::SimRunner::create(argc, argv);

    DES_LOG_DEBUG("des.main", "Start Simulation Loop (Headless Mode: %d)", headless);
    return des::cli::setupAndRun(*app);
}
