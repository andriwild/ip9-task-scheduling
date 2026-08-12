#include <cstdlib>

#include "runner/cli.h"
#include "runner/impl/headless_runner.h"
#include "util/log.h"

int main(const int argc, char* argv[]) {
    des::cli::applyArgs(argc, argv);

    const auto app = des::HeadlessRunner::create(argc, argv);
    DES_LOG_DEBUG("des.main", "Start Simulation Loop (headless, no ROS)");
    return des::cli::setupAndRun(*app);
}
