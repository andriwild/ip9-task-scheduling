/*
 * The discrete event loop: pop the next event, advance the clock, execute it.
 * The surrounding state machine only decides whether to run, pause, reset or quit.
 *
 */

#pragma once

#include <chrono>
#include <cstdlib>
#include <thread>

#include "runner/runner.h"
#include "util/log.h"
#include "util/stop.h"

namespace des {

// Returns false when the run has to be aborted because the battery ran dry.
inline bool executeNextEvent(IAppRunner& app) {
    if (app.m_eventQueue.empty()) {
        app.onSimulationComplete();
        return true;
    }

    const auto event = app.m_eventQueue.top();
    app.m_eventQueue.pop();
    if (event->cancelled || event->isStale(*app.m_ctx)) {
        return true;
    }

    app.m_ctx->advanceTime(event->time);
    app.m_ctx->executeEvent(event);
    app.m_protocol.push_back(event);
    DES_LOG_DEBUG("des.main", "-> Event Execute: %s %s", event->getName().c_str(), toHumanReadableTime(event->time).c_str());

    if (app.m_ctx->getRobot()->isBatteryDepleted()) {
        DES_LOG_ERROR("des.main", "Robot battery reached SoC 0 at %s — aborting simulation", toHumanReadableTime(event->time).c_str());
        app.m_ctx->getRobot()->closeStateLog(event->time);
        app.reporter().report(*app.m_ctx, app.m_protocol);
        return false;
    }

    if (event->getType() == EventType::SIMULATION_END) {
        app.m_eventQueue.clear();
        app.reporter().report(*app.m_ctx, app.m_protocol);
        app.onSimulationComplete();
    }
    return true;
}

inline int runSimulation(IAppRunner& app) {
    bool running = true;
    bool batteryHealthy= false;

    while (running && !stop::requested()) {
        app.updateConfig();
        switch (app.loadAppState()) {
            case RunState::Reset:
                app.reset();
                app.enterPause();
                break;
            case RunState::Pause:
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
            case RunState::Exit:
                running = false;
                break;
            case RunState::Run:
                batteryHealthy = executeNextEvent(app);
                if (!batteryHealthy) {
                    running = false;
                }
                break;
        }
    }

    DES_LOG_DEBUG("des.main", "Simulation complete");
    app.shutdown();
    return batteryHealthy ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace des
