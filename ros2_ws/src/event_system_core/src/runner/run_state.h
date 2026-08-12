/*
 * What the simulation loop should do.
 * Headless drives this itself, the interactive runner maps the SetSystemState service onto it.
 *
 */

#pragma once

namespace des {

enum class RunState {
    Run,
    Pause,
    Reset,
    Exit,
};

}  // namespace des
