# contracts

Every role that `SimulationContext` plays has its own interface here: clock, event sink, robot access, order board, person registry, world model and more. Without this split, each plugin, BT node and algorithm would depend on the whole engine, even when it only needs the time or the robot. A test would also have to build the full context just to check one decision.

A consumer names only the role it really uses. This way `plugins/`, `behaviour/`, `algo/` and `observer/` depend on `contracts/` and not on `engine/` itself. `ISimContext` joins all roles together and is the type the engine passes around.
