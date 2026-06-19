#pragma once

inline constexpr const char* CHARGE_SUBTREE_XML = R"(
  <BehaviorTree ID="BackgroundChargeRoutine">
    <ReactiveSequence>
      <IsActiveOrderType type="charge"/>
      <SubTree ID="Charge_ToFull" _autoremap="true"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="Charge_ToFull">
    <Sequence>
      <Fallback>
        <ChargeMissionIsAtDock />
        <ChargeMissionGoToDock />
      </Fallback>
      <ExecuteChargeMission />
    </Sequence>
  </BehaviorTree>
)";
