#pragma once

inline constexpr const char* CHARGE_SUBTREE_XML = R"(
  <BehaviorTree ID="BackgroundChargeRoutine">
    <Sequence>
      <IsActiveOrderType type="charge"/>
      <SubTree ID="Charge_ToFull" _autoremap="true"/>
    </Sequence>
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
