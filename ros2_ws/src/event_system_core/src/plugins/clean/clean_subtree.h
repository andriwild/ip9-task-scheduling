#pragma once

inline constexpr const char* CLEAN_SUBTREE_XML = R"(
  <BehaviorTree ID="CleanRoutine">
    <Sequence>
      <IsActiveOrderType type="clean"/>
      <SubTree ID="Clean_Sweep" _autoremap="true"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="Clean_Sweep">
    <Sequence>
      <Fallback>
        <CleanIsAtTargetLocation />
        <CleanGoToLocation />
      </Fallback>
      <ExecuteClean />
    </Sequence>
  </BehaviorTree>
)";
