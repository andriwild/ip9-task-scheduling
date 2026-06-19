#pragma once

inline constexpr const char* CLEAN_SUBTREE_XML = R"(
  <BehaviorTree ID="CleanRoutine">
    <ReactiveSequence>
      <IsActiveOrderType type="clean"/>
      <SubTree ID="Clean_Sweep" _autoremap="true"/>
    </ReactiveSequence>
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
