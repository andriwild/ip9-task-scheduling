#pragma once

inline constexpr const char* INFORMATION_SUBTREE_XML = R"(
  <BehaviorTree ID="InformationRoutine">
    <ReactiveSequence>
      <IsActiveOrderType type="information"/>
      <ExecuteInformation />
    </ReactiveSequence>
  </BehaviorTree>
)";
