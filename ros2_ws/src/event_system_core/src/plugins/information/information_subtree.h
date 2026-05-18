#pragma once

inline constexpr const char* INFORMATION_SUBTREE_XML = R"(
  <BehaviorTree ID="InformationRoutine">
    <Sequence>
      <IsActiveOrderType type="information"/>
      <ExecuteInformation />
    </Sequence>
  </BehaviorTree>
)";
