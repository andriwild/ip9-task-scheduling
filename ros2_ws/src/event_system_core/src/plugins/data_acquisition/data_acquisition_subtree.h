inline constexpr const char* DATA_ACQUISITION_SUBTREE_XML = R"(
  <BehaviorTree ID="DataAcquisitionRoutine">
    <ReactiveSequence>
      <IsActiveOrderType type="data_acquisition"/>
      <SubTree ID="DataAcquisition_Acquire" _autoremap="true"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="DataAcquisition_Acquire">
    <Sequence>
      <Fallback>
        <IsAtTargetLocation />
        <GoToLocation />
      </Fallback>
      <ExecuteAcquisition />
    </Sequence>
  </BehaviorTree>
)";
