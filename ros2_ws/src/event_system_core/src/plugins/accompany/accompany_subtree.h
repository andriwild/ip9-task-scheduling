inline constexpr const char* ACCOMPANY_SUBTREE_XML = R"(
  <BehaviorTree ID="AccompanyRoutine">
    <ReactiveSequence>
      <IsActiveOrderType type="accompany"/>
      <Fallback>
        <SubTree ID="AccompanyConversate" _autoremap="true"/>
        <SubTree ID="AccompanySearch"     _autoremap="true"/>
        <SubTree ID="AccompanyAccompany"  _autoremap="true"/>
      </Fallback>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanySearch">
    <Sequence name="Seq_SearchMain">
      <IsSearching />
      <Fallback name="Fallback_SearchActions">
        <Sequence name="Seq_FoundTarget">
          <FoundPerson />
          <StartAccompanyConversation />
        </Sequence>
        <Sequence name="Seq_AskBystander">
          <HasPendingAsk />
          <StartAskConversation />
        </Sequence>
        <Sequence name="Seq_ScanRoom">
          <HasScanPoint />
          <ScanNextPoint />
        </Sequence>
        <Sequence name="Seq_NextRoom">
          <HasNextLocation />
          <MoveToNextLocation />
        </Sequence>
        <Sequence name="Seq_GiveUp">
          <ReportSearchAbort />
          <FailMission />
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanyConversate">
    <Sequence name="Seq_ConversateMain">
      <IsConversating/>
      <ConversationFinished/>
      <Fallback name="Fallback_ConverseType">
        <Sequence name="Seq_AskType">
          <IsConversationKind kind="ask_directions"/>
          <Fallback name="Fallback_AskResult">
            <Sequence name="Seq_AskSuccess">
              <WasConversationSuccessful/>
              <ApplyDirections/>
            </Sequence>
            <AlwaysSuccess/>
          </Fallback>
          <ResumeSearchAfterAsk/>
          <AlwaysFailure/>
        </Sequence>
        <Sequence name="Seq_FoundPersonType">
          <IsConversationKind kind="found_person"/>
          <Fallback name="Fallback_FoundPersonResult">
            <Sequence name="Seq_FoundPersonSuccess">
              <WasConversationSuccessful/>
              <StartAccompanyAction/>
            </Sequence>
            <Sequence name="Seq_GiveUp">
              <ReportSearchAbort/>
              <FailMission/>
            </Sequence>
          </Fallback>
        </Sequence>
        <Sequence name="Seq_DropOffType">
          <IsConversationKind kind="drop_off"/>
          <Fallback name="Fallback_DropOffResult">
            <Sequence name="Seq_DropOffSuccess">
              <WasConversationSuccessful/>
              <CompleteMission/>
            </Sequence>
            <FailMission/>
          </Fallback>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanyAccompany">
    <Sequence name="Seq_AccompanyMain">
      <IsAccompany/>
      <HasArrived/>
      <Fallback name="Fallback_AccompanyActions">
        <Sequence name="Seq_ArrivalAndDropOff">
          <ArrivedWithPerson/>
          <StartDropOffConversation/>
        </Sequence>
        <FailMission/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
)";
