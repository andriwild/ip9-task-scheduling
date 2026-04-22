inline constexpr const char* ACCOMPANY_SUBTREE_XML = R"(
  <BehaviorTree ID="AccompanyRoutine">
    <Sequence>
      <IsActiveOrderType type="accompany"/>
      <Fallback>
        <SubTree ID="AccompanySearch"     _autoremap="true"/>
        <SubTree ID="AccompanyConversate" _autoremap="true"/>
        <SubTree ID="AccompanyAccompany"  _autoremap="true"/>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanySearch">
    <Sequence name="Seq_SearchMain">
      <IsSearching />
      <Fallback name="Fallback_SearchActions">
        <Sequence>
          <Inverter>
            <IsScanning />
          </Inverter>
          <ScanLocation />
        </Sequence>
        <Sequence name="Seq_FoundTarget">
          <FoundPerson />
          <StartAccompanyConversation />
        </Sequence>
        <Sequence name="Seq_NextLocation">
          <HasNextLocation />
          <MoveToNextLocation />
        </Sequence>
        <AbortSearch />
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanyConversate">
      <Sequence name="Seq_ConversateMain">
          <IsConversating/>
          <ConversationFinished/>
          
          <Fallback name="Fallback_ConverseType">
             <Sequence name="Seq_FoundPersonType">
                 <IsFoundPersonConversation/>
                 <Fallback name="Fallback_FoundPersonResult">
                      <Sequence name="Seq_FoundPersonSuccess">
                          <WasConversationSuccessful/>
                          <StartAccompanyAction/>
                      </Sequence>
                      <!-- If conversation failed -->
                      <AbortSearch/> 
                 </Fallback>
             </Sequence>

             <Sequence name="Seq_DropOffType">
                 <IsDropOffConversation/>
                 <CompleteMissionAction/>
             </Sequence>
          </Fallback>
      </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="AccompanyAccompany">
      <Sequence name="Seq_AccompanyMain">
          <IsAccompany/>
          <Fallback name="Fallback_AccompanyActions">
              <Sequence name="Seq_ArrivalAndDropOff">
                  <ArrivedWithPerson/>
                  <StartDropOffConversation/>
              </Sequence>
              <AbortAccompany/>
          </Fallback>
      </Sequence>
  </BehaviorTree>
)";
