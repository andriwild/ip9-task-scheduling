#pragma once

#include <fstream>
#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include "../model/context.h"
#include "accompany.h"
#include "idle.h"
#include "search.h"
#include "conversation.h"
#include "charge.h"
#include "mission_control.h"

constexpr bool W_OUT_TREE = true;
const std::string TREE_FILE = "bt_config.xml";

inline std::shared_ptr<BT::Tree> setupBehaviorTree(std::shared_ptr<SimulationContext> ctx) {
    BT::BehaviorTreeFactory factory;

    // charge
    factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<Charge>("Charge");

    // search
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<ScanLocation>("ScanLocation");
    factory.registerNodeType<HasNextLocation>("HasNextLocation");
    factory.registerNodeType<MoveToNextLocation>("MoveToNextLocation");
    factory.registerNodeType<StartAccompanyConversation>("StartAccompanyConversation");
    factory.registerNodeType<AbortSearch>("AbortSearch");

    // accompany
    factory.registerNodeType<IsAccompany>("IsAccompany");
    factory.registerNodeType<ArrivedWithPerson>("ArrivedWithPerson");
    factory.registerNodeType<StartDropOffConversation>("StartDropOffConversation");
    factory.registerNodeType<AbortAccompany>("AbortAccompany");

    // idle
    factory.registerNodeType<IsIdle>("IsIdle");
    factory.registerNodeType<Docking>("Docking");
    factory.registerNodeType<EnterIdle>("EnterIdle");

    // conversation
    factory.registerNodeType<IsConversating>("IsConversating");
    factory.registerNodeType<ConversationFinished>("ConversationFinished");
    factory.registerNodeType<WasConversationSuccessful>("WasConversationSuccessful");
    factory.registerNodeType<IsFoundPersonConversation>("IsFoundPersonConversation");
    factory.registerNodeType<IsDropOffConversation>("IsDropOffConversation");
    factory.registerNodeType<StartAccompanyAction>("StartAccompanyAction");
    factory.registerNodeType<CompleteMissionAction>("CompleteMissionAction");

    // mission control
    factory.registerNodeType<HasPendingMission>("HasPendingMission");
    factory.registerNodeType<IsRobotBusy>("IsRobotBusy");
    factory.registerNodeType<AcceptMissionAction>("AcceptMissionAction");
    factory.registerNodeType<RejectMissionAction>("RejectMissionAction");
    factory.registerNodeType<MissionFeasibilityCheck>("MissionFeasibilityCheck");

    static const char * xml_text = R"(
     <root BTCPP_format="4" main_tree_to_execute="MainTree">
         <BehaviorTree ID="MainTree">
            <Sequence name="Seq_MainLoop">

                <SubTree ID="ChargeRoutine" _autoremap="true"/>
                <SubTree ID="MissionControlRoutine" _autoremap="true"/>

                <Fallback name="Fallback_MainStrategy">
                    <SubTree ID="SearchRoutine" _autoremap="true"/>
                    <SubTree ID="ConversateRoutine" _autoremap="true"/>
                    <SubTree ID="AccompanyRoutine" _autoremap="true"/>
                    <SubTree ID="IdleRoutine" _autoremap="true"/>
                </Fallback>
            </Sequence>
         </BehaviorTree>


        <BehaviorTree ID="ChargeRoutine">
                 <Sequence name="Seq_DockAndCharge">
                    <IsBatteryLow/>
                    <Docking/>
                    <Charge/>
                </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="MissionControlRoutine">
             <Fallback>
                 <Sequence name="Seq_HandleMissions">
                    <IsIdle/>
                    <HasPendingMission/>
                    <Fallback name="Fallback_MissionDecision">
                        <Sequence name="Seq_TryAccept">
                            <MissionFeasibilityCheck/>
                            <AcceptMissionAction/>
                        </Sequence>
                         <RejectMissionAction/> 
                    </Fallback>
                </Sequence>
                <AlwaysSuccess/>
             </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="SearchRoutine">
            <Sequence name="Seq_SearchMain">
                <IsSearching/>
                <Fallback name="Fallback_SearchActions">
                    <Sequence name="Seq_FoundTarget">
                        <ScanLocation/>
                        <StartAccompanyConversation/>
                    </Sequence>
                    <Sequence name="Seq_NextLocation">
                        <HasNextLocation/>
                        <MoveToNextLocation/>
                    </Sequence>
                    <AbortSearch/>
                </Fallback>
            </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="ConversateRoutine">
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

        <BehaviorTree ID="AccompanyRoutine">
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

        <BehaviorTree ID="IdleRoutine">
            <Sequence name="Seq_IdleMain">
                <IsIdle/>
                <Inverter>
                    <HasPendingMission/>
                </Inverter>
                <Docking/>
                <EnterIdle/>
            </Sequence>
        </BehaviorTree>
     </root>
    )";

    auto tree = std::make_shared<BT::Tree>(factory.createTreeFromText(xml_text));
    tree->rootBlackboard()->set("ctx", ctx);

    if (W_OUT_TREE) {
        std::string xml_models = BT::writeTreeNodesModelXML(factory);
        std::string xml_full_tree = BT::WriteTreeToXML(*tree, true, true);

        std::ofstream file(TREE_FILE);
        if (file.is_open()) {
            file << xml_full_tree; 
            file.close();
            std::cout << "BehaviorTree and model written to file: " << TREE_FILE << std::endl;
        }
    }
    return tree;
}