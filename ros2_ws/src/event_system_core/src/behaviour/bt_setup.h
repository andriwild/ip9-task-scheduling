#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <rclcpp/rclcpp.hpp>

#include "../model/context.h"
#include "accompany.h"
#include "idle.h"
#include "search.h"

inline std::shared_ptr<BT::Tree> setupBehaviorTree(std::shared_ptr<SimulationContext> ctx) {
    BT::BehaviorTreeFactory factory;

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
    factory.registerNodeType<StartDropoffConversation>("StartDropoffConversation");
    factory.registerNodeType<AbortAccompany>("AbortAccompany");

    // idle
    factory.registerNodeType<Docking>("Docking");
    factory.registerNodeType<EnterIdle>("EnterIdle");

    static const char * xml_text = R"(
     <root BTCPP_format="4" main_tree_to_execute="MainTree">
         <BehaviorTree ID="MainTree">
            <Fallback name="Fallback_MainStrategy">
                <SubTree ID="SearchRoutine" _autoremap="true"/>
                <SubTree ID="AccompanyRoutine" _autoremap="true"/>
                <SubTree ID="IdleRoutine" _autoremap="true"/>
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

        <BehaviorTree ID="AccompanyRoutine">
            <Sequence name="Seq_AccompanyMain">
                <IsAccompany/>
                <Fallback name="Fallback_AccompanyActions">
                    <Sequence name="Seq_ArrivalAndDropoff">
                        <ArrivedWithPerson/>
                        <StartDropoffConversation/>
                    </Sequence>
                    <AbortAccompany/>
                </Fallback>
            </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="IdleRoutine">
            <Sequence name="Seq_IdleMain">
                <Docking/>
                <EnterIdle/>
            </Sequence>
        </BehaviorTree>
     </root>
    )";

    auto tree = std::make_shared<BT::Tree>(factory.createTreeFromText(xml_text));
    tree->rootBlackboard()->set("ctx", ctx);

    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
    return tree;
}
