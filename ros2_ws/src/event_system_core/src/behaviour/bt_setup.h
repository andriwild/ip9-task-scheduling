#pragma once

#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include "../model/i_sim_context.h"
#include "idle.h"
#include "charge.h"
#include "../plugins/order_registry.h"
#include "mission_control.h"


constexpr bool W_OUT_TREE   = true;
const std::string TREE_FILE = "bt_config.xml";

const std::string MISSION_CONTROL_ROUTINE = R"(
  <BehaviorTree ID="MissionControlRoutine">
    <Sequence name="Seq_HandleMissions">
      <IsIdle/>
      <Sequence>
        <HasPendingMission/>
        <Fallback name="AcceptOrDeclineMission">
          <Sequence name="AcceptMissionSequence">
            <MissionFeasibilityCheck/>
            <AcceptMissionAction/>
          </Sequence>
          <RejectMissionAction/>
        </Fallback>
      </Sequence>
    </Sequence>
  </BehaviorTree>
)";


const std::string CHARGE_ROUTINE = R"(
<BehaviorTree ID="ChargeRoutine">
  <Fallback name="ChargeLogicTopLevel">

    <Sequence name="ContinueRegularWork">
      <Inverter><IsBatteryLow /></Inverter>
      <Inverter><IsCharging /></Inverter>
    </Sequence>

    <IsTaskActive />

    <ForceFailure>
      <Fallback name="Execution">
        <Sequence name="DockingSequence">
          <GoToDock />
          <Charge />
        </Sequence>
      </Fallback>
    </ForceFailure>

  </Fallback>
</BehaviorTree>
)";

const std::string IDLE_ROUTINE = R"(
<BehaviorTree ID="IdleRoutine">
    <Sequence name="Seq_IdleMain">
        <IsIdle/>
        <Inverter>
            <HasPendingMission/>
        </Inverter>
        <Inverter>
            <IsRobotBusy/>
        </Inverter>
        <Docking/>
        <EnterIdle/>
    </Sequence>
</BehaviorTree>
)";


inline void registerCoreNodes(BT::BehaviorTreeFactory& factory) {
    // charge
    factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<IsCharging>("IsCharging");
    factory.registerNodeType<IsTaskActive>("IsTaskActive");
    factory.registerNodeType<Charge>("Charge");
    factory.registerNodeType<GoToDock>("GoToDock");

    // idle
    factory.registerNodeType<IsIdle>("IsIdle");
    factory.registerNodeType<Docking>("Docking");
    factory.registerNodeType<EnterIdle>("EnterIdle");
    factory.registerNodeType<HasPendingMissionIdle>("HasPendingMissionIdle");

    // mission control
    factory.registerNodeType<HasPendingMission>("HasPendingMission");
    factory.registerNodeType<IsRobotBusy>("IsRobotBusy");
    factory.registerNodeType<AcceptMissionAction>("AcceptMissionAction");
    factory.registerNodeType<RejectMissionAction>("RejectMissionAction");
    factory.registerNodeType<MissionFeasibilityCheck>("MissionFeasibilityCheck");
}


                    // <SubTree ID="SearchRoutine" _autoremap="true"/>
                    // <SubTree ID="ConversateRoutine" _autoremap="true"/>
                    // <SubTree ID="AccompanyRoutine" _autoremap="true"/>

inline std::string buildXml() {
    std::string xml = R"(
     <root BTCPP_format="4" main_tree_to_execute="MainTree">
         <BehaviorTree ID="MainTree">
            <Sequence name="Seq_MainLoop">
                <SubTree ID="ChargeRoutine" _autoremap="true"/>
                <Fallback name="Fallback_MainStrategy">
                    <SubTree ID="MissionControlRoutine" _autoremap="true"/>)";

    for (auto* plugin : OrderRegistry::instance().all()) {
        for (const auto& subtreeId : plugin->subtreeIds()) {
            xml += "        <SubTree ID=\"" + subtreeId + "\" _autoremap=\"true\"/>\n";
        }
    }
    
    xml += R"(       <SubTree ID="IdleRoutine" _autoremap="true"/>
                </Fallback>
            </Sequence>
         </BehaviorTree>)";

    xml += CHARGE_ROUTINE;
    xml += MISSION_CONTROL_ROUTINE;
    xml += IDLE_ROUTINE;

    for (auto* plugin : OrderRegistry::instance().all()){
        xml += plugin->subtreeXml();
    }

    xml += "</root>";
    return xml;
}

inline std::shared_ptr<BT::Tree> setupBehaviorTree(std::shared_ptr<ISimContext> ctx) {
    RCLCPP_DEBUG(rclcpp::get_logger("des_application"), "Create Behaviour Tree");
    BT::BehaviorTreeFactory factory;

    registerCoreNodes(factory);

    for (auto* plugin : OrderRegistry::instance().all()) {
        plugin->registeredNodes(factory);
    }

    std::string xml = buildXml();


    factory.registerBehaviorTreeFromText(xml);
    auto tree = std::make_shared<BT::Tree>(factory.createTree("MainTree"));
    tree->rootBlackboard()->set<std::shared_ptr<ISimContext>>("ctx", ctx);

    if (W_OUT_TREE) {
        std::string xml_models    = BT::writeTreeNodesModelXML(factory);
        std::string xml_full_tree = BT::WriteTreeToXML(*tree, true, true);

        std::ofstream file(TREE_FILE);
        if (file.is_open()) {
            file << xml_full_tree;
            file.close();
            std::cout << "BehaviorTree and model written to file: " << TREE_FILE << std::endl;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("des_application"), "Behaviour Tree created");
    return tree;
}
