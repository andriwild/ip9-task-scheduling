#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include "../behaviour/nodes/sim.h"
#include "../model/context.h"

inline std::shared_ptr<BT::Tree> setupBehaviorTree(SimulationContext& ctx) {
    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<IsSearching>("IsSearching");
    factory.registerNodeType<IsEscorting>("IsEscorting");
    factory.registerNodeType<HandleSearch>("HandleSearch");
    factory.registerNodeType<HandleEscort>("HandleEscort");
    factory.registerNodeType<HandleIdle>("HandleIdle");

    static const char* xml_text = R"(
     <root BTCPP_format="4">
         <BehaviorTree ID="MainTree">
            <Fallback>
                <Sequence>
                    <IsSearching/>
                    <HandleSearch/>
                </Sequence>
                <Sequence>
                    <IsEscorting/>
                    <HandleEscort/>
                </Sequence>
                <HandleIdle/>
            </Fallback>
         </BehaviorTree>
     </root>
    )";

    auto tree = std::make_shared<BT::Tree>(factory.createTreeFromText(xml_text));
    tree->rootBlackboard()->set("ctx", &ctx);
    return tree;
}
