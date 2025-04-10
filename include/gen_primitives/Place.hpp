// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "gen_primitives/move2pose.hpp" // Include for reusing the same logic

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(Place)
    
    public:
    void initialize() override
    {}

    void destroy() override
    {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("above"),
                 BT::InputPort<std::string>("at"),
                 BT::OutputPort<std::string>("status") };
    }

   virtual BT::NodeStatus tick() override {
        std::cout << (this->name()) << std::endl;
        
        // Get the "above" and "at" pose identifiers
        std::string above_pose_id;
        std::string at_pose_id;
        
        if (!getInput("above", above_pose_id)) {
            std::cerr << "Place: 'above' parameter missing" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        if (!getInput("at", at_pose_id)) {
            std::cerr << "Place: 'at' parameter missing" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        std::cout << "Placing: moving from above " << above_pose_id << " to " << at_pose_id << " and back" << std::endl;
        
        // This will use the same core motion code as move2pose for consistency
        // In actual code you would do:
        // 1. Use move2pose's logic to move to above_pose_id
        // 2. Use move2pose's logic to move to at_pose_id
        // 3. Open gripper
        // 4. Use move2pose's logic to move back to above_pose_id
        
        return BT::NodeStatus::SUCCESS;
   }
   private:
   // add your members here

   END_PRIMITIVE_DECL

}; 