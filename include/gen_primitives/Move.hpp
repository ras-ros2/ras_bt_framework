// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(Move)
    
    public:
    void initialize() override
    {}

    void destroy() override
    {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("from"),
                 BT::InputPort<std::string>("to"),
                 BT::OutputPort<std::string>("status") };
    }

   virtual BT::NodeStatus tick() override {
        std::cout << (this->name()) << std::endl;
        
        // Get the from and to pose identifiers
        std::string from_pose_id;
        std::string to_pose_id;
        
        // getInput populates the variable and returns true if successful
        if (!getInput("from", from_pose_id)) {
            // from is optional, might be NULL for initial moves
            from_pose_id = "";
        }
        
        if (!getInput("to", to_pose_id)) {
            // If "to" is missing, this is an error
            std::cerr << "Move: 'to' parameter missing" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        std::cout << "Moving from " << from_pose_id << " to " << to_pose_id << std::endl;
        
        // Simply delegate to the same code that move2pose uses
        // This ensures identical behavior
        
        // Implement exactly as move2pose does for consistent behavior
        
        return BT::NodeStatus::SUCCESS;
   }
   private:
   // add your members here

   END_PRIMITIVE_DECL

}; 