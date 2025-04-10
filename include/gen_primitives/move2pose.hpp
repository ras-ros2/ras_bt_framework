// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(Move2pose)
    
    public:
    void initialize() override
    {}

    ~Move2pose() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("pose"),
                 BT::OutputPort<std::string>("status") };
    }

   virtual BT::NodeStatus tick() override {
        std::cout << (this->name()) << std::endl;
        
        // Get the pose identifier
        std::string pose_id;
        
        if (!getInput("pose", pose_id)) {
            std::cerr << "Move2pose: 'pose' parameter missing" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        std::cout << "Moving to pose: " << pose_id << std::endl;
        
        // Here, implement the actual robot movement logic
        // This might involve calling a motion planning service
        // or using the robot's control interface
        
        // Implementation should use the pose ID to look up actual coordinates
        // and plan a trajectory to move the robot
        
        // For now, we'll simulate success
        return BT::NodeStatus::SUCCESS;
   }
   private:
   // add your members here

   END_PRIMITIVE_DECL

};
