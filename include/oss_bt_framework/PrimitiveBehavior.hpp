#pragma once
#include "behaviortree_cpp/action_node.h"

namespace oss_bt_framework
{
    class PrimitiveBehavior : public BT::SyncActionNode
    {
    public:
        inline PrimitiveBehavior(const std::string& name, const BT::NodeConfig& config) :
         BT::SyncActionNode(name, config) {}
        virtual BT::NodeStatus tick() = 0 ;
    };
}