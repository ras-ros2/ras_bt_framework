#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "oss_bt_framework/PrimitiveBehavior.hpp"
#include "oss_bt_framework/BtExecutor.hpp"
#include <type_traits>

#define REGISTER_NODE_TYPE(factory,namespace,nodetype) { factory->registerNodeType<namespace::nodetype>(#nodetype); }

#include "oss_bt_framework/primitives/say_something.hpp"
#include "oss_bt_framework/primitives/think_something_to_say.hpp"
#include "oss_bt_framework/primitives/move_to_pose.hpp"
#include "oss_bt_framework/primitives/trigger.hpp"
#include "oss_bt_framework/primitives/execute_traj.hpp"
#include "oss_bt_framework/primitives/rotate_eff.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();
    REGISTER_NODE_TYPE(factory,oss_bt_framework,SaySomething);
    REGISTER_NODE_TYPE(factory,oss_bt_framework,ThinkSomethingToSay);
    REGISTER_NODE_TYPE(factory,oss_bt_framework,MoveToPose);
    REGISTER_NODE_TYPE(factory,oss_bt_framework,Trigger);
    REGISTER_NODE_TYPE(factory,oss_bt_framework,ExecuteTrajectory);
    REGISTER_NODE_TYPE(factory,oss_bt_framework,RotateEffector);
    std::shared_ptr<oss_bt_framework::BTExecutor> bt_executor = std::make_shared<oss_bt_framework::BTExecutor>(factory);
    // factory.registerBehaviorTreeFromFile("./test.xml");
    // auto tree = factory.createTree("MainTree");
    // tree.tickWhileRunning();
    rclcpp::spin(bt_executor);
    // rclcpp::shutdown();
    return 0;
}