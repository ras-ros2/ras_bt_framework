/*
 * 
 * Copyright (C) 2024 Harsh Davda
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * For inquiries or further information, you may contact:
 * Harsh Davda
 * Email: info@opensciencestack.org
*/

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "ras_bt_framework/BtExecutor.hpp"
#include <type_traits>

#define REGISTER_NODE_TYPE(factory,namespace,nodetype) { factory->registerNodeType<namespace::nodetype>(#nodetype); }

#include "ras_bt_framework/primitives/say_something.hpp"
#include "ras_bt_framework/primitives/think_something_to_say.hpp"
#include "ras_bt_framework/primitives/move_to_pose.hpp"
#include "ras_bt_framework/primitives/trigger.hpp"
#include "ras_bt_framework/primitives/execute_traj.hpp"
#include "ras_bt_framework/primitives/rotate_eff.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();
    REGISTER_NODE_TYPE(factory,ras_bt_framework,SaySomething);
    REGISTER_NODE_TYPE(factory,ras_bt_framework,ThinkSomethingToSay);
    REGISTER_NODE_TYPE(factory,ras_bt_framework,MoveToPose);
    REGISTER_NODE_TYPE(factory,ras_bt_framework,Trigger);
    REGISTER_NODE_TYPE(factory,ras_bt_framework,ExecuteTrajectory);
    REGISTER_NODE_TYPE(factory,ras_bt_framework,RotateEffector);
    std::shared_ptr<ras_bt_framework::BTExecutor> bt_executor = std::make_shared<ras_bt_framework::BTExecutor>(factory);
    // factory.registerBehaviorTreeFromFile("./test.xml");
    // auto tree = factory.createTree("MainTree");
    // tree.tickWhileRunning();
    rclcpp::spin(bt_executor);
    // rclcpp::shutdown();
    return 0;
}