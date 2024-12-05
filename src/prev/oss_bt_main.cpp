#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <std_srvs/srv/set_bool.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "oss_interfaces/srv/traj_action.hpp"
#include "oss_interfaces/srv/status_log.hpp"
#include "oss_interfaces/srv/traj_sec.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("behavior_tree_node");

    rclcpp::Rate rate(10);

	BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

	auto blackboard = BT::Blackboard::create();
	blackboard->set("node", node);

    factory.registerFromPlugin(loader.getOSName("get_trajectory_node"));
    factory.registerFromPlugin(loader.getOSName("pick_node"));
    factory.registerFromPlugin(loader.getOSName("drop_node"));
    factory.registerFromPlugin(loader.getOSName("traj_node"));
    factory.registerFromPlugin(loader.getOSName("traj_fallback_node"));
    factory.registerFromPlugin(loader.getOSName("counter_node"));

    std::string pkgpath = ament_index_cpp::get_package_share_directory("oss_behavior_tree");
    std::string xml_file = pkgpath + "/xml/oss_arm_bt2.xml";
	
	auto tree = factory.createTreeFromFile(xml_file, blackboard);

    BT::Groot2Publisher publisher(tree);


    while (rclcpp::ok()) 
    {
        tree.tickWhileRunning();
        rclcpp::spin_some(node);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        rate.sleep();
    }

    // std::cout << "Hello World";
 
    rclcpp::shutdown();
    return 0;
}