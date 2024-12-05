#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "oss_interfaces/srv/pose_req.hpp" 
#include <oss_interfaces/srv/joint_sat.hpp>
#include <oss_interfaces/srv/load_exp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/point.h>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/position_constraint.h>
#include <moveit_msgs/msg/robot_state.h>
#include <moveit_msgs/msg/workspace_parameters.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>

#include <string>
#include <cmath>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "lite6";

class TestTrajectory : public rclcpp::Node {
public:
  TestTrajectory(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("test_trajectory"),
        move_group_arm(move_group_node, PLANNING_GROUP_ARM)
         {
         RCLCPP_INFO(LOGGER, "Node Started");


    // Initialize the publisher here
    trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_topic", 10);

    move_to_pose_srv_ = this->create_service<oss_interfaces::srv::PoseReq>(
        "/create_traj",
        std::bind(&TestTrajectory::move_to_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    fallback_srv = this->create_service<oss_interfaces::srv::LoadExp>(
        "/fallback_info",
        std::bind(&TestTrajectory::fallback_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    fallback_client = this->create_client<oss_interfaces::srv::LoadExp>(
      "/fallback_pose"
    );

    reset_srv = this->create_service<std_srvs::srv::SetBool>(
        "/reset_traj_gen",
        std::bind(&TestTrajectory::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    

    sync_srv = this->create_service<oss_interfaces::srv::JointSat>(
        "/sync_arm",
        std::bind(&TestTrajectory::sync_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&TestTrajectory::joint_state_callback, this, std::placeholders::_1));

  } // end of constructor
  public:
    bool fallback_flag;

  // Getting Basic Information
  void get_info() {
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
  }

  void fallback_callback(const std::shared_ptr<oss_interfaces::srv::LoadExp::Request> request,
      std::shared_ptr<oss_interfaces::srv::LoadExp::Response> response)
  {
    move_group_arm.stop();
    std::vector<double> joint_values2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_arm.setJointValueTarget(joint_values2);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    bool success = (move_group_arm.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan2);
    
    std::cout<<"fallback"<<std::endl;
    fallback_flag = true;
    auto result_future = fallback_client->async_send_request(
                request, std::bind(&TestTrajectory::fallback_response, this,
                                std::placeholders::_1));  
    response->success;
  }

  

  void reset_callback( const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    move_group_arm.clearPathConstraints();
    std::vector<double> joint_values3 = { 0.0, 0.172788, 0.5550147, 0.0, 0.3822271, 0.0};
    move_group_arm.setJointValueTarget(joint_values3);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
    bool success = (move_group_arm.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arm.execute(my_plan3); 
    trajectory_msg = my_plan3.trajectory_.joint_trajectory;
    trajectory_pub->publish(trajectory_msg);
    fallback_flag = false;
    response->success = 1;
  }

  void fallback_response(
    rclcpp::Client<oss_interfaces::srv::LoadExp>::SharedFuture future) {
   
  }

  void sync_callback(   const std::shared_ptr<oss_interfaces::srv::JointSat::Request> request,
      std::shared_ptr<oss_interfaces::srv::JointSat::Response> response)
  {
    std::vector<double> joint_values;
    for (const auto& x : request->joint_state.position)
    {
      joint_values.push_back(x);
    }

    move_group_arm.setJointValueTarget(joint_values);
    if(fallback_flag == false)
    {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std::cout<<"sync"<<std::endl;
    move_group_arm.execute(my_plan);

    response->successq = 1;

    }
    else
    {
      response->successq = 1;
    }

  }
  
  void current_state() {
    RCLCPP_INFO(LOGGER, "Get Robot Current State");

    current_state_arm = move_group_arm.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
  }

  void set_constraints()
  {
    RCLCPP_INFO(LOGGER, "Orientation Constrains Set");

   // Goal constraints - position
    moveit_msgs::msg::Constraints goal_constraints;

    // Goal constraints - orientation
    moveit_msgs::msg::OrientationConstraint ori_constraint;
    ori_constraint.header.stamp = this->get_clock()->now(); // Set the current time
    ori_constraint.header.frame_id = "link_base";
    ori_constraint.orientation.x = -0.707388044876899;
    ori_constraint.orientation.y = -0.7068249569936026;
    ori_constraint.orientation.z = 0.0005628637715330792;
    ori_constraint.orientation.w = 0.0005633121735972125;
    ori_constraint.link_name = "link_eef";
    ori_constraint.absolute_x_axis_tolerance = 0.75;
    ori_constraint.absolute_y_axis_tolerance = 0.75;
    ori_constraint.absolute_z_axis_tolerance = 0.75;
    ori_constraint.weight = 1.0;
    ori_constraint.parameterization = 1.0;
    goal_constraints.orientation_constraints.push_back(ori_constraint);

    move_group_arm.setPathConstraints(goal_constraints);
  }

  void type_A(geometry_msgs::msg::Pose target_pose) {
    trajectory_msgs::msg::JointTrajectory trajectory_msg;

    move_group_arm.setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    move_group_arm.setPlannerId("RRTConnectkConfigDefault");

    move_group_arm.setNumPlanningAttempts(5);
    move_group_arm.setPlanningTime(2);
    move_group_arm.setGoalTolerance(0.005);
    move_group_arm.setGoalOrientationTolerance(0.005);
    move_group_arm.setMaxVelocityScalingFactor(0.4);
    move_group_arm.setMaxAccelerationScalingFactor(0.4);



    set_constraints();

    float z_offset = 0.1;

    target_pose.position.z = target_pose.position.z + z_offset;

    move_group_arm.setPoseTarget(target_pose);

    int count = 5;
    for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
      bool success = (move_group_arm.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan4.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan4);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan7;
      move_group_arm.plan(my_plan7);
      move_group_arm.execute(my_plan7);
      trajectory_msg = my_plan7.trajectory_.joint_trajectory;
      trajectory_pub->publish(trajectory_msg);
      }
      
    }
    
    
    set_constraints();

    target_pose.position.z = target_pose.position.z - z_offset;

    move_group_arm.setPoseTarget(target_pose);

     for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
      bool success = (move_group_arm.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan5.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan5);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan8;
      move_group_arm.plan(my_plan8);
      move_group_arm.execute(my_plan8);     
      trajectory_msg = my_plan8.trajectory_.joint_trajectory;   
      trajectory_pub->publish(trajectory_msg); 
      }
      
    }

    set_constraints();

    target_pose.position.z = target_pose.position.z + z_offset;

    move_group_arm.setPoseTarget(target_pose);

    for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
      bool success = (move_group_arm.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan6.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan6);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan9;
      move_group_arm.plan(my_plan9);
      move_group_arm.execute(my_plan9);
      trajectory_msg = my_plan9.trajectory_.joint_trajectory;
      trajectory_pub->publish(trajectory_msg);
     
      }
      
    }
  }

  void type_B(geometry_msgs::msg::Pose target_pose) {

    target_pose.orientation.x = -0.7032384;
    target_pose.orientation.y = -0.0075927;
    target_pose.orientation.z = 0.7108721;
    target_pose.orientation.w = 0.0076751;
    
    float offset = 0.05;
    trajectory_msgs::msg::JointTrajectory trajectory_msg;

    double theta = atan2(target_pose.position.y, target_pose.position.x); 
    target_pose.position.x = (std::sqrt(std::pow(target_pose.position.x, 2) + std::pow(target_pose.position.y, 2)) - offset) * cos(theta);
    target_pose.position.y = (std::sqrt(std::pow(target_pose.position.x, 2) + std::pow(target_pose.position.y, 2)) - offset) * sin(theta);

    move_group_arm.setPoseTarget(target_pose);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    trajectory_msg = my_plan.trajectory_.joint_trajectory;

    if (success) {
      move_group_arm.move();
    }

    if (trajectory_pub) {
      trajectory_pub->publish(trajectory_msg);
    } else {
      RCLCPP_ERROR(LOGGER, "trajectory_pub is not initialized");
    }

    target_pose.position.x = target_pose.position.x + offset * sin(theta);
    target_pose.position.y = target_pose.position.y + offset * cos(theta);

    move_group_arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    bool success2 = (move_group_arm.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    trajectory_msg = my_plan2.trajectory_.joint_trajectory;

    if (success2) {
      move_group_arm.move();
    }

    if (trajectory_pub) {
      trajectory_pub->publish(trajectory_msg);
    } else {
      RCLCPP_ERROR(LOGGER, "trajectory_pub is not initialized");
    }
  }

  void type_C(geometry_msgs::msg::Pose target_pose) {
    trajectory_msgs::msg::JointTrajectory trajectory_msg;

    move_group_arm.setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    move_group_arm.setPlannerId("RRTConnectkConfigDefault");

    move_group_arm.setNumPlanningAttempts(5);
    move_group_arm.setPlanningTime(2);
    move_group_arm.setGoalTolerance(0.005);
    move_group_arm.setGoalOrientationTolerance(0.005);
    move_group_arm.setMaxVelocityScalingFactor(0.4);
    move_group_arm.setMaxAccelerationScalingFactor(0.4);



    set_constraints();

    float z_offset = 0.1;

    target_pose.position.z = target_pose.position.z + z_offset;

    move_group_arm.setPoseTarget(target_pose);

    int count = 5;
    for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
      bool success = (move_group_arm.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan4.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan4);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan7;
      move_group_arm.plan(my_plan7);
      move_group_arm.execute(my_plan7);
      trajectory_msg = my_plan7.trajectory_.joint_trajectory;
      trajectory_pub->publish(trajectory_msg);
      }
      
    }
    
    
    set_constraints();

    target_pose.position.z = target_pose.position.z - z_offset;

    move_group_arm.setPoseTarget(target_pose);

     for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
      bool success = (move_group_arm.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan5.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan5);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan8;
      move_group_arm.plan(my_plan8);
      move_group_arm.execute(my_plan8);     
      trajectory_msg = my_plan8.trajectory_.joint_trajectory;   
      trajectory_pub->publish(trajectory_msg); 
      }
      
    }

     for (int i = 0; i < count; i++)
    {
      std::vector<double> joint_values3 = {joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3], joint_angle[4], 1.57};
      move_group_arm.setJointValueTarget(joint_values3);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
      bool success = (move_group_arm.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
      move_group_arm.execute(my_plan3); 
      trajectory_msg = my_plan3.trajectory_.joint_trajectory;
      trajectory_pub->publish(trajectory_msg);
        break;
      }   
    }

    set_constraints();

    target_pose.position.z = target_pose.position.z + z_offset;

    move_group_arm.setPoseTarget(target_pose);

    for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
      bool success = (move_group_arm.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan6.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm.execute(my_plan6);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else if (i == 4)
      {
      move_group_arm.clearPathConstraints();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan9;
      move_group_arm.plan(my_plan9);
      move_group_arm.execute(my_plan9);
      trajectory_msg = my_plan9.trajectory_.joint_trajectory;
      trajectory_pub->publish(trajectory_msg);
     
      }
      
    }

  }

  void move_to_pose_callback(
      const std::shared_ptr<oss_interfaces::srv::PoseReq::Request> request,
      std::shared_ptr<oss_interfaces::srv::PoseReq::Response> response) 
  {
    RCLCPP_WARN(this->get_logger(), "Received Pose request...");

    auto msg = request->object_pose;

    // Log the received pose
    RCLCPP_INFO(this->get_logger(),
        "Received pose:\n"
        "Position: x=%f, y=%f, z=%f\n"
        "Orientation: x=%f, y=%f, z=%f, w=%f",
        msg.position.x, msg.position.y, msg.position.z,
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    );
    std::string item = request->type;
    geometry_msgs::msg::Pose config_pose;
    config_pose = request->object_pose;
    std::string comp = "test_tube";
    std::string comp2 = "jar";

    if (item == comp)
    {
      type_B(config_pose);
    }
    else if (item == comp2)
    {
      type_C(config_pose);
    }
    else
    {
      type_A(config_pose);
    }
    

    response->success = 1;
  }

  void plan_arm_joint_space() {
    RCLCPP_INFO(LOGGER, "Planning to Joint Space");

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.709139;
    target_pose.orientation.y = 0.7050171;
    target_pose.orientation.z = -0.0072675;
    target_pose.orientation.w = 0.0044732;
    target_pose.position.x = -0.298;
    target_pose.position.y = -0.053;
    target_pose.position.z = 0.272;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.x = -0.7032384;
    target_pose2.orientation.y = -0.0075927;
    target_pose2.orientation.z = 0.7108721;
    target_pose2.orientation.w = 0.0076751;
    target_pose2.position.x = -0.298;
    target_pose2.position.y = -0.053;
    target_pose2.position.z = 0.272;

    type_B(target_pose2);
  }

private:

  std::vector<float> joint_angle;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)  
  {
    joint_angle.clear();

    for (auto i : msg->position){
      joint_angle.push_back(i);
    }
  }
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  moveit::planning_interface::MoveGroupInterface move_group_arm;

  const moveit::core::JointModelGroup *joint_model_group_arm;

  moveit::core::RobotStatePtr current_state_arm;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub; 
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Service<oss_interfaces::srv::PoseReq>::SharedPtr move_to_pose_srv_;
  rclcpp::Service<oss_interfaces::srv::JointSat>::SharedPtr sync_srv;
  rclcpp::Service<oss_interfaces::srv::LoadExp>::SharedPtr fallback_srv;
  rclcpp::Client<oss_interfaces::srv::LoadExp>::SharedPtr fallback_client;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_srv;
  
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<TestTrajectory> planner_node =
      std::make_shared<TestTrajectory>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
