#!/usr/bin/env python3
import os
import rclpy
from ras_bt_framework.managers.behavior_tree_generator import BehaviorTreeGenerator,PrimitiveActionManager
from ras_bt_framework.behaviors.modules import SaySomethingSequence,MyCustomSequence,PickObject, RotateEffectorSequence
from ras_bt_framework.behaviors.primitives import RotateEffector, SaySomething,ThinkSomethingToSay,MoveToPose,Trigger
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml

from ras_bt_framework.behaviors.modules import SaySomethingSequence,MyCustomSequence,PickObject
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from ras_interfaces.action import BTInterface
from ras_interfaces.srv import LoadExp
from std_srvs.srv import SetBool
from pathlib import Path

from ament_index_python import get_package_share_directory

import xml.etree.ElementTree as ET
import json



class Batman(Node):
    def __init__(self):
        super().__init__("batman")
        self.my_callback_group = ReentrantCallbackGroup()
        self.get_logger().info("Node Init")
        self.manager = BehaviorTreeGenerator(PrimitiveActionManager())
        self._action_client = ActionClient(self,BTInterface,"bt_executor")
        self.create_service(SetBool, "/test_experiment", self.bt_execution_callback,callback_group=self.my_callback_group)
        self.counter_reset_client = self.create_client(SetBool, '/reset_counter', callback_group=self.my_callback_group)
        self.create_service(LoadExp, "/get_exepriment", self.load_exp, callback_group=self.my_callback_group)
    
    def load_exp(self, req, resp):
        self.sequence_list = []
        exp_id = req.exepriment_id
        path = os.path.join(os.environ["RAS_APP_PATH"],"configs","experiments",f"{exp_id}.yaml")
        print(path)
        pose_list = read_yaml_to_pose_dict(path)

        for i in pose_list:
            if type(i) == bool:
                self.sequence_list.append(Trigger(input_ports={"trigger": f"{i}"}))
            if type(i) == float:
                self.sequence_list.append(RotateEffector(input_ports={"rotation_angle": f"{i}"}))
            elif type(i) == tuple:
                self.sequence_list.append(MoveToPose(input_ports={"pose":",".join(map(str, i))}))
        resp.success = True

        self.get_logger().info("Experiment Loaded...")

        return resp

    def bt_execution_callback(self, req, resp):
        self.get_logger().info("Batman Called ...")
        if len(self.sequence_list) == 0:
            self.get_logger().warning("Load Experiment First....")
            pass
        counter_reset = SetBool.Request()
        counter_reset.data = True
        self.counter_reset_client.call_async(counter_reset)
        path = Path(os.environ["RAS_WORKSPACE_PATH"])/"src"/"ras_bt_framework"/"xml"/"sim.xml"
        behavior = PickObject(self.sequence_list)
        self.run_module(behavior,path)
        self.get_logger().info("real_bt_generation_started")
        tree = ET.parse(path)
        root = tree.getroot()
        update_xml(root)
        tree.write("/ras_sim_lab/ros2_ws/src/ras_bt_framework/xml/real.xml", encoding="utf-8", xml_declaration=True)
        resp.success = True
        return resp

    
    def send_goal(self,path:str):
        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    
    def run_module(self,behavior,path:Path):
        self.manager.feed_root(behavior)
        self.manager.verify_sanity()
        bt_path = str(path.resolve().absolute())
        self.manager.generate_xml_trees(bt_path)
        future = self.send_goal(bt_path)
        rclpy.spin_until_future_complete(self,future)

def main():
    rclpy.init(args=None)
    batman = Batman()
    while rclpy.ok():
        rclpy.spin_once(batman)
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()
