#!/usr/bin/env python3

"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

import os
import rclpy
from ras_bt_framework.managers.behavior_tree_generator import BehaviorTreeGenerator,PrimitiveActionManager
from ras_bt_framework.behaviors.modules import SaySomethingSequence,MyCustomSequence,PickObject, RotateEffectorSequence
from ras_bt_framework.behaviors.primitives import RotateEffector, SaySomething,ThinkSomethingToSay,MoveToPose,Trigger
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml
from ras_bt_framework.managers.keywords_module_generator import  KeywordModuleGenerator
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper

from ras_bt_framework.behaviors.modules import SaySomethingSequence,MyCustomSequence,PickObject,BehaviorModuleSequence
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
        self.target_pose_map =  TargetPoseMap()
        self.keyword_module_gen = KeywordModuleGenerator()
        self.keyword_module_gen.register(
            {
                "move2pose":self.target_pose_map.move2pose_module,
                "move2pose_sequence":self.target_pose_map.move2pose_sequence_module,
            }
        )
        self.main_module = BehaviorModuleSequence()
    
    def load_exp(self, req, resp):
        self.sequence_list = []
        exp_id = req.exepriment_id
        path = os.path.join(os.environ["RAS_APP_PATH"],"configs","experiments",f"{exp_id}.yaml")
        # print(path)
        pose_dict,targets = read_yaml_to_pose_dict(path)
        for _k,_v in pose_dict.items():
            self.target_pose_map.register_pose(_k,_v)
        self.main_module = self.keyword_module_gen.generate("MainModule",targets)
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
        # behavior = PickObject(self.sequence_list)
        self.run_module(self.main_module,path)
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
