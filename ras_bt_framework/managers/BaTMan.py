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

from ..generators.behavior_tree_generator import BehaviorTreeGenerator
from .primitive_action_manager import PrimitiveActionManager
# from ..behavior_template.instruction import TrajectoryPrimitive
# from .BTconverter import BTconverter
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper
from ras_bt_framework.generators.keywords_module_generator import  KeywordModuleGenerator
from ras_bt_framework.behaviors.modules import SaySomethingSequence,MyCustomSequence,PickObject,BehaviorModuleSequence
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from ras_interfaces.action import BTInterface
from ras_interfaces.srv import PrimitiveExec
from ras_interfaces.msg import BTNodeStatus

from rclpy.action import ActionClient
from ras_interfaces.action import BTInterface

from rclpy.node import Node

from pathlib import Path

import time

class BaTMan(Node):
    def __init__(self):
        super().__init__("batman")
        # self.mode_sim = mode_sim
        self.alfred = PrimitiveActionManager(self)
        # self.converter = None
        
        self.get_logger().info("Node Init")

        self.manager = BehaviorTreeGenerator(self.alfred)
        self._action_client = ActionClient(self,BTInterface,"bt_executor")
        self.target_pose_map =  TargetPoseMap()
        self.keyword_module_gen = KeywordModuleGenerator()
        self.keyword_module_gen.register({
            "move2pose":self.target_pose_map.move2pose_module,
            "rotate":rotate,
            "gripper":gripper
        })
        self.main_module = BehaviorModuleSequence()
        self.tick_cli = self.create_client(PrimitiveExec, '/bt_tick')
        self.loop_rate = self.create_rate(10)
        self.session_started = False

    def generate_module_from_keywords(self,keywords:list,pose_targets:dict):
        for _k,_v in pose_targets.items():
            self.target_pose_map.register_pose(_k,_v)
        self.main_module = self.keyword_module_gen.generate("MainModule",keywords)
    
    def send_goal(self,path:str):
        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future : rclpy.Future):
        goal_handle : ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.session_started = True

    def get_result_callback(self, future : rclpy.Future):
        result : BTInterface.Result = future.result()
        self.get_logger().info('Result: {0}'.format(result.status))
    
    def feedback_callback(self, feedback_msg ):
        feedback : BTInterface.Feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0} {1}'.format(feedback.status,feedback.behavior_stack))
        # if self.mode_sim and feedback.status == "0":
        #     primitive = self.manager.get_registered_primitive(feedback.primitive)
            # if issubclass(primitive,TrajectoryPrimitive):
            #     self.converter.prim_seq.append(primitive)
            #     self.converter.pull_to_map()
    
    def tick_loop(self):
        req = PrimitiveExec.Request()
        status = BTNodeStatus.IDLE
        while rclpy.ok():
            # if(self.session_started):
            if status == BTNodeStatus.IDLE:
                self.get_logger().info("Sending tick")
                future = self.tick_cli.call_async(req)
                rclpy.spin_until_future_complete(self,future)
                resp : PrimitiveExec.Response = future.result()
                status = resp.status.data
            
            print("status is",status)
            if(status in [BTNodeStatus.FAILURE,BTNodeStatus.SKIPPED,BTNodeStatus.SUCCESS,BTNodeStatus.IDLE]):
                break
            self.loop_rate.sleep()
            rclpy.spin_once(self,timeout_sec=0.1)
        return status
    
    def execute_bt(self,bt_path:str|Path):
        bt_path = str(bt_path)
        self.send_goal(bt_path)
        time.sleep(2)
        return self.tick_loop()

    def run_module(self,path:Path,behavior:BehaviorModuleSequence = None):
        if not isinstance(behavior,BehaviorModuleSequence):
            if not isinstance(self.main_module,BehaviorModuleSequence):
                raise ValueError("Invalid module supplied")
            behavior = self.main_module
        self.manager.feed_root(behavior)
        self.manager.verify_sanity()
        path = Path(path)
        bt_path = str(path.resolve().absolute())
        self.manager.generate_xml_trees(bt_path)
        return self.execute_bt(bt_path)