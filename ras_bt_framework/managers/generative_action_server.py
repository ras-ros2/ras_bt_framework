#!/usr/bin/env python3

"""
Copyright (C) 2025 Sachin Kumar

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
Sachin Kumar
Email: info@opensciencestack.org
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from dataclasses import dataclass,field
from typing import Callable,ClassVar,Set,Dict, List
from ras_interfaces.action import BTGenerative
from ras_interfaces.msg import BTNodeStatus, BTGenMapping
import json
import importlib
from types import ModuleType
from ras_bt_framework.behavior_template.instruction import GenerativeInstruction
from ras_common.package.utils import include_module_import_class, get_module_path_and_class_name, ObjectInfo

def convert_dict_to_gen_mapping(port_map: dict[str, str]) -> List[BTGenMapping]:
    result: List[BTGenMapping] = list()
    for key, value in port_map.items():
        result.append(BTGenMapping(key=key, value=value))
    return result

# TODO: implement later
def convert_gen_mapping_to_dict():
    pass

class GenerativeActionServer():
    def __init__(self,node:Node):
        self._node = node
        self._action_server = ActionServer(self._node, BTGenerative, 'bt_generative', self.execute_callback)

    def execute_callback(self, action_goal: ServerGoalHandle):
        self._node.get_logger().info('Executing goal...')
        req : BTGenerative.Goal = action_goal.request
        module_path: BTGenerative.Goal.module_path = req.module_path
        class_name: BTGenerative.Goal.class_name = req.class_name
        class_argument: Dict[str,str] = req.class_arguments
        input_ports: Dict[str,str] = req.input_ports

        class_ = include_module_import_class(module_path, class_name)
        if class_ is None:
            self._node.get_logger().error(f"Class {class_name} not found for module {module_path}")
            result = BTGenerative.Result()
            result.status = BTNodeStatus.FAILURE
            action_goal.abort()
            return result

        try:
            self.generative_class: GenerativeInstruction = class_(**class_argument, **input_ports)
            self.generative_class.tick()
            output_port_map: dict[str, str] = self.generative_class._output_ports
            result = BTGenerative.Result()
            result.status = BTNodeStatus.SUCCESS
            result.output_ports = convert_dict_to_gen_mapping(output_port_map)
            action_goal.succeed()
            return result
            

        except Exception as e:
            self._node.get_logger().error(f"Error executing generative class: {e}")
            result = BTGenerative.Result()
            result.status = BTNodeStatus.FAILURE
            action_goal.abort()
            return result

        # feedback_msg = BTGenerative.Feedback()
        # feedback_msg.status = BTNodeStatus.RUNNING
        # action_goal.publish_feedback(feedback_msg)
