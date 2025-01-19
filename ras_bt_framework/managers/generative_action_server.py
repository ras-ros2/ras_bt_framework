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
from typing import Callable,ClassVar,Set,Dict
from ras_interfaces.action import BTGenerative
from ras_interfaces.msg import BTNodeStatus
import json
import importlib
from types import ModuleType
from ras_bt_framework.behavior_template.instruction import GenerativeInstruction

class GenerativeActionServer():
    def __init__(self,node:Node):
        self._node = node
        self._action_server = ActionServer(self._node, BTGenerative, 'bt_generative', self.execute_callback)

    def include_module_import_class(self, module_path:str, class_name:str) -> type:
        self._node.get_logger().info(f"module path: {module_path}")
        module: ModuleType = importlib.import_module(module_path)
        try:
            class_ = getattr(module, class_name)
        except:
            self._node.get_logger().error(f"Class {class_name} not found in module {module_path}")
            return None

        return class_
    
    def execute_callback(self, action_goal: ServerGoalHandle):
        self._node.get_logger().info('Executing goal...')
        req : BTGenerative.Goal = action_goal.request
        module_path: BTGenerative.Goal.module_path = req.module_path
        class_name: BTGenerative.Goal.class_name = req.class_name
        class_argument: Dict[str,str] = req.class_argument
        input_ports: Dict[str,str] = req.input_ports

        class_ = self.include_module_import_class(module_path, class_name)

        self.generative_class: GenerativeInstruction = class_(**class_argument, **input_ports)
        self.generative_class.tick()

        feedback_msg = BTGenerative.Feedback()
        feedback_msg.status = BTNodeStatus.RUNNING
        action_goal.publish_feedback(feedback_msg)
