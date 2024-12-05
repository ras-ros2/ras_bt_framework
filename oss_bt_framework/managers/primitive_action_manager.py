import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from dataclasses import dataclass
from typing import Callable
from oss_interfaces.action import BTInterface
import json

class PrimitiveActionHandler(object):
    def __init__(self,name:str,func:Callable):
        self.name = name
        self.func = func

    def call(self,*args,**kwargs):
        return self.func(*args,**kwargs)
    
class PrimitiveActionManager(Node):
    def __init__(self):
        super().__init__('primitive_action_manager')
        self._action_server = ActionServer(
            self,
            BTInterface,
            'PAMServer',
            self.execute_callback)
        self.handlers = dict()
    
    def register_action(self,name:str,func:Callable):
        self.handlers[name] = PrimitiveActionHandler(name,func)

    def execute_callback(self, action_goal:BTInterface.Goal):
        self.get_logger().info('Executing goal...')
        handler_id = action_goal.identifier
        if handler_id not in self.handlers:
            raise ValueError(f"Handler {handler_id} not found")
        params = json.loads(action_goal.param_json)
        result = BTInterface.Result()
        result.success = self.handlers[handler_id].call(**params)
        return result