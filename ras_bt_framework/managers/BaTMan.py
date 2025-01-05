from .behavior_tree_generator import BehaviorTreeGenerator
from .primitive_action_manager import PrimitiveActionManager
from ..behavior_template.instruction import TrajectoryPrimitive
# from .BTconverter import BTconverter
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ras_interfaces.action import BTInterface
from pathlib import Path
class BaTMan(Node):
    def __init__(self,mode_sim):
        super().__init__("batman")
        self.mode_sim = mode_sim
        self.alfred = PrimitiveActionManager(self)
        self.converter = None
        if self.mode_sim:
            self.converter = BTconverter(self)
        self.manager = BehaviorTreeManager(self.alfred,self.mode_sim,self.converter)
        self._action_client = ActionClient(self,BTInterface,"bt_executor")
    
    def send_goal(self,path:str):
        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0} {1}'.format(feedback.status,feedback.primitive))
        if self.mode_sim and feedback.status == "0":
            primitive = self.manager.get_registered_primitive(feedback.primitive)
            if issubclass(primitive,TrajectoryPrimitive):
                self.converter.prim_seq.append(primitive)
                self.converter.pull_to_map()
    
    def run_module(self,behavior,path:Path):
        self.manager.feed_root(behavior)
        self.manager.verify_sanity()
        bt_path = str(path.resolve().absolute())
        self.manager.generate_xml_trees(bt_path)
        self.send_goal(bt_path)