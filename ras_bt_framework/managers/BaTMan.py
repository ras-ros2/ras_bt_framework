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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from ras_interfaces.action import BTInterface
from ras_interfaces.srv import LoadExp
from std_srvs.srv import SetBool
from rclpy.node import Node
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml
from pathlib import Path
import xml.etree.ElementTree as ET
import time,os

class BaTMan(Node):
    def __init__(self):
        super().__init__("batman")
        # self.mode_sim = mode_sim
        self.alfred = PrimitiveActionManager(self)
        # self.converter = None
        self.my_callback_group = ReentrantCallbackGroup()
        self.get_logger().info("Node Init")

        self.manager = BehaviorTreeGenerator(self.alfred)
        self._action_client = ActionClient(self,BTInterface,"bt_executor")
        self.create_service(SetBool, "/test_experiment", self.bt_execution_callback,callback_group=self.my_callback_group)
        self.counter_reset_client = self.create_client(SetBool, '/reset_counter', callback_group=self.my_callback_group)
        self.create_service(LoadExp, "/get_exepriment", self.load_exp, callback_group=self.my_callback_group)
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
    
    def send_goal(self,path:str):
        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    

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
        status = self.run_module(self.main_module,path)
        if status in [BTNodeStatus.SUCCESS,BTNodeStatus.IDLE]:
            self.get_logger().info("BT Execution Successful")
        else:
            self.get_logger().info("BT Execution Failed")
            # resp.success = False
            # return resp
        self.get_logger().info("real_bt_generation_started")
        tree = ET.parse(path)
        root = tree.getroot()
        update_xml(root)
        tree.write("/ras_sim_lab/ros2_ws/src/ras_bt_framework/xml/real.xml", encoding="utf-8", xml_declaration=True)
        resp.success = True
        return resp
    
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

    def run_module(self,behavior,path:Path):
        self.manager.feed_root(behavior)
        self.manager.verify_sanity()
        path = Path(path)
        bt_path = str(path.resolve().absolute())
        self.manager.generate_xml_trees(bt_path)
        self.send_goal(bt_path)
        time.sleep(2)
        return self.tick_loop()