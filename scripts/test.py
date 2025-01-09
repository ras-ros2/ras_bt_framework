#!/usr/bin/env python3
import rclpy
from ras_bt_framework.generators.behavior_tree_generator import BehaviorTreeGenerator
from ras_bt_framework.managers.primitive_action_manager import PrimitiveActionManager
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper
from ras_bt_framework.behavior_template.keyword import keyword2module,BehaviorModule,BehaviorModuleSequence
from ras_bt_framework.behaviors.functions import Hello,SaySomethingPy
from ras_bt_framework.behaviors.modules import SaySomething,ThinkSomethingToSay
from ras_bt_framework.managers.BaTMan import BaTMan
import rclpy.node
import os
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml, update_bt
def hello():
    print("hello")
    return BehaviorModule()
def main():
    tpm = TargetPoseMap()
    # kmg = KeywordModuleGenerator()
    tpm.register_pose("pose1","1,2,3")
    # kmg.register({
    #     "move2pose":tpm.move2pose_module,
    #     "rotate":rotate,
    #     "gripper":gripper
    # })
    myBehavior = BehaviorModuleSequence()
    myBehavior.add_children([
        # keyword2module(tpm.move2pose_module,"move2pose",{"pose":"pose1"}),
        # keyword2module(rotate,"rotate",{"angle":90}),
        # keyword2module(gripper,"gripper",True),
        # SaySomething(input_ports={"message":"hello"}),
        Hello(),
        SaySomethingPy(to_say="hellopy",next_line="this is a test"),
        # SaySomething(input_ports={"message":"bye"}),
        # keyword2module(hello)
    ])
    rclpy.init()
    # node = rclpy.node.Node("my_node")
    # pam = PrimitiveActionManager()
    # my_generator = BehaviorTreeGenerator(pam)
    # my_generator.feed_root(myBehavior)
    # my_generator.verify_sanity()
    # my_generator.generate_xml_trees("test.xml")
    btm = BaTMan()
    path = os.path.join(os.environ["RAS_APP_PATH"],"configs","experiments","0.yaml")
        # print(path)
    pose_dict,targets = read_yaml_to_pose_dict(path)
    btm.generate_module_from_keywords(targets,pose_dict)
    new_module = update_bt(btm.main_module)
    btg = BehaviorTreeGenerator(btm.alfred)
    bt_path = "/ras_sim_lab/ros2_ws/src/ras_bt_framework/xml/real.xml"
    btg.feed_root(new_module)
    try:
        btg.generate_xml_trees(bt_path)
    except Exception as e:
        btm.get_logger().error(f"Error in BT Generation: {e}")
        exit(1)
    # btm.run_module(myBehavior,"test.xml")
    btm.execute_bt("/ras_real_lab/ros2_ws/src/ras_aws_transport/real_bot_zip/real.xml")
    rclpy.spin(btm)

if __name__ == "__main__":
    main()