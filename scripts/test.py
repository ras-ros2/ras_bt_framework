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
    btm = BaTMan(mode_sim=True)
    btm.run_module(myBehavior,"test.xml")
    rclpy.spin(btm)

if __name__ == "__main__":
    main()