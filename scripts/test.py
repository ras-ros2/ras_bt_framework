#!/usr/bin/env python3

from ras_bt_framework.managers.keywords_module_generator import  KeywordModuleGenerator
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper
from ras_bt_framework.behavior_template.keyword import keyword2module,BehaviorModule
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
    print(keyword2module(tpm.move2pose_module,"move2pose","pose1"))
    print(keyword2module(tpm.move2pose_module,"move2pose",{"pose":"pose1"}))
    # print(keyword2module(tpm.move2pose_module,"move2pose",["pose1"]))
    print(keyword2module(hello))

if __name__ == "__main__":
    main()