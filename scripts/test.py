#!/usr/bin/env python3

from ras_bt_framework.managers.keywords_module_generator import  KeywordModuleGenerator
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper

def main():
    # print(callable(rotate))
    # print(callable(gripper))
    # print(callable(TargetPoseMap))
    # exit()
    tpm = TargetPoseMap()
    kmg = KeywordModuleGenerator()
    tpm.register_pose("pose1","1,2,3")
    kmg.register({
        "move2pose":tpm.move2pose_module,
        "rotate":rotate,
        "gripper":gripper
    })
    mod = kmg.generate("my_module",[{"rotate":1.35},{"gripper":False},{"move2pose":{"pose":"pose1"}}])
    print(mod)

if __name__ == "__main__":
    main()