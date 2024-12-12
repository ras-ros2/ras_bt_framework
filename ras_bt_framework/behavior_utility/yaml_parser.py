#!/usr/bin/env python3
import os
import yaml
import tf_transformations
from ament_index_python import get_package_share_directory
from ..managers.keywords_module_generator import KeywordModuleGenerator
def read_yaml_to_pose_dict(path):
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'poses' is missing from the YAML file.")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(pose_values['roll'], pose_values['pitch'], pose_values['yaw'])
        pose_dict[pose_name] = (
            pose_values['x'],
            pose_values['y'],
            pose_values['z'],
            qx, qy, qz, qw
        )
        
    if 'targets' not in data:
        raise KeyError("The key 'targets' is missing from the YAML file.")
        
    target_pose = []

    for i in data["targets"]:
        if i == "grasp":
            target_pose.append(True)
        elif i == "release":
            target_pose.append(False)
        elif type(i) == float:
            target_pose.append(i)
        else:
            for j in pose_dict:
                if j == i:
                    target_pose.append(pose_dict[j])

    return target_pose
