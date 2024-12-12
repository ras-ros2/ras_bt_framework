#!/usr/bin/env python3
import os
import yaml
import tf_transformations
from ament_index_python import get_package_share_directory
from ..managers.keywords_module_generator import KeywordModuleGenerator
def read_yaml_to_pose_dict(path):

     # Load lab_setup.yaml
    with open("/ras_sim_lab/configs/lab_setup.yaml", 'r') as lab_file:
        lab_data = yaml.safe_load(lab_file)

    locations = lab_data.get('locations', {})

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'poses' is missing from the YAML file.")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(pose_values['roll'], pose_values['pitch'], pose_values['yaw'])
        if 'location_id' in pose_values:
            location_id = pose_values['location_id']
            loc_values = locations.get(location_id, {})
            x, y, z = loc_values.get('x', 0), loc_values.get('y', 0), loc_values.get('z', 0)
            pose_dict[pose_name] = (
                x,
                y,
                z,
                qx, qy, qz, qw
            )
            print(f"Pose {pose_name} is at location {location_id} with values {x}, {y}, {z}")
        else:
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
