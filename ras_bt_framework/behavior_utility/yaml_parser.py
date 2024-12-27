#!/usr/bin/env python3

"""
Copyright (C) 2024 Harsh Davda

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
Harsh Davda
Email: info@opensciencestack.org
"""

import os
import yaml
import tf_transformations


def read_yaml_to_pose_dict(path):
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'poses' is missing from the YAML file.")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(pose_values['roll'], pose_values['pitch'], pose_values['yaw'])
        pose_dict[pose_name] = ",".join(map(str,[
            pose_values['x'],
            pose_values['y'],
            pose_values['z'],
            qx, qy, qz, qw
        ]))


    if 'targets' not in data:
        raise KeyError("The key 'targets' is missing from the YAML file.")
        
    # target_pose = []

    # for i in data["targets"]:
    #     if i == "grasp":
    #         target_pose.append(True)
    #     elif i == "release":
    #         target_pose.append(False)
    #     elif type(i) == float:
    #         target_pose.append(i)
    #     else:
    #         for j in pose_dict:
    #             if j == i:
    #                 target_pose.append(pose_dict[j])

    return pose_dict,data["targets"]
