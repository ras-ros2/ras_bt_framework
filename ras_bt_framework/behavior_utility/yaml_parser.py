#!/usr/bin/env python3

# """
# Copyright (C) 2024 Harsh Davda

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

# For inquiries or further information, you may contact:
# Harsh Davda
# Email: info@opensciencestack.org
# """

# import os
# import yaml
# import tf_transformations
# from .grid_parser import GridConfig
# from ..behaviors.ports import PortPoseCfg
# from ras_common.config.loaders.common import PoseConfig


# def read_yaml_to_pose_dict(path):
#     with open(path, 'r') as file:
#         data = yaml.safe_load(file)
    
#     if 'Poses' not in data:
#         raise KeyError("The key 'poses' is missing from the YAML file.")
    
#     pose_dict = {}
#     for pose_name, pose_values in data['Poses'].items():
#         # qx, qy, qz, qw = tf_transformations.quaternion_from_euler(pose_values['roll'], pose_values['pitch'], pose_values['yaw'])
#         # pose_dict[pose_name] = ",".join(map(str,[
#         #     pose_values['x'],
#         #     pose_values['y'],
#         #     pose_values['z'],
#         #     qx, qy, qz, qw
#         # ]))
#         pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(pose_values))

#     if 'targets' not in data:
#         raise KeyError("The key 'targets' is missing from the YAML file.")
#     grid_dict = {}
#     if "Grids" in data:
#         for grid_name, grid_values in data['Grids'].items():
#             grid_dict[grid_name] = GridConfig.from_dict(grid_values)
#     # target_pose = []

#     # for i in data["targets"]:
#     #     if i == "grasp":
#     #         target_pose.append(True)
#     #     elif i == "release":
#     #         target_pose.append(False)
#     #     elif type(i) == float:
#     #         target_pose.append(i)
#     #     else:
#     #         for j in pose_dict:
#     #             if j == i:
#     #                 target_pose.append(pose_dict[j])

#     return pose_dict,data["targets"],grid_dict

import os
import yaml
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.common import PoseConfig
from ras_common.config.loaders.lab_setup import LabSetup
def validate_pose_values(pose_values):
    """
    Validate pose values against robot-specific constraints.
    Args:
        pose_values (dict): Dictionary containing pose values with keys 'x', 'y', 'z', 'roll', 'pitch', 'yaw'
    Raises:
        ValueError: If any pose value is out of range
    """
    # Initialize LabSetup if not already initialized
    LabSetup.init()
    # Get the robot constraints
    if LabSetup.constraints is None:
        raise ValueError(f"No constraints found for robot {LabSetup.robot_name}")
    workspace_constraints = LabSetup.constraints.workspace
    orientation_constraints = LabSetup.constraints.orientation
    for key, value in pose_values.items():
        if key in workspace_constraints:
            min_val, max_val = workspace_constraints[key]
        elif key in orientation_constraints:
            min_val, max_val = orientation_constraints[key]
        else:
            raise ValueError(f"Unknown constraint key: {key}")
        if not (min_val <= value <= max_val):
            raise ValueError(
                f"Invalid value for {key}: {value}. Must be between {min_val} and {max_val} for {LabSetup.robot_name} robot"
            )

def convert_pose_to_meters(pose_values):
    """
    Converts pose values from centimeters to meters.

    Args:
        pose_values (dict): A dictionary containing pose values in centimeters.

    Returns:
        dict: A dictionary with pose values converted to meters.
    """
    return {
        'x': pose_values['x'] / 100,  # Convert cm to meters
        'y': pose_values['y'] / 100,
        'z': pose_values['z'] / 100,
        'roll': pose_values['roll'],  # Assuming roll, pitch, yaw are in radians
        'pitch': pose_values['pitch'],
        'yaw': pose_values['yaw']
    }

def read_yaml_to_pose_dict(path):
    print(f"Reading YAML file: {path}")
    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    if 'Poses' not in data:
        raise KeyError("The key 'poses' is missing from the YAML file.")
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():

        # Convert pose values from centimeters to meters
        pose_values = convert_pose_to_meters(pose_values)

        validate_pose_values(pose_values)
        pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(pose_values))
    if 'targets' not in data:
        raise KeyError("The key 'targets' is missing from the YAML file.")
    target_pose = []
    for action in data["targets"]:
        if isinstance(action, dict):
            key = list(action.keys())[0]
            value = action[key]
            if key == "move2pose":
                if value in pose_dict:
                    target_pose.append({key: value})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "gripper":
                target_pose.append({key: bool(value)})
            elif key == "rotate":
                target_pose.append({key: float(value)})
            else:
                raise KeyError(f"Unknown target action: {key}")
        else:
            raise ValueError("Invalid format in 'targets'. Each entry must be a dictionary.")
    return pose_dict, target_pose