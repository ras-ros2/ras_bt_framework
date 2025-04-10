import os
import yaml
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.common import PoseConfig
from ras_common.config.loaders.lab_setup import LabSetup

def validate_pose_values(pose_values):
    """
    Validate pose values against robot-specific constraints defined in lab_setup.yaml.
    Uses the currently active robot's constraints from LabSetup to validate workspace and orientation limits.

    Args:
        pose_values (dict): Dictionary containing pose values with the following keys:
            - x (float): X coordinate in meters
            - y (float): Y coordinate in meters
            - z (float): Z coordinate in meters
            - roll (float): Roll angle in radians
            - pitch (float): Pitch angle in radians
            - yaw (float): Yaw angle in radians

    Raises:
        ValueError: If any of the following conditions are met:
            - No constraints found for the current robot
            - Unknown constraint key in pose_values
            - Pose value is outside the robot's workspace or orientation limits
            - Missing required keys in pose_values

    Example:
        >>> pose = {
        ...     'x': 0.5, 'y': 0.3, 'z': 0.4,
        ...     'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57
        ... }
        >>> validate_pose_values(pose)  # Will raise ValueError if pose is invalid
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
    """
    Read and parse a YAML experiment file in the updated format (with `steps`).
    Validates poses and parses the sequence of actions.

    Args:
        path (str): Path to the YAML experiment file

    Returns:
        tuple: A tuple containing:
            - pose_dict (dict): Mapping from pose names to PortPoseCfg objects
            - steps (list): List of parsed step dictionaries, e.g.
                {'action': 'Move', 'from': 'pose1', 'to': 'pose2'}

    Raises:
        KeyError: Missing required sections or undefined pose references
        ValueError: Invalid pose values or action formatting
    """
    print(f"Reading YAML file: {path}")

    with open(path, 'r') as file:
        data = yaml.safe_load(file)

    # Validate and load poses
    if 'Poses' not in data:
        raise KeyError("The key 'Poses' is missing from the YAML file.")

    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        pose_values = convert_pose_to_meters(pose_values)
        validate_pose_values(pose_values)
        pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(pose_values))

    # Validate and load steps
    if 'steps' not in data:
        raise KeyError("The key 'steps' is missing from the YAML file.")

    steps = []
    for idx, step in enumerate(data['steps']):
        if not isinstance(step, dict) or 'action' not in step:
            raise ValueError(f"Step {idx} must contain an 'action' field.")

        action = step['action']
        parsed_step = {'action': action}

        if action == 'Move':
            if 'from' not in step or 'to' not in step:
                raise ValueError(f"'Move' step must include 'from' and 'to'. Step: {step}")
            if step['from'] not in pose_dict or step['to'] not in pose_dict:
                raise KeyError(f"Undefined pose in Move step: {step}")
            parsed_step['from'] = step['from']
            parsed_step['to'] = step['to']

        elif action == 'Pick' or action == 'Place':
            if 'above' not in step or 'at' not in step:
                raise ValueError(f"'{action}' step must include 'above' and 'at'. Step: {step}")
            if step['above'] not in pose_dict or step['at'] not in pose_dict:
                raise KeyError(f"Undefined pose in {action} step: {step}")
            parsed_step['above'] = step['above']
            parsed_step['at'] = step['at']

        else:
            raise ValueError(f"Unknown action type: {action}")

        steps.append(parsed_step)

    return pose_dict, steps
