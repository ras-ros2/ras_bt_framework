from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.primitives import MoveToPose,RotateEffector,Trigger, MoveToJointState
from ..behaviors.modules import PickSequence,PlaceSequence
from typing import List,Dict
from ras_common.config.loaders.objects import ObjectTypes
from .ports import PortPoseCfg
from copy import deepcopy

class TargetPoseMap(object):
    """
    A class to manage and execute robot poses and sequences.
    
    This class provides functionality to register poses and create behavior modules
    for various robot actions including moving to poses, picking, and placing objects.
    
    Attributes:
        pose_map (dict): Dictionary mapping pose names to their corresponding pose configurations
    """
    
    def __init__(self):
        self.pose_map = {}
    
    def register_pose(self,pose_name,pose):
        """
        Register a new pose with the pose map.
        
        Args:
            pose_name (str): Name identifier for the pose
            pose: Pose configuration to be registered
        """
        self.pose_map[pose_name] = pose
    
    def move2pose_module(self,pose:str):
        """
        Create a MoveToPose module for moving to a specific pose.
        
        Args:
            pose (str): Name of the registered pose to move to
            
        Returns:
            MoveToPose: Behavior module for moving to the specified pose
            
        Raises:
            ValueError: If pose name is invalid or pose input type is incorrect
        """
        if (isinstance(pose,str)):
            if pose in self.pose_map:
                return MoveToPose(i_pose=self.pose_map[pose])
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")
        
    def move2pose_sequence_module(self,poses:List[str]):
        """
        Create a sequence of MoveToPose modules for multiple poses.
        
        Args:
            poses (List[str]): List of pose names to move to in sequence
            
        Returns:
            BehaviorModuleSequence: Sequence of move-to-pose behaviors
        """
        move2pose_sequence = BehaviorModuleSequence()
        move2pose_sequence.add_children([self.move2pose_module(pose) for pose in poses])
        return move2pose_sequence

    def pick_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        """
        Create a pick sequence module for grasping objects.
        
        Args:
            pose (str): Name of the pose where the object is located
            clearance (float, optional): Distance to move before/after picking. Defaults to 0.10.
            height (float, optional): Height to lift the object. Defaults to 0.10.
            
        Returns:
            PickSequence: Behavior module for picking sequence
            
        Raises:
            ValueError: If pose name is invalid
        """
        if pose in self.pose_map:
            return PickSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")
    
    def place_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        """
        Create a place sequence module for releasing objects.
        
        Args:
            pose (str): Name of the pose where the object should be placed
            clearance (float, optional): Distance to move before/after placing. Defaults to 0.10.
            height (float, optional): Height to lower the object. Defaults to 0.10.
            
        Returns:
            PlaceSequence: Behavior module for placing sequence
            
        Raises:
            ValueError: If pose name is invalid
        """
        if pose in self.pose_map:
            return PlaceSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")

def rotate(angle:float):
    """
    Create a rotation behavior module.
    
    Args:
        angle (float): Angle to rotate by in radians
        
    Returns:
        RotateEffector: Behavior module for rotating the end effector
    """
    return RotateEffector(i_rotation_angle=angle)

def gripper(open:bool):
    """
    Create a gripper control behavior module.
    
    Args:
        open (bool): True to open gripper, False to close gripper
        
    Returns:
        Trigger: Behavior module for controlling the gripper
    """
    return Trigger(i_trigger=open)

def joint_state(joints:list):
    """
    Create a behavior module to move to a specific joint configuration.
    
    Args:
        joints (list): List of joint angles in radians
        
    Returns:
        MoveToJointState: Behavior module for moving to the specified joint configuration
        
    Raises:
        ValueError: If the number of joints doesn't match the robot's configuration
    """
    from ras_common.config.loaders.lab_setup import LabSetup
    LabSetup.init()
    joint_names = list(LabSetup.conf.robot.home_joint_state.keys())
    if len(joints) != len(joint_names): 
        raise ValueError(f"Invalid number of joints {len(joints)}")
    joint_state = ",".join([f"{joint_names[i]}:{joints[i]}" for i in range(len(joints))])
    return MoveToJointState(i_joint_state=joint_state)

# New methods for the updated YAML format
def Move(pose=None, from_pose=None, to_pose=None, **kwargs):
    """
    Create a movement behavior module from one pose to another.
    
    Args:
        pose (str, optional): Name of the pose to move to (legacy parameter)
        from_pose (str, optional): Starting pose
        to_pose (str, optional): Target pose
        **kwargs: Backward compatibility parameters:
            from (str): Same as from_pose
            to (str): Same as to_pose
        
    Returns:
        BehaviorModuleSequence: Sequence of behaviors for moving to a pose
        
    Raises:
        ValueError: If neither 'pose' nor both 'from_pose'/'from' and 'to_pose'/'to' are specified
    """
    # Support legacy 'from' and 'to' parameter names
    from_val = from_pose
    to_val = to_pose
    
    # Check if we're using the legacy format with 'from' and 'to' in kwargs
    if 'from' in kwargs:
        from_val = kwargs['from']
    if 'to' in kwargs:
        to_val = kwargs['to']
    
    # Add debug information
    print(f"Move action parameters: pose={pose}, from={from_val}, to={to_val}")
    
    # Check if we're using the format with from and to values
    if from_val is not None and to_val is not None:
        # Ensure the poses exist in the target_pose_map
        if from_val not in target_pose_map.pose_map:
            raise ValueError(f"Invalid 'from' pose: {from_val}. Pose not found in target_pose_map.")
        if to_val not in target_pose_map.pose_map:
            raise ValueError(f"Invalid 'to' pose: {to_val}. Pose not found in target_pose_map.")
        
        move_sequence = BehaviorModuleSequence()
        # Move from the starting pose to the target pose
        move_sequence.add_children([
            target_pose_map.move2pose_module(from_val),
            target_pose_map.move2pose_module(to_val)
        ])
        
        return move_sequence
    
    # Legacy support for just 'pose'
    elif pose is not None:
        # Ensure the pose exists in the target_pose_map
        if pose not in target_pose_map.pose_map:
            raise ValueError(f"Invalid 'pose': {pose}. Pose not found in target_pose_map.")
        
        move_sequence = BehaviorModuleSequence()
        # Move to the target pose
        move_sequence.add_children([target_pose_map.move2pose_module(pose)])
        
        return move_sequence
    
    else:
        raise ValueError("Either 'pose' or both 'from'/'from_pose' and 'to'/'to_pose' must be specified for Move action")

def Pick(above=None, at=None, **kwargs):
    """
    Create a pick behavior module for picking an object.
    
    Args:
        above (str): Name of the pose above the object
        at (str): Name of the pose at the object
        **kwargs: Additional backward compatibility parameters
        
    Returns:
        BehaviorModuleSequence: Sequence of behaviors for picking an object
        
    Raises:
        ValueError: If required parameters are missing or pose names are invalid
    """
    # Handle different parameter formats
    above_val = above
    at_val = at
    
    # Check for parameters in kwargs for backward compatibility
    if above is None and 'above' in kwargs:
        above_val = kwargs['above']
    if at is None and 'at' in kwargs:
        at_val = kwargs['at']
    
    # Add debug information
    print(f"Pick action parameters: above={above_val}, at={at_val}")
    
    if above_val is None or at_val is None:
        raise ValueError("Both 'above' and 'at' must be specified for Pick action")
    
    # Ensure the poses exist in the target_pose_map
    if above_val not in target_pose_map.pose_map:
        raise ValueError(f"Invalid 'above' pose: {above_val}. Pose not found in target_pose_map.")
    if at_val not in target_pose_map.pose_map:
        raise ValueError(f"Invalid 'at' pose: {at_val}. Pose not found in target_pose_map.")
    
    pick_sequence = BehaviorModuleSequence()
    # Move to the pose above the object
    pick_sequence.add_children([
        target_pose_map.move2pose_module(above_val),
        # Open the gripper
        gripper(True),
        # Move to the object
        target_pose_map.move2pose_module(at_val),
        # Close the gripper
        gripper(False),
        # Move back to the pose above the object
        target_pose_map.move2pose_module(above_val)
    ])
    
    return pick_sequence

def Place(above=None, at=None, **kwargs):
    """
    Create a place behavior module for placing an object.
    
    Args:
        above (str): Name of the pose above the target location
        at (str): Name of the pose at the target location
        **kwargs: Additional backward compatibility parameters
        
    Returns:
        BehaviorModuleSequence: Sequence of behaviors for placing an object
        
    Raises:
        ValueError: If required parameters are missing or pose names are invalid
    """
    # Handle different parameter formats
    above_val = above
    at_val = at
    
    # Check for parameters in kwargs for backward compatibility
    if above is None and 'above' in kwargs:
        above_val = kwargs['above']
    if at is None and 'at' in kwargs:
        at_val = kwargs['at']
    
    # Add debug information
    print(f"Place action parameters: above={above_val}, at={at_val}")
    
    if above_val is None or at_val is None:
        raise ValueError("Both 'above' and 'at' must be specified for Place action")
    
    # Ensure the poses exist in the target_pose_map
    if above_val not in target_pose_map.pose_map:
        raise ValueError(f"Invalid 'above' pose: {above_val}. Pose not found in target_pose_map.")
    if at_val not in target_pose_map.pose_map:
        raise ValueError(f"Invalid 'at' pose: {at_val}. Pose not found in target_pose_map.")
    
    place_sequence = BehaviorModuleSequence()
    # Move to the pose above the target location
    place_sequence.add_children([
        target_pose_map.move2pose_module(above_val),
        # Move to the target location
        target_pose_map.move2pose_module(at_val),
        # Open the gripper
        gripper(True),
        # Move back to the pose above the target location
        target_pose_map.move2pose_module(above_val)
    ])
    
    return place_sequence

# Create a global instance of TargetPoseMap to be used by the action functions
target_pose_map = TargetPoseMap()

# Mapping of keyword functions to their implementations
keyword_mapping = {
    "rotate": rotate,
    "gripper": gripper,
    'joint_state': joint_state,
    'Move': Move,
    'Pick': Pick,
    'Place': Place,
    'move2pose': target_pose_map.move2pose_module  # Keep for backward compatibility
}