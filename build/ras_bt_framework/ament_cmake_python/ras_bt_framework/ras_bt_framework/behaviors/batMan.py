import os
import yaml
from typing import Dict, List, Optional, Tuple
from ras_bt_framework.managers.pose_map import PortPoseCfg
from ras_bt_framework.behavior_template.node import BehaviorModuleSequence
from ras_common.config.loaders.lab_setup import LabSetup
from ras_common.config.loaders.objects import ObjectTypes
from .behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behaviors.keywords import Move, Pick, Place, gripper, rotate, joint_state, TargetPoseMap

class BatMan:
    """
    A class for managing and executing robot experiments with a focus on pick-and-place tasks.
    
    This class extends the functionality of ExperimentsService to provide specialized
    methods for handling pick-and-place operations and experiment sequences.
    
    Attributes:
        pose_map (TargetPoseMap): Map of registered poses and their configurations
        experiments_dir (str): Directory containing experiment YAML files
        current_experiment (str): Name of the currently loaded experiment
    """
    
    def __init__(self, experiments_dir: str):
        """
        Initialize the BatMan experiment manager.
        
        Args:
            experiments_dir (str): Path to the directory containing experiment YAML files
        """
        self.pose_map = TargetPoseMap()
        self.experiments_dir = experiments_dir
        self.current_experiment = None
        
    def load_experiment(self, experiment_name: str) -> Tuple[Dict[str, PortPoseCfg], List[Dict]]:
        """
        Load an experiment configuration from a YAML file.
        
        Args:
            experiment_name (str): Name of the experiment file (without .yaml extension)
            
        Returns:
            Tuple[Dict[str, PortPoseCfg], List[Dict]]: A tuple containing:
                - Dictionary mapping pose names to their configurations
                - List of target actions to execute
                
        Raises:
            FileNotFoundError: If the experiment file doesn't exist
        """
        experiment_path = os.path.join(self.experiments_dir, f"{experiment_name}.yaml")
        if not os.path.exists(experiment_path):
            raise FileNotFoundError(f"Experiment file not found: {experiment_path}")
            
        pose_dict, target_pose = read_yaml_to_pose_dict(experiment_path)
        
        # Register all poses with the pose map
        for pose_name, pose in pose_dict.items():
            self.pose_map.register_pose(pose_name, pose)
            
        self.current_experiment = experiment_name
        return pose_dict, target_pose
        
    def create_behavior_sequence(self, target_pose: List[Dict]) -> BehaviorModuleSequence:
        """
        Create a behavior sequence from a list of target actions (supports new and legacy formats).
        
        Args:
            target_pose (List[Dict]): List of target actions to execute

        Returns:
            BehaviorModuleSequence: Sequence of behavior modules to execute

        Raises:
            ValueError: If an unknown target action is encountered
        """
        sequence = BehaviorModuleSequence()

        for action in target_pose:
            if not isinstance(action, dict):
                raise ValueError("Invalid format in target_pose. Each entry must be a dictionary.")

            # New format: has an 'action' key
            if "action" in action:
                act = action["action"].lower()

                if act == "move":
                    sequence.add_child(Move(from_pose=action["from"], to_pose=action["to"]))
                elif act == "gripper":
                    sequence.add_child(gripper(action["state"]))
                elif act == "rotate":
                    sequence.add_child(rotate(action["angle"]))
                elif act == "joint_state":
                    sequence.add_child(joint_state(action["values"]))
                elif act == "pick":
                    sequence.add_child(Pick(above=action["above"], at=action["at"]))
                elif act == "place":
                    sequence.add_child(Place(above=action["above"], at=action["at"]))
                else:
                    raise ValueError(f"Unknown action type in YAML: {act}")

            else:
                key = list(action.keys())[0]
                value = action[key]

                if key == "move2pose":
                    # Handle the old move2pose format by converting to Move
                    if isinstance(value, dict) and "from" in value and "to" in value:
                        sequence.add_child(Move(from_pose=value["from"], to_pose=value["to"]))
                    else:
                        # Single pose move2pose
                        sequence.add_child(Move(pose=value))
                elif key == "gripper":
                    sequence.add_child(gripper(value))
                elif key == "rotate":
                    sequence.add_child(rotate(value))
                elif key == "joint_state":
                    sequence.add_child(joint_state(value))
                else:
                    raise ValueError(f"Unknown target action: {key}")

        return sequence

    def execute_experiment(self, experiment_name: str) -> BehaviorModuleSequence:
        """
        Load and create a behavior sequence for an experiment.
        
        Args:
            experiment_name (str): Name of the experiment to execute
            
        Returns:
            BehaviorModuleSequence: Sequence of behavior modules to execute
            
        Raises:
            FileNotFoundError: If the experiment file doesn't exist
            ValueError: If there's an error processing the experiment configuration
        """
        _, target_pose = self.load_experiment(experiment_name)
        return self.create_behavior_sequence(target_pose)
        
    def create_pick_and_place_sequence(self, pick_pose: str, place_pose: str, 
                                     clearance: float = 0.10, height: float = 0.10) -> BehaviorModuleSequence:
        """
        Create a sequence for picking an object from one pose and placing it at another.
        
        Args:
            pick_pose (str): Name of the pose where the object is located
            place_pose (str): Name of the pose where the object should be placed
            clearance (float, optional): Distance to move before/after picking/placing. Defaults to 0.10.
            height (float, optional): Height to lift/lower the object. Defaults to 0.10.
            
        Returns:
            BehaviorModuleSequence: Sequence of behavior modules for pick-and-place operation
            
        Raises:
            ValueError: If either pick_pose or place_pose is invalid
        """
        sequence = BehaviorModuleSequence()
        
        # Add pick sequence
        sequence.add_child(self.pose_map.pick_module(pick_pose, clearance, height))
        
        # Add place sequence
        sequence.add_child(self.pose_map.place_module(place_pose, clearance, height))
        
        return sequence
        
    def get_current_experiment(self) -> Optional[str]:
        """
        Get the name of the currently loaded experiment.
        
        Returns:
            Optional[str]: Name of the current experiment, or None if no experiment is loaded
        """
        return self.current_experiment 