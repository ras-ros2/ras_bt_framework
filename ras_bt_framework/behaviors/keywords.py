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

from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.primitives import MoveToPose,RotateEffector,Trigger, MoveToJointState
from ..behaviors.modules import PickSequence,PlaceSequence
from typing import List,Dict
from ..behavior_utility.grid_parser import GridConfig
from .ports import PortPoseCfg
from copy import deepcopy

class TargetPoseMap(object):
    def __init__(self):
        self.pose_map = {}
    
    def register_pose(self,pose_name,pose):
        self.pose_map[pose_name] = pose
    
    def move2pose_module(self,pose:str):
        if (isinstance(pose,str)):
            if pose in self.pose_map:
                return MoveToPose(i_pose=self.pose_map[pose])
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")
        
    def move2pose_sequence_module(self,poses:List[str]):
        move2pose_sequence = BehaviorModuleSequence()
        move2pose_sequence.add_children([self.move2pose_module(pose) for pose in poses])
        return move2pose_sequence

    def pick_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        if pose in self.pose_map:
            return PickSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")
    
    def place_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        if pose in self.pose_map:
            return PlaceSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")
        
    

class GridLocationMap(object):
    def __init__(self,stack_height):
        self.grid_pose_map : Dict[str,TargetPoseMap] = {}
        self.stack_height = stack_height
    
    def register_grid(self,grid_name,grid:GridConfig):
        pose_map = TargetPoseMap()
        
        for location_name,location in grid.locations.items():
            pose_config = deepcopy(grid.pose)
            pose_config.x += location.x
            pose_config.y += location.y
            pose = PortPoseCfg(pose=pose_config)
            pose_map.register_pose(location_name,pose)
        self.grid_pose_map[grid_name] = pose_map
    
    def move2location(self,grid:str,location:str):
        if grid in self.grid_pose_map:
            return self.grid_pose_map[grid].move2pose_module(location)
        else:
            raise ValueError(f"Invalid grid name {grid}")
    def move2location_sequence(self,grid:str,locations:List[str]):
        if grid in self.grid_pose_map:
            return self.grid_pose_map[grid].move2pose_sequence_module(locations)
        else:
            raise ValueError(f"Invalid grid name {grid}")
    
    def pick_location(self,grid:str,location:str,level:int=0,clearance:float=0.07,height:float=0.07):
        if grid in self.grid_pose_map:
            pose_map = self.grid_pose_map[grid].pose_map
            if location in pose_map:
                pose : PortPoseCfg = deepcopy(pose_map[location])
                pose.pose.z += level*self.stack_height
                return PickSequence(pose,clearance=clearance,height=height)
            else:
                raise ValueError(f"Invalid location name {location}")
        else:
            raise ValueError(f"Invalid grid name {grid}")
    
    def place_location(self,grid:str,location:str,level:int=0,clearance:float=0.07):
        if grid in self.grid_pose_map:
            pose_map = self.grid_pose_map[grid].pose_map
            if location in pose_map:
                pose : PortPoseCfg = deepcopy(pose_map[location])
                pose.pose.z += level*self.stack_height
                return PlaceSequence(pose,clearance=clearance)
            else:
                raise ValueError(f"Invalid location name {location}")
        else:
            raise ValueError(f"Invalid grid name {grid}")

def rotate(angle:float):
    return RotateEffector(i_rotation_angle=angle)

def gripper(open:bool):
    return Trigger(i_trigger=open)

# def home_joint_state():
#     from ras_common.config.loaders.lab_setup import LabSetup
#     LabSetup.init()
#     return MoveToJointState(i_joint_names=','.join(map(str,LabSetup.conf.robot.home_joint_state.keys())),i_joint_values=','.join(map(str,LabSetup.conf.robot.home_joint_state.values())))

def joint_state(joints:list):
    from ras_common.config.loaders.lab_setup import LabSetup
    LabSetup.init()
    joint_names = list(LabSetup.conf.robot.home_joint_state.keys())
    if len(joints) != len(joint_names): 
        raise ValueError(f"Invalid number of joints {len(joints)}")
    joint_state = ",".join([f"{joint_names[i]}:{joints[i]}" for i in range(len(joints))])
    return MoveToJointState(i_joint_state=joint_state)

# def pick_object(object_name:str,dst:str):
#     src = TargetPoseMap().pose_map[object_name]
#     return BehaviorModuleSequence(children=[MoveToPose(input_ports={"pose":src}),
#                                             Trigger(input_ports={"trigger":"True"}),
#                                             MoveToPose(input_ports={"pose":dst}),
#                                             Trigger(input_ports={"trigger":"False"})])

keyword_mapping = {
            "rotate":rotate,
            "gripper":gripper,
            'joint_state': joint_state
            }