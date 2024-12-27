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
from ..behaviors.primitives import MoveToPose,RotateEffector,Trigger
from typing import List

class TargetPoseMap(object):
    def __init__(self):
        self.pose_map = {}
    
    def register_pose(self,pose_name,pose):
        self.pose_map[pose_name] = pose
    
    def move2pose_module(self,pose:str):
        if (isinstance(pose,str)):
            if pose in self.pose_map:
                return MoveToPose(input_ports={"pose":self.pose_map[pose]})
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")
        
    def move2pose_sequence_module(self,poses:List[str]):
        return BehaviorModuleSequence(children=[self.move2pose_module(pose) for pose in poses])

def rotate(angle:float):
    return RotateEffector(input_ports={"rotation_angle":str(angle)})

def gripper(open:bool):
    return Trigger(input_ports={"trigger":str(open)})

# def pick_object(object_name:str,dst:str):
#     src = TargetPoseMap().pose_map[object_name]
#     return BehaviorModuleSequence(children=[MoveToPose(input_ports={"pose":src}),
#                                             Trigger(input_ports={"trigger":"True"}),
#                                             MoveToPose(input_ports={"pose":dst}),
#                                             Trigger(input_ports={"trigger":"False"})])

keyword_mapping = {
            "rotate":rotate,
            "gripper":gripper,
            }