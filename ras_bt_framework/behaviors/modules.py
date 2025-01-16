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
from .primitives import RotateEffector, SaySomething,ThinkSomethingToSay,MoveToPose,Trigger

from ..behavior_template.module import BehaviorModuleSequence
from geometry_msgs.msg import Pose

from ..behavior_utility.yaml_parser import read_yaml_to_pose_dict

from ament_index_python import get_package_share_directory
from ..behavior_template.port import PortStr,PortData,PortEntry,RefPortEntry

class SaySomethingSequence(BehaviorModuleSequence):
    def __init__(self,message_entry):
        self.o_message = RefPortEntry(message_entry,"out_message")
        super().__init__()
        self.add_children([
            ThinkSomethingToSay(i_reference="Hello World!",o_message="{out_message}"),
            ])
        
class MyCustomSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.add_children([
            SaySomethingSequence(message_entry="{my_message}"),
            SaySomething(i_message="{my_message}"),
            ])

class PickObject(BehaviorModuleSequence):
    def __init__(self, sequence_list):
        super().__init__()
        self.add_children(sequence_list)

class RotateEffectorSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.add_children([
            RotateEffector(i_rotation_angle=1.8),
            ])
    
class PressButton(BehaviorModuleSequence):
    def __init__(self,pose:Pose,travel_length):
        super().__init__()
        import numpy as np
        quaternion = np.array(quaternion)
        quaternion /= np.linalg.norm(quaternion)
        from scipy.spatial.transform import Rotation as R
        
        rot = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        
        rotated_vector = rot.apply(np.array([1, 0, 0]))
        
        rotated_vector / np.linalg.norm(rotated_vector)
        
        pressed_pose = Pose()
        pressed_pose.position.x = pose.position.x + rotated_vector[0] * travel_length
        pressed_pose.position.y = pose.position.y + rotated_vector[1] * travel_length
        pressed_pose.position.z = pose.position.z + rotated_vector[2] * travel_length
        pressed_pose.orientation = pose.orientation

        self.add_children([
            MoveToPose(i_pose=",".join(map(str, pose))),
            MoveToPose(i_pose=",".join(map(str, pressed_pose))),
            ])
