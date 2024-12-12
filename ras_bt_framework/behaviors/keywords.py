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

from..behavior_template.keyword import KeywordPrimitives
from ..behaviors.primitives import MoveToPose,RotateEffector
from tf_transformations import quaternion_from_euler

class MoveToPose(KeywordPrimitives):
    input_port_names = {"pose"}
    def __init__(self,pose_dict):
        qx, qy, qz, qw = quaternion_from_euler(pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'])
        pose_value  = ",".join(map(str, (
            pose_dict['x'],
            pose_dict['y'],
            pose_dict['z'],
            qx, qy, qz, qw
        )))
        super().__init__(pose=pose_value)

class RotateEffector(KeywordPrimitives):
    input_port_names = {"rotation_angle"}
    def __init__(self,rotation_angle):
        super().__init__(rotation_angle=rotation_angle)



