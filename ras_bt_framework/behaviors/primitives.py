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

from numpy import double
from ..behavior_template.instruction import PrimitiveInstruction
from ..behavior_template.port import PortStr,PortData
from dataclasses import dataclass
from typing import ClassVar,Set
from geometry_msgs.msg import Pose

@dataclass
class PortPose(PortData):
    value: Pose

    def serialize(self):
        return self.value

@dataclass
class PortDouble(PortData):
    value: double

    def serialize(self):
        return PortData.default_serialize(self.value)

@dataclass
class PortBool(PortData):
    value: bool

    def serialize(self):
        return PortData.default_serialize(self.value)
    
@dataclass
class PortInt(PortData):
    value: int

    def serialize(self):
        return PortData.default_serialize(self.value)

@dataclass
class ActionInstruction(PrimitiveInstruction):
    name: str

@dataclass
class SaySomething(PrimitiveInstruction):
    name:str
    i_message: PortStr

@dataclass
class ThinkSomethingToSay(PrimitiveInstruction):
    name:str
    i_reference: PortStr
    o_message: PortStr

@dataclass
class MoveToPose(PrimitiveInstruction):
    name:str
    i_pose: PortPose

@dataclass
class RotateEffector(PrimitiveInstruction):
    name:str
    i_rotation_angle: PortDouble

@dataclass
class Trigger(PrimitiveInstruction):
    name:str
    i_trigger: PortBool

@dataclass
class ExecuteTrajectory(PrimitiveInstruction):
    name:str
    i_sequence: PortInt

@dataclass
class LoggerClientTrigger(PrimitiveInstruction):
    name:str
