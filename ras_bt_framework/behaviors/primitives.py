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
from dataclasses import dataclass
from typing import ClassVar,Set
from geometry_msgs.msg import Pose

@dataclass
class ActionInstruction(PrimitiveInstruction):
    name: str

@dataclass
class SaySomething(PrimitiveInstruction):
    name:str
    input_port_names: ClassVar[Set[str]] = {"message"}

@dataclass
class ThinkSomethingToSay(PrimitiveInstruction):
    name:str
    input_port_names: ClassVar[Set[str]] = {"reference"}
    output_port_names: ClassVar[Set[str]] = {"message"}

@dataclass
class MoveToPose(PrimitiveInstruction):
    name:str
    input_port_names: ClassVar[Set[Pose]] = {"pose"}

@dataclass
class RotateEffector(PrimitiveInstruction):
    name:str
    input_port_names: ClassVar[Set[double]] = {"rotation_angle"}

@dataclass
class Trigger(PrimitiveInstruction):
    name:str
    input_port_names: ClassVar[Set[bool]] = {"trigger"}