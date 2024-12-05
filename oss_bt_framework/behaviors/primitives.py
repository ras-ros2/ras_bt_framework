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