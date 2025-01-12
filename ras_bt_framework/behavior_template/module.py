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

from  dataclasses import dataclass, field
from py_trees.behaviour import Behaviour
from py_trees.composites import Composite,Sequence
from abc import ABC,abstractmethod
from typing import List,ClassVar,Set,Dict
import re
from geometry_msgs.msg import Pose

class Port(ABC):
    @abstractmethod
    def serialize(self) -> str:
        pass

class PortEntry(Port):
    def __init__(self,name:str):
        self.name = name
        
    def serialize(self):
        return f"{self.name}"

class PortData(Port):
    def serialize(self) -> str:
        pass

class PortString(PortData):
    def __init__(self,v:str):
        self.value = v

    def serialize(self):
        return self.value

class PortPose(PortData):
    def __init__(self,v:Pose):
        self.value = v

    def serialize(self):
        return f"Position: (x={self.value.position.x}, y={self.value.position.y}, z={self.value.position.z}), Orientation: (x={self.value.orientation.x}, y={self.value.orientation.y}, z={self.value.orientation.z}, w={self.value.orientation.w})"

@dataclass
class BehaviorBase(Behaviour,ABC):
    type_name: ClassVar[str] = field(default=None)

    @classmethod
    def get_type_info(cls):
        if cls.type_name is None:
            return cls.__name__
        return cls.type_name
    
    def update(self):
        return


@dataclass(kw_only=True)
class BehaviorModule(BehaviorBase,ABC):
    # input_port_names: ClassVar[Set[str]] = field(default=set())
    # output_port_names: ClassVar[Set[str]] = field(default=set())
    # input_ports: Dict[str,str] = field(default_factory=dict)
    # output_ports: Dict[str,str] = field(default_factory=dict)

    @staticmethod
    def _check_ports(decl_set:set,def_dict:dict):
            def_vals = def_dict.values()
            for def_val in def_vals:
                if not isinstance(def_val,Port):
                    raise ValueError(f"Invalid port type: {type(def_val)}")
            def_set = set(def_dict.keys())
            if not def_set.issuperset(decl_set):
                raise ValueError(f"Missing ports: {decl_set - def_set}")
            undecl_keys = def_set - decl_set
            if len(undecl_keys) > 0:
                print(f"WARNING: Undeclared ports: {undecl_keys}")
                for key in undecl_keys:
                    del def_dict[key]
            

    def __post_init__(self):
        self._input_port_names = set()
        self._output_port_names = set()
        self._input_ports = {}
        self._output_ports = {}

        for _k in self.__dict__.keys():
            if _k.startswith("i_"):
                self._input_port_names.add(_k[2:])
            elif _k.startswith("o_"):
                self._output_port_names.add(_k[2:])

        if len(self._input_port_names.intersection(self._output_port_names)) > 0:
            raise ValueError(f"Duplicate ports: {self._input_port_names.intersection(self._output_port_names)}")
        for _k, _v in self.__dict__.items():
            if _k.startswith("i_"):
                self._input_ports[_k[2:]] = _v
            elif _k.startswith("o_"):
                self._output_ports[_k[2:]] = _v
        self._check_ports(self._input_port_names, self._input_ports)
        self._check_ports(self._output_port_names, self._output_ports)

    @staticmethod
    def deserialize_ports(ports):
        deserialized = {}
        for key, value in ports.items():
            if hasattr(value, 'serialize') and callable(getattr(value, 'serialize')):
                deserialized[key] = value.serialize()
            else:
                raise AttributeError(f"The object for key '{key}' does not have a callable 'serialize' method.")
        return deserialized


    def get_port_map(self):
        return {**self.deserialize_ports(self._input_ports),**self.deserialize_ports(self._output_ports)}

@dataclass
class BehaviorModuleCollection(Composite,BehaviorModule):
    # output_port_values : Dict[str,str] = field(default_factory=dict)
    # children: List[BehaviorModule] = field(default_factory=list)
    # out_children: List[BehaviorModule] = field(default_factory=list,init=False)

    # def __post_init__(self):
    #     super().__post_init__()
    #     from .instruction import ScriptInstruction
    #     ref_set = set(self.output_port_values.keys())
    #     decl_set = set(self.output_port_names)
    #     if not decl_set.issuperset(ref_set):
    #         raise ValueError(f"Missing ports values: {ref_set - decl_set}")
    #     undecl_keys = ref_set - decl_set
    #     if len(undecl_keys) > 0:
    #         print(f"WARNING: Undeclared ports: {undecl_keys}")
    #         for key in undecl_keys:
    #             del self.output_port_values[key]
    #     pattern = r"^\{([a-zA-Z_][a-zA-Z0-9_]*)\}$"
        
    #     for port_name in self.output_port_names:
    #         value = self.output_port_values[port_name]
    #         if isinstance(value,str):
    #             key_match = re.fullmatch(pattern,value)
    #             if isinstance(key_match,re.Match):
    #                 value = key_match.group(1)
    #         else:
    #             raise ValueError(f"Invalid output port value: {value}")
    #         self.out_children.append(ScriptInstruction(code=f" {port_name}:={value} "))

    def add_children(self, children: List[BehaviorModule]):
        if isinstance(children, list):
            for child in children:
                self.add_child(child)
        elif isinstance(children, BehaviorModule):
            self.children.append(children)
        else:
            raise ValueError(f"Invalid type of children: {type(children)}")

    def get_port_map(self):
        return {**self.input_ports,**self.output_ports}

    def iterate(self):
        for child in self.children:
            yield child
        for child in self.out_children:
            yield child
    
    def add_children(self, children:List[BehaviorModule]|BehaviorModule):
        if isinstance(children,list):
            for child in children:
                self.add_children(child)
        elif isinstance(children,BehaviorModule):
            children.uid = children.get_type_info() + str(len(self.children))      
            self.children.append(children)
        else:
            raise ValueError(f"Invalid children type: {type(children)}")
            
        

@dataclass
class BehaviorModuleSequence(Sequence,BehaviorModuleCollection):
    pass