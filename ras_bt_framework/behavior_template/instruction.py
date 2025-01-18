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

from dataclasses import dataclass,field
from typing import OrderedDict,Callable,Set,ClassVar
from copy import deepcopy
import inspect
from abc import ABC,abstractmethod
from .module import BehaviorModule,BehaviorModuleSequence

@dataclass
class InstructionParams(object):
    signature_keywords: OrderedDict[str,type]
    params: dict = field(default_factory=dict)

    def __post_init__(self):
        self.check_params()

    def check_params(self):
        sig_kw = set(self.signature_keywords.keys())
        params_set = set(self.params.keys())
        unused_params = params_set-sig_kw
        if len(unused_params)>0:
            print("WARN: Discarding unused params: ",unused_params)
            for _key in unused_params:
                del self.params[_key]

    def update_params(self,**kwargs):
        self.params.update(kwargs)
        self.check_params()
    
    def verify_call(self):
        self.check_params()
        sig_kw = set(self.signature_keywords.keys())
        params_set = set(self.params.keys())
        return sig_kw.issuperset(params_set)
        

@dataclass(kw_only=True)
class BehaviorInstructionBase(BehaviorModule,ABC):
    arguments: InstructionParams = field(default=None)
    
    @abstractmethod
    def execute(self,**kwargs):
        pass

@dataclass
class GenerativeInstruction(BehaviorModule, ABC):
    input_ports: ClassVar[OrderedDict[str,type]] = OrderedDict()
    output_ports: ClassVar[OrderedDict[str,type]] = OrderedDict()
    @abstractmethod
    def tick(self):
        pass


class EmptyInstruction(BehaviorInstructionBase):
    def __init__(self):
        super().__init__(InstructionParams(OrderedDict()))
    
    def execute(self, **kwargs):
        return
# EmptyInstruction = EmptyInstruction()

@dataclass(kw_only=True)
class PrimitiveInstruction(BehaviorInstructionBase,ABC):
    def execute(self, **kwargs):
        raise Exception("Invalid call for primitives.")

@dataclass
class ScriptInstruction(BehaviorInstructionBase):
    code: str

    def execute(self, **kwargs):
        raise Exception("Invalid call for scripts.")
