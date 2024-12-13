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

from .module import BehaviorModule,BehaviorModuleSequence
from .instruction import PrimitiveInstruction,FunctionalInstruction

class KeywordInput(BehaviorModule):
    def __init__(self, **kwargs):
        super().__init__()
        if len(self.output_port_names) != 0:
            raise ValueError("KeywordInputs has no output ports")
        self._check_ports(self.input_port_names,kwargs)
        self.input_ports = kwargs

# class KeywordPrimitive(PrimitiveInstruction,KeywordInput):
#     def __init__(self, **kwargs):
#         KeywordInput.__init__(self,**kwargs)
#         PrimitiveInstruction.__init__(self)

# class KeywordFunction(FunctionalInstruction,KeywordInput):
#     def __init__(self, **kwargs):
#         KeywordInput.__init__(self,**kwargs)
#         FunctionalInstruction.__init__(self)

# class KeywordModuleSequence(BehaviorModuleSequence,KeywordInput):
#     def __init__(self, **kwargs):
#         KeywordInput.__init__(self,**kwargs)
#         BehaviorModuleSequence.__init__(self)
