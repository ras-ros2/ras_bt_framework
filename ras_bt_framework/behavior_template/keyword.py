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
    def __init__(self, kw_params):
        super().__init__()
        if len(self.output_port_names) != 0:
            raise ValueError("KeywordInputs has no output ports")
        self._check_ports(self.input_port_names,kw_params)
        self.input_ports = kw_params

def as_keyword_input(behavior_type):
    assert issubclass(behavior_type, BehaviorModule)
    assert not issubclass(behavior_type, KeywordInput)
    new_name = behavior_type.__name__
    new_class = type(new_name, (behavior_type, KeywordInput), dict())
    def new_init(self, kw_params):
        super(new_class, self).__init__()
        KeywordInput.__init__(self, kw_params)
    new_class.__init__ = new_init
    return new_class
