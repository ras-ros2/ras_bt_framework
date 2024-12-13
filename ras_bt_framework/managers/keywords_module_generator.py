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

from ..behavior_template.keyword import KeywordInput
from typing import Dict,Iterable,List
from ..behavior_template.module import BehaviorModuleSequence

class KeywordModuleGenerator(object):
    def __init__(self):
        self.registered_keywords : Dict[str,type[KeywordInput]] = {}

    def register(self,keyword: Iterable[KeywordInput]|type[KeywordInput]):
        if isinstance(keyword,Iterable):
            for k in keyword:
                self.register(k)
        elif isinstance(keyword,type) and issubclass(keyword,KeywordInput):
            kw_typename = keyword.get_type_info()
            if kw_typename in self.registered_keywords:
                raise ValueError(f"Duplicate keyword: {kw_typename}")
            self.registered_keywords[kw_typename] = keyword
        else:
            raise ValueError(f"Invalid keyword type: {keyword}")

    def generate(self,name,keyword_map:List[Dict[str,dict]]):
        behavior_module : BehaviorModuleSequence= type(name,(BehaviorModuleSequence,),{})()
        for identifier,params in keyword_map:
            if identifier not in self.registered_keywords:
                raise ValueError(f"Unknown keyword: {identifier}")
            keyword_module = self.registered_keywords[identifier]
            