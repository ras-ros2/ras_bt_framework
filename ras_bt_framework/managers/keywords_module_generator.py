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
from typing import Dict,Iterable,List,Callable
from ..behavior_template.module import BehaviorModuleSequence,BehaviorModule


class KeywordModuleGenerator(object):
    def __init__(self):
        self.registered_keywords : Dict[str,type[BehaviorModule]] = {}

    def register(self,keyword: Callable|Dict[str,Callable] , name:str=None):
        if isinstance(keyword,dict):
            for _n,_k in keyword.items():
                self.register(_k,_n)
        elif callable(keyword):
            if not isinstance(name,str):
                name = keyword.__name__
            self.registered_keywords[name] = keyword
        else:
            raise ValueError(f"Invalid keyword type: {keyword} {type(keyword)}")

    def generate(self,name,keyword_seq:List[Dict[str,dict]]):
        behavior_module : BehaviorModuleSequence= type(name,(BehaviorModuleSequence,),{})()
        for _kw in keyword_seq:
            identifier,params = list(_kw.items())[0]
            if identifier not in self.registered_keywords:
                raise ValueError(f"Unknown keyword: {identifier}")
            if (not isinstance(params,Iterable)) or (isinstance(params,str)) :
                keyword_module = self.registered_keywords[identifier](params)
                behavior_module.children.append(keyword_module)
            elif isinstance(params,Iterable):
                if not isinstance(params,dict):
                    keyword_module = self.registered_keywords[identifier](*params)
                    behavior_module.children.append(keyword_module)
                else:
                    keyword_module = self.registered_keywords[identifier](**params)
                    behavior_module.children.append(keyword_module)
            else:
                raise ValueError(f"Invalid keyword parameters: {params}")
        return behavior_module
        
            