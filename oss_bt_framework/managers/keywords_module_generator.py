from ..behavior_template.keyword import KeywordInputs
from typing import Dict,Iterable,List
from ..behavior_template.module import BehaviorModuleSequence

class KeywordModuleGenerator(object):
    def __init__(self):
        self.registered_keywords : Dict[str,type[KeywordInputs]] = {}

    def register(self,keyword: Iterable[KeywordInputs]|type[KeywordInputs]):
        if isinstance(keyword,Iterable):
            for k in keyword:
                self.register(k)
        elif isinstance(keyword,type) and issubclass(keyword,KeywordInputs):
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
            