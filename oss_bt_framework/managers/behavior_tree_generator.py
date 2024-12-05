from ..behavior_template.module import BehaviorModule
from ..behavior_template.module import BehaviorModuleSequence,BehaviorModuleCollection
from ..behavior_template.instruction import EmptyInstruction,PrimitiveInstruction,FunctionalInstruction,ScriptInstruction
from typing import Iterable,List,Dict,Set
from dataclasses import dataclass,field
from oss_common.xml_utils.behavior_tree_gen import BTXml,ElementTree
from ..managers.primitive_action_manager import PrimitiveActionManager


__registered_primitives : Set[type[PrimitiveInstruction]] = {}

@dataclass
class BehaviorTreeGenerator(object):
    root_behavior: BehaviorModuleSequence = field(init=False,default=None)
    action_manager: PrimitiveActionManager

    def feed_root(self,behavior:type[BehaviorModule]|type[BehaviorModuleSequence]):
       self.root_behavior = behavior
    
    @staticmethod
    def is_valid_primitive(primitive:type[PrimitiveInstruction]|PrimitiveInstruction|str):
        if isinstance(primitive,str):
            for _p in __registered_primitives:
                if _p.get_type_info() == primitive:
                    return True
            return False
        elif isinstance(primitive,type):
            if issubclass(primitive,PrimitiveInstruction):
                return True
            else:
                return False
        else:
            return isinstance(primitive,PrimitiveInstruction)
        
    def verify_sanity(self):
        behavior_stack = []
        def _recursive_check(behavior:BehaviorModule|BehaviorModuleSequence):
            if isinstance(behavior,BehaviorModuleSequence):
                type_info = behavior.get_type_info()
                if type_info not in behavior_stack:
                    behavior_stack.append(type_info)
                else:
                    raise ValueError(f"Behavior {type_info} is already in the stack.{behavior_stack}.Recursive behaviors are not allowed.")
                for child in behavior.iterate():
                    _recursive_check(child)
                    behavior_stack.pop()
            elif isinstance(behavior,BehaviorModule):
                type_info = behavior.get_type_info()
                if type_info not in behavior_stack:
                    behavior_stack.append(type_info)
                else:
                    raise ValueError(f"Behavior {type_info} is already in the stack.{behavior_stack}.Recursive behaviors are not allowed.")
            else:
                raise ValueError(f"Behavior {type(behavior)} is not an instruction or a module.")
        
        _recursive_check(self.root_behavior)


    def generate_xml_trees(self,file_path:str):
        self.verify_sanity()
        tree_gen = BTXml()
        def  _iterate_tree(behavior:BehaviorModule|BehaviorModuleSequence,parent_elem:ElementTree):
            if isinstance(behavior,BehaviorModuleCollection):
                new_tree,subtree = tree_gen.new_subtree(behavior.get_type_info(),parent_elem,behavior.get_port_map())
                collection_elem = None
                if isinstance(behavior,BehaviorModuleSequence):
                    collection_elem = tree_gen.add_sequence(new_tree)
                else:
                    raise ValueError(f"Behavior Collection {type(behavior)} is not implemented.")
                for child in behavior.iterate():
                    _iterate_tree(child,collection_elem)
            elif isinstance(behavior,BehaviorModule):
                if isinstance(behavior,PrimitiveInstruction):
                    if self.is_valid_primitive(behavior):
                        tree_gen.add_primitive_node(parent_elem,behavior.get_type_info(),behavior.name,behavior.get_port_map())
                    else:
                        raise ValueError(f"Primitive instruction {type(behavior)} is not in the list of primitives.")
                elif isinstance(behavior,FunctionalInstruction):
                    tree_gen.add_primitive_node(parent_elem,behavior.get_type_info(),behavior.name,behavior.get_port_map())
                elif isinstance(behavior,ScriptInstruction):
                    tree_gen.add_script(parent_elem,code=behavior.code)
                else:
                    raise ValueError(f"Behavior {type(behavior)} is not a primitive or functional instruction.")
        _iterate_tree(self.root_behavior,tree_gen.main_tree)
        
        tree_gen.dump_xml(file_path)
                