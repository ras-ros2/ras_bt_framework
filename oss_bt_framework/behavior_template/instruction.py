from dataclasses import dataclass,field
from typing import OrderedDict,Callable,Set,ClassVar
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


class FunctionalInstruction(BehaviorInstructionBase,Callable,ABC):
    
    @abstractmethod
    def execute(self, **kwargs):
        pass
    
    def __call__(self, *args, **kwargs):
        return self.execute(kwargs)

class DynamicInstruction(BehaviorInstructionBase):
    def __init__(self,func:Callable):
        assert isinstance(func,Callable)
        sig = inspect.signature(func)
        param_kw = OrderedDict()
        for name, param in sig.parameters.items():
            if param.default != inspect.Parameter.empty:
                param_kw[name] = param.annotation
            else:
                param_kw[name] = None
        super().__init__(InstructionParams(param_kw))
        self.function = func

    def execute(self,**kwargs):
        self.arguments.update_params(kwargs)
        if not self.arguments.verify_call():
            raise ValueError("Invalid arguments for instruction")
        return self.function(**self.arguments.params)

class EmptyInstruction(BehaviorInstructionBase):
    def __init__(self):
        super().__init__(InstructionParams(OrderedDict()))
    
    def execute(self, **kwargs):
        return
# EmptyInstruction = EmptyInstruction()

@dataclass   
class PrimitiveInstruction(BehaviorInstructionBase,ABC):
    name: str = field(default=None)

    def execute(self, **kwargs):
        raise Exception("Invalid call for primitives.")

@dataclass
class ScriptInstruction(BehaviorInstructionBase):
    code: str

    def execute(self, **kwargs):
        raise Exception("Invalid call for scripts.")
