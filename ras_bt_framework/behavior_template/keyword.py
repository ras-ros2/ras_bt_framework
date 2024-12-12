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
