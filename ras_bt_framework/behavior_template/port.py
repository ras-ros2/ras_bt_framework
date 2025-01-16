from dataclasses import dataclass
from abc import ABC, abstractmethod

@dataclass
class Port(ABC):
    @abstractmethod
    def serialize(self) -> str:
        pass

@dataclass
class PortEntry(Port):
    name: str
        
    def serialize(self):
        return f"{self.name}"

@dataclass
class PortData(Port):
    @staticmethod
    def default_serialize(value):
        return f"{value}"

@dataclass
class PortString(PortData):
    value: str

    def serialize(self):
        return self.value
    
@dataclass
class RefPortEntry(PortEntry):
    name: str
    reference: PortEntry

    def serialize(self):
        PortEntry.serialize(self)

    def ref_serialize(self):
        return f" {self.name}:={self.reference.name} "
