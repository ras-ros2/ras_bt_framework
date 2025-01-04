#!/usr/bin/env python3

"""
Copyright (C) 2025 Sachin Kumar

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
Sachin Kumar
Email: info@opensciencestack.org
"""

import os
from utils import read_yaml
from dataclasses import dataclass, field
from typing import List,Dict
from string import Template
import sys
from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from ras_resource_lib.generators.common_utils import _expand_template

PATH_HPP = "generated_headers" # TODO (Sachin): remove this later

@dataclass
class PrimitiveTemplate(ConfigLoaderBase):
    type: str
    input_ports: Dict[str, str] = field(default_factory=dict)
    output_ports: Dict[str, str] = field(default_factory=dict)

    def set_name(self, name: str):
        self.name = name

    def prepare_bt_ports(self):
        bt_ports : List[str] = []
        for port in self.input_ports:
            bt_ports.append(f'BT::InputPort<std::string>("{port}")')
        for port in self.output_ports:
            bt_ports.append(f'BT::OutputPort<std::string>("{port}")')

        return ', '.join([f'{port}' for port in bt_ports]) 

    def get_template_data(self) -> dict:
        template_data = {
            "class_name": self.name.capitalize(),
            "provided_ports" : self.prepare_bt_ports(),
        }
        return template_data
    
@dataclass
class PrimitivesConfig(ConfigLoaderBase):
    primitives: Dict[str, PrimitiveTemplate]

class PrimitiveGenerator:
    """
    A class used to generate and manage primitives.

    Attributes
    ----------
        A list of Primitive objects to be generated.

    Methods
    -------
    generate_primitives() -> List[Primitive]
        Generates header files for each primitive in the primitives list.
    load_primitives(path: str) -> bool
        Loads primitives from a YAML file located at the given path.
    _gen_header_file(prim: Primitive)
        Generates a header file for a given primitive using a template.
    """

    primitives : List[PrimitivesConfig]

    def __init__(self, path: str):
        if not self.load_primitives(path):
            raise ValueError("Primitives not loaded")

    def generate_primitives_header_files(self) -> None:
        if len(self.primitives) < 1:
            raise ValueError("Primitives not loaded")
        for prim in self.primitives:
            self._gen_header_file(prim)
    
    def load_primitives(self, path: str) -> bool:
        try:
            if "primitives" not in read_yaml(path):
                raise KeyError("The key 'primitives' is missing from the YAML file.")
            data : dict = read_yaml(path)["primitives"]
            # self.primitives = extract_primitives(data)
            for _key, value in data.items():
                # self.primitives = extract_primitives(value)
                print("debug")
                print("value", value)
                prim = PrimitiveTemplate.from_dict(value)
                print("post debug")
                # prim.set_name(_key)
            return True
        except Exception as e:
            print(f"Error loading primitives: {e}")
            return False

    def _gen_header_file(self, prim: PrimitiveTemplate):
        _expand_template("templates/primitive_template.hpp.em", prim.get_template_data(), f"{PATH_HPP}/{prim.name}.hpp")

def main():
    if len(sys.argv) < 2:
        print("Usage: generate_primitive.py <path_to_yaml>")
        sys.exit(1)
    path = sys.argv[1]
    try:
        prim_gen = PrimitiveGenerator(path)
        prim_gen.generate_primitives_header_files()
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()