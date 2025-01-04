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
from typing import List
from string import Template
import sys

PATH_HPP = "generated_headers"

if not os.path.exists(PATH_HPP):
    os.makedirs(PATH_HPP)

@dataclass
class Primitive:
    name: str
    type: str
    input_ports: List[str] = field(default_factory=list)
    output_ports: List[str] = field(default_factory=list)

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

def extract_primitives(item : dict) -> List[Primitive]:
    primitives = []
    if ('primitives' not in item):
        raise KeyError("The key 'primitives' is missing from the YAML file.")
    for name, details in item['primitives'].items():
        if 'type' not in details or 'input_ports' not in details or 'output_ports' not in details:
            raise KeyError(f"The item missing from the primitive '{name}'")

        primitive = Primitive(
            name=name,
            type=details['type'],
            input_ports=details.get('input_ports', []),
            output_ports=details.get('output_ports', [])
        )
        primitives.append(primitive)
    return primitives

class GeneratePrimitive:
    primitives : List[Primitive]

    def generate_primitives(self) -> List[Primitive]:
        if len(self.primitives) < 1:
            raise ValueError("Primitives not loaded")
        for prim in self.primitives:
            self._gen_header_file(prim)
    
    def load_primitives(self, path: str) -> bool:
        try:
            data : dict = read_yaml(path)
            self.primitives = extract_primitives(data)
            return True
        except Exception as e:
            print(f"Error loading primitives: {e}")
            return False

    def _gen_header_file(self, prim: Primitive):
        with open("templates/primitive_template.hpp", "r") as template_file:
            template_content = template_file.read()
        filled_template = Template(template_content).substitute(prim.get_template_data())
        with open(f"{PATH_HPP}/{prim.name}.hpp", "w") as output_file:
            output_file.write(filled_template)
        print(f"Template filled and saved to {PATH_HPP}/{prim.name}.hpp")

def main():
    if len(sys.argv) < 2:
        print("Usage: generate_primitive.py <path_to_yaml>")
        sys.exit(1)
    path = sys.argv[1]
    gen_prim = GeneratePrimitive()
    res = gen_prim.load_primitives(path)
    if res:
        gen_prim.generate_primitives()


if __name__ == '__main__':
    main()
