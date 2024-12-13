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

import xml.etree.ElementTree as ET

mapping = {
    "MoveToPose": "ExecuteTrajectory",
    "Trigger": "Trigger",
    "RotateEffector": "ExecuteTrajectory"
}

# tree = ET.parse("behavior_tree.xml")
# root = tree.getroot()

# Function to update XML based on mapping
def update_xml(element, sequence=1):
    for child in element:
        if child.tag == "MoveToPose":
            # Replace the tag name
            child.tag = mapping.get("MoveToPose", child.tag)
            
            # Replace `pose` with `sequence` attribute
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1
        elif child.tag == "Trigger":
            # Replace the tag name
            child.tag = mapping.get("Trigger", child.tag)
        
        # Recursively process child elements
        sequence = update_xml(child, sequence)
    return sequence

# Update the XML tree
# update_xml(root)

# Write the modified XML back to a file
# tree.write("updated_behavior_tree.xml", encoding="utf-8", xml_declaration=True)
