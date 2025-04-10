from ras_bt_framework.behavior_template.instruction import PrimitiveInstruction
from dataclasses import dataclass
from typing import Any


@dataclass
class trigger(PrimitiveInstruction):
	i_trigger : str
	o_status : str

@dataclass
class move2pose(PrimitiveInstruction):
	i_pose : str
	o_status : str

@dataclass
class Move(PrimitiveInstruction):
    """
    Move primitive for moving from one pose to another.
    This corresponds to the C++ Move primitive.
    
    Attributes:
        i_from (Any): The starting pose identifier (can be None for initial moves)
        i_to (Any): The target pose identifier
        o_status (str): The output status of the move operation
    """
    i_from : Any  # Can be either string ID or pose object, or None for initial moves
    i_to : Any    # Can be either string ID or pose object
    o_status : str = ""

@dataclass
class Pick(PrimitiveInstruction):
    """
    Pick primitive for grasping objects.
    This corresponds to the C++ Pick primitive.
    
    The Pick operation moves from an 'above' position to the object position ('at'),
    closes the gripper, and then moves back to the 'above' position.
    
    Attributes:
        i_above (Any): The pose identifier for the position above the object
        i_at (Any): The pose identifier for the position at the object
        o_status (str): The output status of the pick operation
    """
    i_above : Any  # Can be either string ID or pose object
    i_at : Any     # Can be either string ID or pose object
    o_status : str = ""

@dataclass
class Place(PrimitiveInstruction):
    """
    Place primitive for placing objects.
    This corresponds to the C++ Place primitive.
    
    The Place operation moves from an 'above' position to the target position ('at'),
    opens the gripper to release the object, and then moves back to the 'above' position.
    
    Attributes:
        i_above (Any): The pose identifier for the position above the target
        i_at (Any): The pose identifier for the target position
        o_status (str): The output status of the place operation
    """
    i_above : Any  # Can be either string ID or pose object
    i_at : Any     # Can be either string ID or pose object
    o_status : str = ""
