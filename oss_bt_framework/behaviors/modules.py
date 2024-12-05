import os
from .primitives import RotateEffector, SaySomething,ThinkSomethingToSay,MoveToPose,Trigger

from ..behavior_template.module import BehaviorModuleSequence
from geometry_msgs.msg import Pose

from ..behavior_utility.lab_manager import read_yaml_to_pose_dict

from ament_index_python import get_package_share_directory

class SaySomethingSequence(BehaviorModuleSequence):
    output_port_names = {"message"}
    def __init__(self,message_entry):
        super().__init__(output_ports={"message":message_entry},output_port_values={"message":"{out_message}"})
        self.children.extend([
            ThinkSomethingToSay(input_ports={"reference":"Hello World!"},output_ports={"message":"{out_message}"}),
            ])
        
class MyCustomSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.children.extend([
            SaySomethingSequence(message_entry="{my_message}"),
            SaySomething(input_ports={"message":"{my_message}"}),
            ])

class PickObject(BehaviorModuleSequence):
    def __init__(self, sequence_list):
        super().__init__()
        self.children.extend(sequence_list)

class RotateEffectorSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.children.extend([
            RotateEffector(input_ports={"rotation_angle":"1.8"}),
            ])
    

class PressButton(BehaviorModuleSequence):
    def __init__(self,pose:Pose,travel_length):
        super().__init__()
        import numpy as np
        quaternion = np.array(quaternion)
        quaternion /= np.linalg.norm(quaternion)
        from scipy.spatial.transform import Rotation as R
        
        rot = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        
        rotated_vector = rot.apply(np.array([1, 0, 0]))
        
        rotated_vector / np.linalg.norm(rotated_vector)
        
        pressed_pose = Pose()
        pressed_pose.position.x = pose.position.x + rotated_vector[0] * travel_length
        pressed_pose.position.y = pose.position.y + rotated_vector[1] * travel_length
        pressed_pose.position.z = pose.position.z + rotated_vector[2] * travel_length
        pressed_pose.orientation = pose.orientation


        self.children.extend([
            MoveToPose(input_ports={"pose":",".join(map(str, pose))}),
            MoveToPose(input_ports={"pose":",".join(map(str, pressed_pose))}),
            ])