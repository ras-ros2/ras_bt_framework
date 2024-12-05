from..behavior_template.keyword import KeywordPrimitives
from ..behaviors.primitives import MoveToPose,RotateEffector
from tf_transformations import quaternion_from_euler

class MoveToPose(KeywordPrimitives):
    input_port_names = {"pose"}
    def __init__(self,pose_dict):
        qx, qy, qz, qw = quaternion_from_euler(pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'])
        pose_value  = ",".join(map(str, (
            pose_dict['x'],
            pose_dict['y'],
            pose_dict['z'],
            qx, qy, qz, qw
        )))
        super().__init__(pose=pose_value)

class RotateEffector(KeywordPrimitives):
    input_port_names = {"rotation_angle"}
    def __init__(self,rotation_angle):
        super().__init__(rotation_angle=rotation_angle)



