#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class FakeGripperServer(Node):
    def __init__(self):
        super().__init__("fake_gripper_node")
        self.get_logger().info("Node Init")
        self.create_service(SetBool, "/fake_gripper", self.gripper_callback)

    def gripper_callback(self, req, resp):

        if req.data == True:
            self.get_logger().info("Gripper ON")
        else:
            self.get_logger().info("Gripper OFF")
        
        resp.success = True

        return resp

def main():
    rclpy.init(args=None)
    node = FakeGripperServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

