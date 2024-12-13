#!/usr/bin/env python3

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

