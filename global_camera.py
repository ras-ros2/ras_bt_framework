#!/usr/bin/env python3

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv2 import aruco
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler, euler_from_matrix
import tf_transformations

from ultralytics import YOLO

##################### CONFIGURATIONS #######################
# YOLO model path
YOLO_MODEL_PATH = "yolov8n.pt"
model = YOLO(YOLO_MODEL_PATH)

# Camera matrix & distortion (from the first script)
cam_mat = np.array([[931.1829833984375, 0.0, 640.0],
                    [0.0, 931.1829833984375, 360.0],
                    [0.0, 0.0, 1.0]], dtype=np.float32)
dist_mat = np.zeros((5, 1))

# Known size of the ArUco marker (in cm)
size_of_aruco_cm = 15
aruco_area_threshold = 1500

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters()
arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
arucoParams.cornerRefinementWinSize = 8

# Camera info
sizeCamX = 1280
sizeCamY = 720
centerCamX = 640
centerCamY = 360
focalX = 931.1829833984375
focalY = 931.1829833984375


##################### UTILITY FUNCTIONS #######################

def calculate_rectangle_area(coordinates):
    """
    Calculate the area of the ArUco marker's bounding rectangle.
    """
    height = ((coordinates[0] - coordinates[4])**2 + (coordinates[1] - coordinates[5])**2)**0.5
    width = ((coordinates[2] - coordinates[4])**2 + (coordinates[3] - coordinates[5])**2)**0.5
    area = width * height
    return area, width

def detect_aruco(image, depth):
    """
    Detect ArUco markers, filter them by area threshold,
    and return their centers, depths, angles, widths, and IDs.
    """
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids_list = []
    tvecs_list = []

    brightness = 15
    contrast = 2.1
    image_enh = cv2.addWeighted(image, contrast, np.zeros(image.shape, image.dtype), 0, brightness)

    # Detect ArUco
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image_enh, dictionary, parameters=arucoParams)
    if ids is not None:
        for i in range(len(ids)):
            x1 = corners[i][0][0][0]
            y1 = corners[i][0][0][1]
            x2 = corners[i][0][2][0]
            y2 = corners[i][0][2][1]
            x3 = corners[i][0][3][0]
            y3 = corners[i][0][3][1]

            coordinates = [x1, y1, x2, y2, x3, y3]
            area, width = calculate_rectangle_area(coordinates)

            if area >= aruco_area_threshold:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_cm, cam_mat, dist_mat)
                ids_list.append(int(ids[i]))
                current_rvec = rvec[0]
                current_tvec = tvec[0]
                center_aruco = np.mean(corners[i][0], axis=0)
                center_x, center_y = map(int, center_aruco)
                depth_data = depth[center_y, center_x]

                distance_from_rgb_list.append(depth_data)
                center_aruco_list.append((center_x, center_y))
                width_aruco_list.append(width)
                angle_aruco_list.append(current_rvec)
                tvecs_list.append(current_tvec)

                cv2.aruco.drawDetectedMarkers(image, [corners[i]])
                cv2.drawFrameAxes(image, cam_mat, dist_mat, current_rvec, current_tvec, 2, 1)
                cv2.putText(image, f"ID: {ids[i]}", (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                cv2.circle(image, (center_x, center_y), 2, (255, 0, 0), 6)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_list, tvecs_list, image


def detect_containers_yolo(frame, model):
    """Run YOLO inference on the frame to detect potential containers."""
    results = model(frame, conf=0.5)
    detections = []
    if len(results) > 0:
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                detections.append({
                    'class_id': cls_id,
                    'confidence': conf,
                    'bbox': (int(x1), int(y1), int(x2), int(y2))
                })
    return detections

def draw_yolo_detections(frame, detections, class_names=None):
    """Annotate frame with YOLO detection results."""
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        label = f"Class:{det['class_id']} Conf:{det['confidence']:.2f}"
        if class_names and det['class_id'] < len(class_names):
            label = f"{class_names[det['class_id']]} {det['confidence']:.2f}"
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - text_height - baseline), (x1 + text_width, y1), (255, 0, 0), -1)
        cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    return frame


##################### CLASS DEFINITION #######################

class IntegratedDetectionNode(Node):
    def __init__(self):
        super().__init__('integrated_detection_node')
        self.cv_image = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        # Timer callback to process images
        self.timer = self.create_timer(0.2, self.process_image)

    def depthimagecb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def process_image(self):
        if self.cv_image is None or self.depth_image is None:
            return

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # Attempt ArUco detection
        center_list, distance_list, angle_list, width_list, ids_list, tvec_list, out_img = detect_aruco(gray, self.depth_image)

        if len(ids_list) > 0:
            # Found ArUco markers; broadcast transforms
            for i, aruco_id in enumerate(ids_list):
                distance = distance_list[i]
                angle_aruco = angle_list[i]
                tvec = tvec_list[i]

                # Compute adjusted angle
                # Using a polynomial approximation as in the original script
                angle_est = angle_aruco[0][2]
                angle_est = ((0.788 * angle_est) - ((angle_est**2)/3160))
                if round(angle_est) == 0:
                    angle_est = angle_est + math.pi
                    roll, pitch, yaw = 0.4, 0, angle_est
                else:
                    roll, pitch, yaw = 0, -0.261, angle_est

                # Convert to proper rotation
                r = tf_transformations.euler_matrix(roll, pitch, yaw)
                # Additional transform adjustments if needed (as in original code)
                # transform_matrix = np.array([[0,0,1],[-1,0,0],[0,1,0]])
                # ... apply if needed

                qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

                center_x, center_y = center_list[i]
                # Compute 3D position using depth and camera intrinsics
                x_1 = distance/1000.0
                y_1 = x_1 * (sizeCamX - center_x - centerCamX) / focalX
                z_1 = x_1 * (sizeCamY - center_y - centerCamY) / focalY

                # Broadcast transform from camera_link to cam_<id>
                transform_msg = TransformStamped()
                transform_msg.header.stamp = self.get_clock().now().to_msg()
                transform_msg.header.frame_id = 'camera_link'
                transform_msg.child_frame_id = f'cam_{aruco_id}'
                transform_msg.transform.translation.x = x_1
                transform_msg.transform.translation.y = y_1
                transform_msg.transform.translation.z = z_1
                transform_msg.transform.rotation.x = qx
                transform_msg.transform.rotation.y = qy
                transform_msg.transform.rotation.z = qz
                transform_msg.transform.rotation.w = qw
                self.br.sendTransform(transform_msg)

                # Attempt lookup from base_link to obj_<id>
                try:
                    t = self.tf_buffer.lookup_transform('base_link', transform_msg.child_frame_id, rclpy.time.Time())
                    transform_msg.header.stamp = self.get_clock().now().to_msg()
                    transform_msg.header.frame_id = 'base_link'
                    transform_msg.child_frame_id = f'obj_{aruco_id}'
                    transform_msg.transform = t.transform
                    self.br.sendTransform(transform_msg)
                except Exception as e:
                    pass

            # Show image with ArUco detections
            cv2.imshow("Detections", out_img)
            cv2.waitKey(1)

        else:
            # No ArUco found; run YOLO detection
            detections = detect_containers_yolo(self.cv_image, model)
            frame_out = draw_yolo_detections(self.cv_image.copy(), detections, class_names=model.names)
            cv2.imshow("Detections", frame_out)
            cv2.waitKey(1)


def main():
    rclpy.init(args=sys.argv)
    node = IntegratedDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
