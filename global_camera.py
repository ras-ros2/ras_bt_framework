import cv2
import numpy as np
from ultralytics import YOLO

# ---------------------
# Configuration
# ---------------------

# Load a pretrained YOLO model. Replace with a model that fits your needs.
YOLO_MODEL_PATH = "yolov8n.pt"
model = YOLO(YOLO_MODEL_PATH)

# Camera intrinsic parameters (example values; replace with yours)
camera_matrix = np.array([[1200, 0, 640],
                          [0, 1200, 360],
                          [0, 0, 1]], dtype=np.float64)
dist_coeffs = np.zeros((5, 1))  # or your known distortion coefficients

# ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
aruco_params = cv2.aruco.DetectorParameters()  # Updated for OpenCV 4.7.0 and later

# Known size of the ArUco marker (in meters)
marker_length = 0.05  # Update with your actual marker size

# ---------------------
# Utility Functions
# ---------------------

def detect_aruco_markers(frame):
    """Detect ArUco markers and estimate their pose."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)  # Updated for OpenCV 4.7.0 and later
    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        markers_info = []
        for i, marker_id in enumerate(ids):
            markers_info.append({
                'id': marker_id[0],
                'corners': corners[i],
                'rvec': rvecs[i],
                'tvec': tvecs[i]
            })
        return markers_info
    else:
        return []

def draw_aruco_markers(frame, markers_info):
    """Annotate frame with ArUco marker detections."""
    for marker in markers_info:
        corners = marker['corners']
        cv2.aruco.drawDetectedMarkers(frame, [corners])
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, marker['rvec'], marker['tvec'], 0.05)
    return frame

def detect_containers_yolo(frame, model):
    """Run YOLO inference on the frame to detect potential containers."""
    results = model(frame, conf=0.5)  # adjust confidence threshold as needed
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

# ---------------------
# Main Loop
# ---------------------

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        markers_info = detect_aruco_markers(frame)

        if markers_info:
            frame = draw_aruco_markers(frame, markers_info)
        else:
            detections = detect_containers_yolo(frame, model)
            frame = draw_yolo_detections(frame, detections, class_names=model.names)

        cv2.imshow("Global Camera View", frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
