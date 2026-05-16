#!/usr/bin/env python3

"""
ROS YOLO Object Detection Node
==============================

This node:
1. Captures frames from a USB camera
2. Applies perspective correction to obtain a top-down workspace view
3. Runs YOLO object detection
4. Converts image coordinates into robot workspace coordinates
5. Publishes detections to a ROS topic

Published Topic:
----------------
/detections (std_msgs/String)

Message format:
----------------
<class_name>,<x>,<y>,<angle>

Example:
--------
cube,512.3,120.7,160
"""

import threading

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from ultralytics import YOLO


# ============================================================
# Configuration
# ============================================================

# Path to trained YOLO model
MODEL_PATH = "/home/akli/dossiers_ROS/workspace/src/control/models/best1.pt"

# Camera index
CAMERA_ID = 0

# Detection publishing rate (Hz)
PUBLISH_RATE = 1

# Default angle offset
ANGLE_OFFSET = 160


# ============================================================
# Workspace Calibration
# ============================================================

# Camera image points (workspace corners in camera frame)
# Order:
# top-left, top-right, bottom-right, bottom-left
camera_points = np.float32([
    [381, 202],
    [400, 366],
    [247, 370],
    [262, 206]
])

# Normalized top-down plane dimensions
plane_points = np.float32([
    [0, 0],
    [640, 0],
    [640, 480],
    [0, 480]
])

# Perspective transform:
# Camera frame -> top-down plane
camera_to_plane = cv2.getPerspectiveTransform(
    camera_points,
    plane_points
)

# Mapping from top-down plane -> robot workspace
plane_source_points = np.float32([
    [0, 0],
    [640, 0],
    [550, 500],
    [-30, 500]
])

workspace_points = np.float32([
    [412, 139],
    [617, 299],
    [739, 129],
    [522, -21]
])

plane_to_workspace = cv2.getPerspectiveTransform(
    plane_source_points,
    workspace_points
)


# ============================================================
# Initialize Model and Camera
# ============================================================

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    raise RuntimeError("Could not open camera.")


# ============================================================
# Camera Grab Thread
# ============================================================

def camera_grabber():
    """
    Continuously grabs frames from the camera buffer.

    This reduces latency by always keeping the latest frame.
    """
    while not rospy.is_shutdown():
        cap.grab()


# ============================================================
# Utility Functions
# ============================================================

def compute_object_pose(box):
    """
    Compute object center position and orientation.

    Parameters
    ----------
    box : ultralytics.engine.results.Boxes
        YOLO bounding box object

    Returns
    -------
    tuple
        (x, y, angle)
    """

    x_min = int(box.xyxy[0][0])
    y_min = int(box.xyxy[0][1])
    x_max = int(box.xyxy[0][2])
    y_max = int(box.xyxy[0][3])

    # Approximate object center
    x = x_max - 90
    y = (y_min + y_max) / 2 + 30

    # Estimate orientation based on box dimensions
    width = x_max - x_min
    height = y_max - y_min

    angle = 0 if width > height else 90
    angle += ANGLE_OFFSET

    return x, y, angle


def transform_to_workspace(x, y):
    """
    Transform image coordinates into robot workspace coordinates.
    """

    point = np.array([x, y], dtype=np.float32)

    transformed = cv2.perspectiveTransform(
        point.reshape(-1, 1, 2),
        plane_to_workspace
    )

    workspace_x = transformed[0][0][0]
    workspace_y = transformed[0][0][1]

    return workspace_x, workspace_y


# ============================================================
# Main ROS Node
# ============================================================

def publisher_node():
    """
    Main ROS node loop.
    """

    rospy.init_node("yolo_detector", anonymous=True)

    publisher = rospy.Publisher(
        "detections",
        String,
        queue_size=1
    )

    rospy.loginfo("YOLO detector node started.")

    rate = rospy.Rate(PUBLISH_RATE)

    # Start camera grabbing thread
    thread = threading.Thread(target=camera_grabber)
    thread.daemon = True
    thread.start()

    while not rospy.is_shutdown():

        # Retrieve latest frame
        ret, frame = cap.retrieve()

        if not ret:
            rospy.logwarn("Failed to retrieve frame.")
            continue

        # Warp perspective to obtain top-down view
        warped_frame = cv2.warpPerspective(
            frame,
            camera_to_plane,
            (640, 480),
            flags=cv2.INTER_LINEAR
        )

        # Run YOLO inference
        results = model(warped_frame)

        # Visualize detections
        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        detection_found = False

        for result in results:

            # No detections
            if len(result.boxes) == 0:
                publisher.publish("0,0,0")
                continue

            detection_found = True

            # Use first detected object
            box = result.boxes[0]

            # Compute pose
            x, y, angle = compute_object_pose(box)

            # Transform coordinates
            workspace_x, workspace_y = transform_to_workspace(x, y)

            # Get class name
            class_name = result.names[int(box.cls)]

            # Create ROS message
            message = (
                f"{class_name},"
                f"{workspace_x},"
                f"{workspace_y},"
                f"{angle}"
            )

            # Publish detection
            publisher.publish(message)

            rospy.loginfo(f"Published: {message}")

        # Optional: publish "clear" when nothing detected
        if not detection_found:
            publisher.publish("clear,0,0")

        rate.sleep()

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":
    try:
        publisher_node()

    except rospy.ROSInterruptException:
        pass