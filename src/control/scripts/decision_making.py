#!/usr/bin/env python3

"""
Robotic Waste Sorting Controller
================================

This ROS node controls a KUKA iiwa robotic arm for automated
waste sorting using:

- YOLO object detections
- RFID-based human detection
- Gripper control
- Cooperative sorting mode
- Safety monitoring

Features
--------
1. Receives object detections from vision system
2. Sorts objects into predefined bins
3. Slows/stops robot when humans are nearby
4. Supports cooperative handling mode
5. Publishes robot status and statistics

ROS Topics
----------
Subscribed:
- /detections
- /readerNodeMessages
- /handlable
- /commands

Published:
- /messages
- /stats
- /UserCommands
"""

import threading
import time

import pyttsx3
import rospy
from std_msgs.msg import String

from client_lib_gripper import kuka_iiwa_ros_client


# ============================================================
# Robot Client Initialization
# ============================================================

robot = kuka_iiwa_ros_client()


# ============================================================
# Configuration
# ============================================================

# RFID tag associated with human operator
HUMAN_TAG_ID = "1234"

# Maximum time before assuming no human is present
HUMAN_TIMEOUT = 5  # seconds

# Default object pickup height
OBJECT_PICK_HEIGHT = 220

# Default robot orientation
DEFAULT_A = 180


# ============================================================
# Sorting Bin Positions
# ============================================================

# Index 0 = home position
BIN_X = [469, 343, 343, 343, 343, 343, 343, 343]
BIN_Y = [0, -420, -420, -420, -420, -420, -420, -420]
BIN_Z = [600, 400, 400, 400, 400, 400, 400, 400]


# ============================================================
# Global State Variables
# ============================================================

human_present = False
human_close = False
human_detection_time = -HUMAN_TIMEOUT

robot_enabled = False
cooperative_mode = False
compliance_mode = False

emergency_alert_sent = False

# Objects allowed in cooperative mode
handlable_objects = []

# List of current detections
detection_queue = []

# Statistics
detections_count = [0, 0, 0, 0, 0, 0]


# ============================================================
# ROS Publishers
# ============================================================

status_pub = rospy.Publisher("messages", String, queue_size=10)

stats_pub = rospy.Publisher("stats", String, queue_size=10)

rfid_command_pub = rospy.Publisher(
    "UserCommands",
    String,
    queue_size=10
)


# ============================================================
# Text-to-Speech
# ============================================================

def speak(text):
    """
    Convert text to speech.
    """

    engine = pyttsx3.init()

    engine.setProperty("rate", 150)
    engine.setProperty("volume", 1.0)

    engine.say(text)
    engine.runAndWait()


# ============================================================
# Detection Class
# ============================================================

class Detection:
    """
    Represents a detected object.
    """

    LABEL_TO_ID = {
        "0": 1,
        "plastic bottle": 2,
        "can": 3,
        "food can": 3,
        "electronic": 4,
        "syringe": 5,
        "glass bottle": 6,
        "glass cup": 6,
        "glass jar": 6,
        "broken_glass": 7,
    }

    def __init__(self, label, x, y):

        self.label = label
        self.x = x
        self.y = y

        self.id = self.LABEL_TO_ID.get(label, 0)


# ============================================================
# Utility Functions
# ============================================================

def wait_until_robot_reaches(target_x, target_y, target_z):
    """
    Wait until the robot reaches the target position.
    """

    while not rospy.is_shutdown():

        current = robot.ToolPosition[0]

        if (
            abs(current[0] - target_x) < 0.1
            and abs(current[1] - target_y) < 0.1
            and abs(current[2] - target_z) < 0.1
        ):
            break


def move_robot(x, y, z, a=0, b=0, c=180):
    """
    Send Cartesian motion command to robot.
    """

    command = (
        f"setPositionXYZABC "
        f"{x} {y} {z} {a} {b} {c} ptp"
    )

    robot.send_command(command)

    status_pub.publish(command)


def update_statistics(detection_id):
    """
    Update sorting statistics.
    """

    index = detection_id - 2

    if index < 0 or index >= len(detections_count):
        return

    detections_count[index] += 1

    total = sum(detections_count)

    stats_message = ",".join(
        map(str, detections_count + [total])
    )

    stats_pub.publish(stats_message)


# ============================================================
# RFID Thread
# ============================================================

def rfid_polling_thread():
    """
    Continuously request RFID tag readings.
    """

    while not rospy.is_shutdown():

        rfid_command_pub.publish("Read tag ID")

        time.sleep(0.2)


# ============================================================
# ROS Callbacks
# ============================================================

def detection_callback(msg):
    """
    Receive object detections from vision node.
    """

    data = msg.data.split(",")

    label = data[0]

    if label == "0":
        return

    if label == "clear":
        detection_queue.clear()
        return

    x = float(data[1])
    y = float(data[2])

    detection_queue.append(
        Detection(label, x, y)
    )


def ui_callback(msg):
    """
    Enable/disable objects in cooperative mode.
    """

    data = msg.data.split(" ")

    obj = data[0]
    state = data[1]

    if state == "on":

        if obj not in handlable_objects:
            handlable_objects.append(obj)

    else:

        if obj in handlable_objects:
            handlable_objects.remove(obj)


def states_callback(msg):
    """
    Handle robot operating states.
    """

    global robot_enabled
    global cooperative_mode
    global compliance_mode

    data = msg.data.split(",")

    command = data[0]

    # --------------------------------------------------------
    # Robot enable/disable
    # --------------------------------------------------------

    if command == "robot_state":

        robot_enabled = data[1] == "on"

    # --------------------------------------------------------
    # Cooperative mode
    # --------------------------------------------------------

    elif command == "coop_state":

        cooperative_mode = data[1] == "on"

    # --------------------------------------------------------
    # Compliance mode
    # --------------------------------------------------------

    elif command == "comp_state":

        if data[1] == "start":

            compliance_mode = True

            robot.send_command(
                "setCompliance 0 0 -13 0 0 0"
            )

            status_pub.publish(
                "setCompliance 0 0 -13 0 0 0"
            )

        else:

            compliance_mode = False

            robot.send_command("resetCompliance")

            status_pub.publish("resetCompliance")


def rfid_callback(msg):
    """
    RFID-based human detection and safety control.
    """

    global human_present
    global human_close
    global human_detection_time
    global emergency_alert_sent

    if not robot.isready:
        return

    data = msg.data.split(",")

    tag_id = data[0]
    rss = float(data[2])

    if tag_id != HUMAN_TAG_ID:
        return

    human_present = True
    human_detection_time = time.time()

    # Slow robot when human detected
    robot.send_command("setJointVelocity 0.1")

    # Emergency stop when human too close
    if rss > 80:

        human_close = True

        robot.send_command("forceStop")

        status_pub.publish("forceStop")

        if not emergency_alert_sent:

            speak("Emergency Stop")

            emergency_alert_sent = True

    else:

        human_close = False
        emergency_alert_sent = False


# ============================================================
# Main Sorting Logic
# ============================================================

def process_detections():
    """
    Main sorting routine.
    """

    global human_present

    if not robot_enabled:
        return

    # Human timeout
    if (
        time.time() - human_detection_time
        > HUMAN_TIMEOUT
    ):
        human_present = False

    for detection in detection_queue:

        # Cooperative mode filter
        if (
            cooperative_mode
            and detection.label not in handlable_objects
        ):
            continue

        # Motion parameters
        robot.send_command("setJointJerk 0.1")

        if human_present:
            robot.send_command("setJointVelocity 0.1")
        else:
            robot.send_command("setJointVelocity 0.3")

        status_pub.publish(detection.label)

        # ----------------------------------------------------
        # Move to object
        # ----------------------------------------------------

        move_robot(
            detection.x,
            detection.y,
            OBJECT_PICK_HEIGHT,
            34,
            0,
            180
        )

        wait_until_robot_reaches(
            detection.x,
            detection.y,
            OBJECT_PICK_HEIGHT
        )

        status_pub.publish("Object reached!")

        # Close gripper
        robot.send_command("setGripperState c")

        status_pub.publish("setGripperState c")

        rospy.sleep(0.5)

        # ----------------------------------------------------
        # Move to sorting bin
        # ----------------------------------------------------

        target_index = detection.id - 1

        move_robot(
            BIN_X[target_index],
            BIN_Y[target_index],
            BIN_Z[target_index],
            0,
            0,
            180
        )

        wait_until_robot_reaches(
            BIN_X[target_index],
            BIN_Y[target_index],
            BIN_Z[target_index]
        )

        # Open gripper
        robot.send_command("setGripperState o")

        status_pub.publish("setGripperState o")

        rospy.sleep(0.5)

        status_pub.publish("Object sorted!")

        # Update statistics
        update_statistics(detection.id)

        # ----------------------------------------------------
        # Return to home position
        # ----------------------------------------------------

        move_robot(
            BIN_X[0],
            BIN_Y[0],
            BIN_Z[0],
            DEFAULT_A,
            0,
            180
        )

        wait_until_robot_reaches(
            BIN_X[0],
            BIN_Y[0],
            BIN_Z[0]
        )

        status_pub.publish("Robot ready")

        rospy.sleep(0.5)

        # Remove processed detection
        if detection in detection_queue:
            detection_queue.remove(detection)


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    rospy.init_node(
        "robot_sorting_controller",
        anonymous=True
    )

    # Subscribers
    rospy.Subscriber(
        "detections",
        String,
        detection_callback
    )

    rospy.Subscriber(
        "readerNodeMessages",
        String,
        rfid_callback
    )

    rospy.Subscriber(
        "handlable",
        String,
        ui_callback
    )

    rospy.Subscriber(
        "commands",
        String,
        states_callback
    )

    rospy.loginfo("Robot sorting controller started.")

    # Allow ROS connections to initialize
    rospy.sleep(2)

    # Request RFID reader connection
    rfid_command_pub.publish("Link with the reader")

    rospy.sleep(2)

    # Start RFID polling thread
    polling_thread = threading.Thread(
        target=rfid_polling_thread
    )

    polling_thread.daemon = True
    polling_thread.start()

    # Main loop
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():

        process_detections()

        rate.sleep()