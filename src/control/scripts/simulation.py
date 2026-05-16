#!/usr/bin/env python3

"""
MoveIt-Based Robotic Sorting Controller
=======================================

This ROS node uses MoveIt to control a robotic arm for:
- Object picking based on vision input
- Sorting into predefined bins
- Simple human-aware speed control via RFID
- Gripper control (open/close)

Architecture
------------
- Input: object detections + RFID messages
- Motion planning: MoveIt
- Output: robot arm trajectories + gripper actions

Topics
------
Subscribed:
- arm_control (std_msgs/String)
- readerNodeMessages (std_msgs/String)
"""

import time

import rospy
import moveit_commander

from geometry_msgs.msg import Pose
from std_msgs.msg import String


# ============================================================
# Configuration
# ============================================================

HUMAN_TAG_ID = "E200680A0000000000000000"

HUMAN_TIMEOUT = 5  # seconds


# ============================================================
# Global State
# ============================================================

human_present = False
last_human_time = 0


# ============================================================
# MoveIt Setup
# ============================================================

robot = None
arm_group = None
gripper_group = None


# ============================================================
# Utility Functions
# ============================================================

def move_to_pose(x, y, z, group):
    """
    Move robot to a Cartesian target pose.
    """

    pose = Pose()

    pose.orientation.x = 1.0
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    group.set_pose_target(pose)
    group.go(wait=True)


def open_gripper():
    """
    Open gripper using named target.
    """

    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)


def close_gripper():
    """
    Close gripper using named target.
    """

    gripper_group.set_named_target("closed")
    gripper_group.go(wait=True)


def go_home():
    """
    Move robot to safe/home position.
    """

    move_to_pose(0.0, 0.3, 0.8, arm_group)


# ============================================================
# Human Detection Callback
# ============================================================

def rfid_callback(msg):
    """
    Update human presence based on RFID detection.
    """

    global human_present
    global last_human_time

    data = msg.data.split("back_antenna")

    tag_id = data[0]

    if tag_id == HUMAN_TAG_ID:

        human_present = True
        last_human_time = time.time()

        rospy.loginfo("Human detected")

        arm_group.set_max_velocity_scaling_factor(0.05)

    else:

        rospy.loginfo("Non-human tag detected")


def update_human_state():
    """
    Reset human presence after timeout.
    """

    global human_present

    if time.time() - last_human_time > HUMAN_TIMEOUT:
        human_present = False
        rospy.loginfo("No human detected")


# ============================================================
# Object Sorting Logic
# ============================================================

def handle_object(obj_id, x, y):
    """
    Main object handling pipeline.
    """

    update_human_state()

    if obj_id == 0:
        return

    # --------------------------------------------------------
    # Check workspace reachability
    # --------------------------------------------------------

    if not (-0.25 <= x <= 0.25 and 0.4 <= y <= 0.7):
        rospy.logwarn("Object not reachable")
        return

    # --------------------------------------------------------
    # Move to object
    # --------------------------------------------------------

    move_to_pose(x, y, 0.6, arm_group)

    rospy.loginfo("Object reached")

    close_gripper()

    # --------------------------------------------------------
    # Sorting logic
    # --------------------------------------------------------

    def pick_bin(px, py):
        move_to_pose(px, py, 0.6, arm_group)
        move_to_pose(px, py, 0.45, arm_group)

    # Fixed object mappings
    if obj_id == 1:
        pick_bin(0.5, 0.55)

    elif obj_id == 9:
        pick_bin(-0.4, -0.5)

    elif obj_id == 5:
        pick_bin(-0.5, 0.2)

    elif obj_id == 6:
        pick_bin(-0.5, -0.15)

    elif obj_id == 7:
        pick_bin(0.0, -0.5)

    # Human-independent bins
    elif not human_present:

        if obj_id == 2:
            pick_bin(0.5, 0.2)

        elif obj_id == 3:
            pick_bin(0.5, -0.15)

        elif obj_id == 4:
            pick_bin(-0.5, 0.55)

        elif obj_id == 8:
            pick_bin(0.4, -0.5)

    rospy.loginfo("Object sorted")

    # --------------------------------------------------------
    # Return home
    # --------------------------------------------------------

    open_gripper()

    move_to_pose(x, y, 0.6, arm_group)

    move_to_pose(0.0, 0.3, 0.8, arm_group)

    rospy.loginfo("Robot ready")


# ============================================================
# Callbacks
# ============================================================

def arm_callback(msg):
    """
    Process incoming object detection messages.
    """

    data = msg.data.split(",")

    obj_id = int(float(data[0]))
    x = float(data[1])
    y = float(data[2])

    handle_object(obj_id, x, y)


# ============================================================
# Main
# ============================================================

def main():

    global robot, arm_group, gripper_group

    rospy.init_node("moveit_sorting_controller")

    moveit_commander.roscpp_initialize([])

    robot = moveit_commander.RobotCommander()

    arm_group = moveit_commander.MoveGroupCommander("arm")

    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # Initial pose
    go_home()

    # Subscribers
    rospy.Subscriber(
        "arm_control",
        String,
        arm_callback,
        queue_size=1
    )

    rospy.Subscriber(
        "readerNodeMessages",
        String,
        rfid_callback
    )

    rospy.loginfo("MoveIt sorting controller started")

    rospy.spin()


if __name__ == "__main__":
    main()