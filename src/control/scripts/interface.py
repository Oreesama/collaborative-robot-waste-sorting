#!/usr/bin/env python3

"""
Robot Sorting System GUI
========================

PyQt5-based graphical interface for controlling and monitoring
the robotic waste sorting system.

Features
--------
1. Start/stop robot
2. Enable cooperative mode
3. Enable compliance mode
4. RFID-based admin access
5. Real-time detection monitoring
6. Real-time robot messages
7. Sorting statistics visualization

ROS Topics
----------
Subscribed:
- /detections
- /messages
- /readerNodeMessages
- /stats

Published:
- /handlable
- /commands
"""

import sys
import threading
import time

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal
from std_msgs.msg import String


# ============================================================
# Configuration
# ============================================================

ADMIN_ID = "1234"

RFID_TIMEOUT = 5  # seconds


# ============================================================
# Global State
# ============================================================

last_rfid_time = 0

admin_present = False


# ============================================================
# ROS Publishers
# ============================================================

handlable_pub = rospy.Publisher(
    "handlable",
    String,
    queue_size=10
)

commands_pub = rospy.Publisher(
    "commands",
    String,
    queue_size=10
)


# ============================================================
# Background Threads
# ============================================================

def admin_presence_monitor():
    """
    Monitor admin RFID timeout.
    """

    global admin_present

    while not rospy.is_shutdown():

        if (time.time() - last_rfid_time) > RFID_TIMEOUT:

            admin_present = False

            ui.update_admin_controls()

        time.sleep(0.5)


def request_rfid_thread():
    """
    Continuously request RFID readings.
    """

    while not rospy.is_shutdown():

        handlable_pub.publish("Read tag ID")

        time.sleep(0.2)


# ============================================================
# Worker Threads for Qt Updates
# ============================================================

class UIWorker(QThread):
    """
    Thread used to safely update the GUI.
    """

    signal = pyqtSignal()

    def run(self):
        self.signal.emit()


class AdminWorker(QThread):
    """
    Thread used to safely update admin controls.
    """

    signal = pyqtSignal()

    def run(self):
        self.signal.emit()


# ============================================================
# Main GUI Class
# ============================================================

class Ui_MainWindow(object):

    def __init__(self):

        self.robot_running = False
        self.cooperative_mode = False
        self.compliance_mode = False

        self.messages = []
        self.detections = []
        self.stats = []

        # UI update thread
        self.ui_thread = UIWorker()
        self.ui_thread.signal.connect(self.update_ui)

        # Admin control thread
        self.admin_thread = AdminWorker()
        self.admin_thread.signal.connect(
            self.update_admin_controls
        )

    # ========================================================
    # Admin Access Control
    # ========================================================

    def update_admin_controls(self):
        """
        Enable/disable controls based on RFID admin presence.
        """

        enabled = admin_present

        controls = [
            self.pb_start_robot,
            self.pb_start_coop,
            self.pb_start_comp,
            self.pb_stop_comp,
            self.plasticH,
            self.plasticR,
            self.metalH,
            self.metalR,
            self.syringeH,
            self.syringeR,
            self.electronicH,
            self.electronicR,
            self.broken_glassH,
            self.broken_glassR,
            self.glassH,
            self.glassR,
            self.classcomp,
        ]

        for widget in controls:
            widget.setEnabled(enabled)

    # ========================================================
    # Robot Controls
    # ========================================================

    def toggle_robot(self):

        self.robot_running = not self.robot_running

        if self.robot_running:

            self.robot_state.setText(
                "Robot is : ON"
            )

            commands_pub.publish(
                "robot_state,on"
            )

        else:

            self.robot_state.setText(
                "Robot is : OFF"
            )

            commands_pub.publish(
                "robot_state,off"
            )

    def toggle_cooperative_mode(self):

        self.cooperative_mode = (
            not self.cooperative_mode
        )

        if self.cooperative_mode:

            self.coop_state.setText(
                "Cooperation mode is : ON"
            )

            commands_pub.publish(
                "coop_state,on"
            )

        else:

            self.coop_state.setText(
                "Cooperation mode is : OFF"
            )

            commands_pub.publish(
                "coop_state,off"
            )

    def start_compliance_mode(self):
        """
        Enable compliance mode.
        """

        class_map = {
            "no object": 1,
            "plastic": 2,
            "metal": 3,
            "electronic": 4,
            "syringe": 5,
            "glass": 6,
            "broken glass": 7,
        }

        selected = self.classcomp.currentText()

        object_id = class_map.get(selected, 1)

        commands_pub.publish(
            f"comp_state,on,{object_id}"
        )

    def stop_compliance_mode(self):
        """
        Disable compliance mode.
        """

        commands_pub.publish(
            f"comp_state,off,"
            f"{self.classcomp.currentText()}"
        )

    # ========================================================
    # UI Update
    # ========================================================

    def update_ui(self):
        """
        Refresh GUI text fields.
        """

        self.box_detections.setText(
            "".join(self.detections)
        )

        self.box_messages.setText(
            "".join(self.messages)
        )

        if len(self.stats) < 7:
            return

        self.n_plastic.setText(self.stats[0])
        self.n_metal.setText(self.stats[1])
        self.n_electronic.setText(self.stats[2])
        self.n_syringe.setText(self.stats[3])
        self.n_glass.setText(self.stats[4])
        self.n_brokenglass.setText(self.stats[5])
        self.n_total.setText(self.stats[6])

    # ========================================================
    # GUI Layout
    # ========================================================

    def setupUi(self, MainWindow):

        MainWindow.setObjectName("MainWindow")

        MainWindow.resize(800, 600)

        self.centralwidget = QtWidgets.QWidget(
            MainWindow
        )

        MainWindow.setCentralWidget(
            self.centralwidget
        )

        # ----------------------------------------------------
        # Robot State
        # ----------------------------------------------------

        self.robot_state = QtWidgets.QLabel(
            self.centralwidget
        )

        self.robot_state.setGeometry(
            QtCore.QRect(40, 20, 200, 30)
        )

        font = QtGui.QFont()
        font.setPointSize(12)

        self.robot_state.setFont(font)

        self.robot_state.setText(
            "Robot is : OFF"
        )

        # ----------------------------------------------------
        # Start/Stop Robot Button
        # ----------------------------------------------------

        self.pb_start_robot = QtWidgets.QPushButton(
            self.centralwidget
        )

        self.pb_start_robot.setGeometry(
            QtCore.QRect(40, 60, 180, 40)
        )

        self.pb_start_robot.setText(
            "Start / Stop Robot"
        )

        self.pb_start_robot.clicked.connect(
            self.toggle_robot
        )

        # ----------------------------------------------------
        # Cooperation Mode
        # ----------------------------------------------------

        self.coop_state = QtWidgets.QLabel(
            self.centralwidget
        )

        self.coop_state.setGeometry(
            QtCore.QRect(40, 130, 250, 30)
        )

        self.coop_state.setFont(font)

        self.coop_state.setText(
            "Cooperation mode is : OFF"
        )

        self.pb_start_coop = QtWidgets.QPushButton(
            self.centralwidget
        )

        self.pb_start_coop.setGeometry(
            QtCore.QRect(40, 170, 180, 45)
        )

        self.pb_start_coop.setText(
            "Start / Stop\nCooperation Mode"
        )

        self.pb_start_coop.clicked.connect(
            self.toggle_cooperative_mode
        )

        # ----------------------------------------------------
        # Compliance Mode
        # ----------------------------------------------------

        self.comp_state = QtWidgets.QLabel(
            self.centralwidget
        )

        self.comp_state.setGeometry(
            QtCore.QRect(40, 250, 250, 30)
        )

        self.comp_state.setFont(font)

        self.comp_state.setText(
            "Compliance mode : OFF"
        )

        self.pb_start_comp = QtWidgets.QPushButton(
            self.centralwidget
        )

        self.pb_start_comp.setGeometry(
            QtCore.QRect(40, 290, 80, 35)
        )

        self.pb_start_comp.setText("Start")

        self.pb_start_comp.clicked.connect(
            self.start_compliance_mode
        )

        self.pb_stop_comp = QtWidgets.QPushButton(
            self.centralwidget
        )

        self.pb_stop_comp.setGeometry(
            QtCore.QRect(130, 290, 80, 35)
        )

        self.pb_stop_comp.setText("Stop")

        self.pb_stop_comp.clicked.connect(
            self.stop_compliance_mode
        )

        # ----------------------------------------------------
        # Compliance Object Selector
        # ----------------------------------------------------

        self.classcomp = QtWidgets.QComboBox(
            self.centralwidget
        )

        self.classcomp.setGeometry(
            QtCore.QRect(220, 290, 140, 30)
        )

        self.classcomp.addItems([
            "no object",
            "plastic",
            "metal",
            "electronic",
            "syringe",
            "glass",
            "broken glass",
        ])

        # ----------------------------------------------------
        # Detection Box
        # ----------------------------------------------------

        self.box_detections = QtWidgets.QTextEdit(
            self.centralwidget
        )

        self.box_detections.setGeometry(
            QtCore.QRect(20, 380, 350, 180)
        )

        self.box_detections.setReadOnly(True)

        # ----------------------------------------------------
        # Messages Box
        # ----------------------------------------------------

        self.box_messages = QtWidgets.QTextEdit(
            self.centralwidget
        )

        self.box_messages.setGeometry(
            QtCore.QRect(420, 380, 350, 180)
        )

        self.box_messages.setReadOnly(True)

        # ----------------------------------------------------
        # Statistics Labels
        # ----------------------------------------------------

        self.n_plastic = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_metal = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_electronic = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_syringe = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_glass = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_brokenglass = QtWidgets.QLabel(
            self.centralwidget
        )

        self.n_total = QtWidgets.QLabel(
            self.centralwidget
        )

        stat_labels = [
            ("Plastic:", self.n_plastic),
            ("Metal:", self.n_metal),
            ("Electronic:", self.n_electronic),
            ("Syringe:", self.n_syringe),
            ("Glass:", self.n_glass),
            ("Broken Glass:", self.n_brokenglass),
            ("Total:", self.n_total),
        ]

        y_pos = 40

        for label_text, value_widget in stat_labels:

            label = QtWidgets.QLabel(
                self.centralwidget
            )

            label.setGeometry(
                QtCore.QRect(500, y_pos, 120, 25)
            )

            label.setText(label_text)

            value_widget.setGeometry(
                QtCore.QRect(650, y_pos, 50, 25)
            )

            value_widget.setText("0")

            y_pos += 30

        # ----------------------------------------------------
        # Window Title
        # ----------------------------------------------------

        MainWindow.setWindowTitle(
            "Robot Sorting System"
        )

    # ========================================================
    # End of Class
    # ========================================================


# ============================================================
# ROS Callbacks
# ============================================================

def rfid_callback(msg):
    """
    Handle RFID admin authentication.
    """

    global last_rfid_time
    global admin_present

    data = msg.data.split(",")

    tag_id = data[0]
    rss = float(data[2])

    if tag_id == ADMIN_ID:

        last_rfid_time = time.time()

        admin_present = rss > 50

        ui.update_admin_controls()


def detections_callback(msg):
    """
    Handle object detections.
    """

    data = msg.data.split(",")

    label = data[0]

    if label == "0":
        return

    x = float(data[1])
    y = float(data[2])

    ui.detections.append(
        f"{label} detected at "
        f"x={x:.2f}, y={y:.2f}\n"
    )

    ui.ui_thread.start()


def messages_callback(msg):
    """
    Handle robot status messages.
    """

    ui.messages.append(f"{msg.data}\n")

    ui.ui_thread.start()


def stats_callback(msg):
    """
    Handle sorting statistics.
    """

    ui.stats = msg.data.split(",")

    ui.ui_thread.start()


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    rospy.init_node(
        "ui_node",
        anonymous=True
    )

    # Subscribers
    rospy.Subscriber(
        "detections",
        String,
        detections_callback
    )

    rospy.Subscriber(
        "messages",
        String,
        messages_callback
    )

    rospy.Subscriber(
        "readerNodeMessages",
        String,
        rfid_callback
    )

    rospy.Subscriber(
        "stats",
        String,
        stats_callback
    )

    # Create Qt Application
    app = QtWidgets.QApplication(sys.argv)

    MainWindow = QtWidgets.QMainWindow()

    ui = Ui_MainWindow()

    ui.setupUi(MainWindow)

    MainWindow.show()

    # Start admin monitoring thread
    admin_thread = threading.Thread(
        target=admin_presence_monitor
    )

    admin_thread.daemon = True

    admin_thread.start()

    # Run application
    sys.exit(app.exec_())