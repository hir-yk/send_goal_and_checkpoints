
import csv
import glob
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from autoware_auto_vehicle_msgs.msg import Engage

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QListWidget
from PyQt5.QtCore import Qt, QTimer


class ROS2TopicPublisher(Node):
    def __init__(self, csv_file, main_window=None):
        super().__init__('csv_topic_publisher')
        self.csv_file = csv_file
        self.main_window = main_window
        self.is_engaged = False

        # ROS2 Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.checkpoint_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/checkpoint', 10)
        self.engage_publisher = self.create_publisher(Engage, '/autoware/engage', 10)
        self.engage_subscription = self.create_subscription(Engage, '/api/autoware/get/engage', self.engage_callback, qos_profile_sensor_data)

        if csv_file:
            self.csv_data = self.read_csv()
        else:
            self.csv_data = []

    def read_csv(self):
        """ Read the CSV file and transform it to list of dict. """
        try:
            with open(self.csv_file, mode='r') as file:
                return list(csv.DictReader(file))
        except FileNotFoundError:
            self.get_logger().error(f'CSV file {self.csv_file} not found.')
            return []

    def selected_publisher(self, row):
        """ Select the appropriate publisher based on row data. """
        if row['type'] == 'c':
            return self.checkpoint_publisher
        elif row['type'] == 'g':
            return self.goal_publisher
        else:
            return None

    def pose_from_row(self, row):
        """ Create a PoseStamped message from csv row. """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(row['x'])
        pose_msg.pose.position.y = float(row['y'])
        pose_msg.pose.position.z = float(row['z'])
        pose_msg.pose.orientation.x = float(row['qx'])
        pose_msg.pose.orientation.y = float(row['qy'])
        pose_msg.pose.orientation.z = float(row['qz'])
        pose_msg.pose.orientation.w = float(row['qw'])
        return pose_msg

    def toggle_engage(self):
        """Toggle the status of self.is_engaged and publish it."""
        msg = Engage()
        self.is_engaged = not self.is_engaged
        msg.engage = self.is_engaged
        self.engage_publisher.publish(msg)
        if self.main_window:
            self.main_window.update_engage_button()
        return self.is_engaged

    def engage_callback(self, msg):
        """Receive engagement status and update self.is_engaged"""
        self.is_engaged = msg.engage
        if self.main_window:
            self.main_window.update_engage_button()

    def publish_pose(self, row):
        """Publish a pose based on given row data."""
        if row:
            pose_msg = self.pose_from_row(row)
            publisher = self.selected_publisher(row)
            if publisher:
                publisher.publish(pose_msg)
                self.get_logger().info(f"Publishing pose: {pose_msg}")


class MainWindow(QWidget):
    def __init__(self, publisher):
        super().__init__()
        self.csv_publisher_node = publisher
        self.init_ui()

    def init_ui(self):
        """Initialize the user interface."""
        self.setLayout(QVBoxLayout())

        self.add_csv_files_list()
        self.add_no_file_label()
        self.add_buttons()

    def add_csv_files_list(self):
        """Create csv files list widget and add it to layout."""
        self.csv_files_listwidget = QListWidget()
        self.csv_files = self.load_csv_files()
        self.csv_files_listwidget.addItems(self.csv_files)
        self.layout().addWidget(self.csv_files_listwidget)

    def load_csv_files(self):
        """Return a list of csv file names in the current directory."""
        csv_files = glob.glob("*.csv")
        return [os.path.basename(file) for file in csv_files]

    def add_no_file_label(self):
        """Create no file label and add it to layout."""
        self.no_file_label = QLabel("No .csv file selected")
        self.no_file_label.setStyleSheet('color: red')
        self.layout().addWidget(self.no_file_label)
        self.update_no_file_label()

    def add_buttons(self):
        """Create publish and engage buttons and add them to layout."""
        self.publish_button = QPushButton("Publish pose")
        self.publish_button.clicked.connect(self.publish_csv)
        self.layout().addWidget(self.publish_button)

        self.engage_button = QPushButton("Engage")
        self.engage_button.clicked.connect(self.toggle_engage)
        self.layout().addWidget(self.engage_button)

    def update_engage_button(self):
        """Update the color of the engage button based on engagement status."""
        self.engage_button.setStyleSheet(f'color: {"green" if self.csv_publisher_node.is_engaged else "black"}')

    def toggle_engage(self):
        """Toggle engagement status."""
        if self.csv_publisher_node and self.csv_files:
            self.csv_publisher_node.toggle_engage()

    def publish_csv(self):
        """Publish poses from csv file."""
        if self.csv_publisher_node and self.csv_files:
            current_index = self.csv_files_listwidget.currentRow()
            file_name = self.csv_files[current_index]
            self.csv_publisher_node.csv_file = file_name
            self.csv_publisher_node.csv_data = self.csv_publisher_node.read_csv()

            for row in self.csv_publisher_node.csv_data:
                self.csv_publisher_node.publish_pose(row)
                time.sleep(1)

    def update_no_file_label(self):
        """Update the visibility of the no file label based on the existence of csv files."""
        self.no_file_label.setVisible(not self.csv_files)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])
    node = ROS2TopicPublisher(None)
    main_window = MainWindow(node)
    node.main_window = main_window
    main_window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node))
    timer.start(100)

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    sys.exit(main())


