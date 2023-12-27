import csv
import glob
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QListWidget


class ROS2TopicPublisher(Node):
    def __init__(self, csv_file):
        super().__init__('csv_topic_publisher')
        self.csv_file = csv_file
        self.csv_data = []
        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.checkpoint_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/checkpoint', 10)
        self.read_csv()

    def read_csv(self):
        try:
            with open(self.csv_file, mode='r') as file:
                csv_reader = csv.DictReader(file)
                for row in csv_reader:
                    self.csv_data.append(row)
        except FileNotFoundError:
            self.get_logger().error(f'CSV file {self.csv_file} not found.')
            rclpy.shutdown()

    def publish_pose(self, row):
        try:
            pose_msg = PoseStamped()
            if row['type'] == 'c':
                publisher = self.checkpoint_publisher
                self.get_logger().info("Publishing checkpoint")
            elif row['type'] == 'g':
                publisher = self.goal_publisher
                self.get_logger().info("Publishing goal")
            else:
                self.get_logger().error('Invalid type.')
                return

            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = float(row['x'])
            pose_msg.pose.position.y = float(row['y'])
            pose_msg.pose.position.z = float(row['z'])
            pose_msg.pose.orientation.x = float(row['qx'])
            pose_msg.pose.orientation.y = float(row['qy'])
            pose_msg.pose.orientation.z = float(row['qz'])
            pose_msg.pose.orientation.w = float(row['qw'])
            publisher.publish(pose_msg)
            self.get_logger().debug(f'Published pose: {pose_msg}')
            self.get_logger().info(f'Published - ID {row["id"]} : {row["comment"]}')
        except (ValueError, KeyError):
            self.get_logger().error('Invalid data format.')
            self.get_logger().info('CSV File Contents:')


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.goals_label = QLabel("GOALS")
        layout.addWidget(self.goals_label)

        self.csv_files_listwidget = QListWidget()
        layout.addWidget(self.csv_files_listwidget)

        self.csv_files = []
        self.node = None

        self.load_csv_files_in_directory()

        self.publish_button = QPushButton("Publish pose")
        layout.addWidget(self.publish_button)
        self.publish_button.clicked.connect(self.publish_csv)

        # Set list height to display 4 rows
        row_height = self.csv_files_listwidget.sizeHintForRow(0)
        list_height = row_height * 4
        self.csv_files_listwidget.setFixedHeight(list_height)

    def load_csv_files_in_directory(self):
        csv_files = glob.glob("*.csv")
        for file_name in csv_files:
            self.csv_files.append(file_name)
            self.csv_files_listwidget.addItem(os.path.basename(file_name))
        if csv_files:
            self.node = ROS2TopicPublisher(csv_files[0])

    def publish_csv(self):
        if self.node:
            current_index = self.csv_files_listwidget.currentRow()
            file_name = self.csv_files[current_index]
            if self.node.csv_file != file_name:
                self.node = ROS2TopicPublisher(file_name)

            # Loop through all poses in csv_data and publish them
            for row in self.node.csv_data:
                self.node.get_logger().info(f"Publishing pose: type={row['type']}, id={row['id']}, comment={row['comment']}")
                self.node.publish_pose(row)

                # Wait for 1 second after publishing
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    exit_code = app.exec_()

    if main_window.node:
        main_window.node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()

