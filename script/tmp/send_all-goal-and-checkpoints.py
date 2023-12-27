import csv
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

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

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        script_name = os.path.basename(sys.argv[0])
        print(f'Usage: python3 {script_name} <goal_csv> <checkpoints_csv>')
        return
    csv_file = sys.argv[1]

    node = ROS2TopicPublisher(csv_file)

    try:
        for row in node.csv_data:
            node.publish_pose(row)
            time.sleep(1)  # 1秒の待機時間を追加
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

