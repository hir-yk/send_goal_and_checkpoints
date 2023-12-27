#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os, sys

class MissionPlanningPublisher(Node):
    def __init__(self, csv_file):
        super().__init__('mission_planning_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.get_logger().debug('Mission Planning Publisher has been initialized.')
        self.csv_file = csv_file
        self.csv_data = []  # CSVファイルから読み込んだデータを格納するリスト
        self.read_csv()

    def read_csv(self):
        try:
            with open(self.csv_file, mode='r') as file:
                csv_reader = csv.reader(file)
                header = next(csv_reader)  # ヘッダー行を読み込み
                self.get_logger().debug(f'CSV File Contents: {header}')
                for row in csv_reader:
                    self.get_logger().debug(f'CSV Data: {row}')
                    self.csv_data.append(row)
        except FileNotFoundError:
            self.get_logger().error(f'CSV file {self.csv_file} not found.')
            rclpy.shutdown()

    def publish_pose(self, data_id):
        try:
            id, x, y, z, qx, qy, qz, qw, comment = self.csv_data[int(data_id) - 1]
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"  # フレームIDを設定
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = float(z)
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)
            self.publisher.publish(pose_msg)
            self.get_logger().debug(f'Published pose for ID {id} with header.frame_id: {pose_msg.header.frame_id}')
            self.get_logger().debug(f'Published pose data: {pose_msg.pose}')
            self.get_logger().debug(f'Comment: {comment}')
            self.get_logger().info(f'published - ID {id} : {comment}')
        except (ValueError, IndexError):
            self.get_logger().error('Invalid ID or data not found.')
            self.get_logger().info('CSV File Contents:')
            for idx, row in enumerate(self.csv_data):
                id, _, _, _, _, _, _, _, comment = row
                self.get_logger().info(f'ID {idx + 1}: {comment}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        script_name = os.path.basename(sys.argv[0])
        print(f'Usage: python {script_name} <csv_file>')
        return
    csv_file = sys.argv[1]

    node = MissionPlanningPublisher(csv_file)

    try:
        while rclpy.ok():
            data_id = input('Enter the ID to publish (or q to quit): ')
            if data_id.lower() == 'q':
                break
            node.publish_pose(data_id)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
