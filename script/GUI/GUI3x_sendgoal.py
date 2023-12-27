#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os, sys
import tkinter as tk  # Tkinterをインポート

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

class MissionPlanningGUI(tk.Tk):
    def __init__(self, csv_file):
        super().__init__()
        self.title("Mission Planning Publisher GUI")
        self.geometry("400x380")

        self.csv_file = csv_file
        self.node = MissionPlanningPublisher(csv_file)

        self.label = tk.Label(self, text="Select an ID to publish:", font=("Arial", 16))
        self.label.pack(pady=10)

        self.selected_id = tk.StringVar(self)  # 選択されたIDを格納する変数
        self.selected_id.set("")  # 初期値は空白

        # CSVファイルからIDとコメントを読み込む
        self.ids_and_comments = self.read_csv_and_get_ids_and_comments()

        # フレームを作成してリストボックスとスクロールバーを配置
        frame = tk.Frame(self)
        frame.pack(pady=5)

        self.comment_listbox = tk.Listbox(frame, selectmode=tk.SINGLE, height=5, font=("Arial", 24))
        self.comment_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.scrollbar = tk.Scrollbar(frame, orient=tk.VERTICAL)
        self.comment_listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.comment_listbox.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for id, comment in self.ids_and_comments.items():
            self.comment_listbox.insert(tk.END, f"{id}: {comment}")

        self.publish_button = tk.Button(self, text="Publish", command=self.publish, font=("Arial", 16))
        self.publish_button.pack(pady=10)

        self.quit_button = tk.Button(self, text="Quit", command=self.quit, font=("Arial", 16))
        self.quit_button.pack(pady=6)

    def read_csv_and_get_ids_and_comments(self):
        ids_and_comments = {}
        try:
            with open(self.csv_file, mode='r') as file:
                csv_reader = csv.reader(file)
                header = next(csv_reader)
                for row in csv_reader:
                    id, _, _, _, _, _, _, _, comment = row
                    ids_and_comments[id] = comment
        except FileNotFoundError:
            self.quit()
        return ids_and_comments

    def publish(self):
        selected_index = self.comment_listbox.curselection()
        if selected_index:
            selected_id = list(self.ids_and_comments.keys())[selected_index[0]]
            self.node.publish_pose(selected_id)
        else:
            print("Please select an ID to publish")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        script_name = os.path.basename(sys.argv[0])
        print(f'Usage: python {script_name} <csv_file>')
        return
    csv_file = sys.argv[1]

    app = MissionPlanningGUI(csv_file)
    app.mainloop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
