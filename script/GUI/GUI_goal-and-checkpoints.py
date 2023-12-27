#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os, sys
import tkinter as tk
from tkinter import ttk

class MissionPlanningPublisher(Node):
    def __init__(self, goal_csv_file, checkpoint_csv_file):
        super().__init__('mission_planning_publisher')

        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.checkpoint_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/checkpoint', 10)

        self.get_logger().debug('Mission Planning Publisher has been initialized.')
        self.goal_csv_file = goal_csv_file
        self.checkpoint_csv_file = checkpoint_csv_file
        self.goal_csv_data = []  # Goal CSVファイルから読み込んだデータを格納するリスト
        self.checkpoint_csv_data = []  # Checkpoint CSVファイルから読み込んだデータを格納するリスト
        self.read_csv()

    def read_csv(self):
        try:
            with open(self.goal_csv_file, mode='r') as goal_file:
                goal_csv_reader = csv.reader(goal_file)
                header = next(goal_csv_reader)
                self.get_logger().debug(f'Goal CSV File Contents: {header}')
                for row in goal_csv_reader:
                    self.get_logger().debug(f'Goal CSV Data: {row}')
                    self.goal_csv_data.append(row)

            with open(self.checkpoint_csv_file, mode='r') as checkpoint_file:
                checkpoint_csv_reader = csv.reader(checkpoint_file)
                header = next(checkpoint_csv_reader)
                self.get_logger().debug(f'Checkpoint CSV File Contents: {header}')
                for row in checkpoint_csv_reader:
                    self.get_logger().debug(f'Checkpoint CSV Data: {row}')
                    self.checkpoint_csv_data.append(row)
        except FileNotFoundError:
            self.get_logger().error(f'CSV files not found.')
            rclpy.shutdown()

    def publish_pose(self, data_id, is_goal=True):
        csv_data_to_use = None
        if is_goal:
            csv_data_to_use = self.goal_csv_data
        else:
            csv_data_to_use = self.checkpoint_csv_data

        try:
            id, x, y, z, qx, qy, qz, qw, comment = csv_data_to_use[int(data_id) - 1]
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = float(z)
            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

            if is_goal:
                self.goal_publisher.publish(pose_msg)
                self.get_logger().info(f'Published goal pose: ID {id} : {comment}')
            else:
                self.checkpoint_publisher.publish(pose_msg)
                self.get_logger().info(f'Published checkpoint pose: ID {id} : {comment}')
        except (ValueError, IndexError):
            self.get_logger().error('Invalid ID or data not found.')

class MissionPlanningGUI(tk.Tk):
    def __init__(self, goal_csv_file, checkpoint_csv_file):
        super().__init__()
        self.title("Mission Planning Publisher GUI")
        self.geometry("320x240") 

        self.goal_csv_file = goal_csv_file
        self.checkpoint_csv_file = checkpoint_csv_file
        self.node = MissionPlanningPublisher(goal_csv_file, checkpoint_csv_file)

        self.columnconfigure(1, weight=1)  # Added column configure

        goal_label = tk.Label(self, text="Goal:")
        goal_label.grid(row=0, column=0, pady=10, sticky=tk.W)

        self.goal_listbox = tk.Listbox(self, selectmode=tk.SINGLE, width=15, height=6)
        self.goal_listbox.grid(row=1, column=0, padx=10, pady=(0, 10))

        separator = ttk.Separator(self, orient='vertical')
        separator.grid(row=0, column=1, rowspan=2, padx=10, sticky='ns')

        checkpoint_label = tk.Label(self, text="Checkpoint:")
        checkpoint_label.grid(row=0, column=2, pady=10)

        self.checkpoint_listbox = tk.Listbox(self, selectmode=tk.SINGLE, width=15, height=6)
        self.checkpoint_listbox.grid(row=1, column=2, padx=(10, 10), pady=(0, 10))

        self.goal_ids = self.read_csv_and_get_ids(self.goal_csv_file)
        self.checkpoint_ids = self.read_csv_and_get_ids(self.checkpoint_csv_file)

        for id in self.goal_ids:
            self.goal_listbox.insert(tk.END, id)
        
        for id in self.checkpoint_ids:
            self.checkpoint_listbox.insert(tk.END, id)

        self.publish_goal_button = tk.Button(self, text="Publish Goal", command=self.publish_goal)
        self.publish_goal_button.grid(row=2, column=0, pady=10)

        self.publish_checkpoint_button = tk.Button(self, text="Publish Checkpoint", command=self.publish_checkpoint)
        self.publish_checkpoint_button.grid(row=2, column=2, pady=10)

        self.quit_button = tk.Button(self, text="Quit", command=self.quit)
        self.quit_button.grid(row=3, column=0, columnspan=3, pady=(0, 10))

    def read_csv_and_get_ids(self, csv_file):
        ids = []
        try:
            with open(csv_file, mode='r') as file:
                csv_reader = csv.reader(file)
                header = next(csv_reader)
                for row in csv_reader:
                    id = row[0]
                    comment = row[8]
                    ids.append(f'{id} : {comment}')
        except FileNotFoundError:
            self.quit()
        return ids

    def publish_goal(self):
        selected_index = self.goal_listbox.curselection()
        if len(selected_index) > 0:
            goal_id = self.goal_listbox.get(selected_index)
            goal_id = self.parse_id(goal_id)
            self.node.publish_pose(goal_id, is_goal=True)
        else:
            print("Please select an ID to publish as goal")

    def publish_checkpoint(self):
        selected_index = self.checkpoint_listbox.curselection()
        if len(selected_index) > 0:
            checkpoint_id = self.checkpoint_listbox.get(selected_index)
            checkpoint_id = self.parse_id(checkpoint_id)
            self.node.publish_pose(checkpoint_id, is_goal=False)
        else:
            print("Please select an ID to publish as checkpoint")

    def parse_id(self, selected_value):
        return selected_value.split(' : ')[0]

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        script_name = os.path.basename(sys.argv[0])
        print(f'Usage: python {script_name}  ')
        return

    goal_csv_file = sys.argv[1]
    checkpoint_csv_file = sys.argv[2]

    app = MissionPlanningGUI(goal_csv_file, checkpoint_csv_file)
    app.mainloop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
