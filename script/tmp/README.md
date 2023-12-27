# autoware_goal_publisher_GUI

## Overview
This repository provides a Graphical User Interface (GUI) application for setting and publishing goals and checkpoints for Autoware using ROS 2 and PyQt5. This application reads pose data to be published from CSV files and publishes them to the appropriate ROS 2 topics. It also provides the ability to engage or disengage the system through a user-friendly interface.

## Functionality 
The application includes the following functionality:

1. Load and select CSV files containing pose data.
2. Publish pose data as goals or checkpoints to ROS 2 topics.
3. Engage or disengage the system.

## Requirements
- ROS 2 Humble
- PyQt5
- Python 3.8 or above

## How to Use
Follow the steps below to use the GUI Application:

1. Clone this repository.
2. Navigate into the repository directory.
3. Run the application using the command: `python3 main.py`
4. Use the interface to select a CSV file.
5. Click the "Publish pose" button to begin publishing poses from the selected CSV file.
6. Use the "Engage" button to engage or disengage the system.

## CSV File Specification

The CSV files should be specified in the following format:

| type | x   | y   | z   | qx  | qy  | qz  | qw  |
| ---- | --- | --- | --- | --- | --- | --- | --- |
| g    | 4.0 | 5.0 | 6.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| c    | 1.0 | 2.0 | 3.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| ...  | ... | ... | ... | ... | ... | ... | ... |

- `type`: Specifies the type of pose. It has two possible values, 'c' for checkpoint and 'g' for goal.
- `x, y, z`: The position of the pose in the map frame (in meters).
- `qx, qy, qz, qw`: The orientation of the pose in the quaternion representation.

Store your CSV files in the same directory as the python script for easy access.

Ensure that the file contains at least one `goal` row and that this row is placed immediately after the header row. Only one `goal` row should be included.

#### Disclaimer
This application has been created for use with Autoware and ROS 2. Make sure that the system is in a safe state before engaging. Double-check the CSV file to make sure the poses are correct and safe to navigate.
