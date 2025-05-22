#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from aruco_dist_msg.msg import ArucoDistance
import getkey
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import time


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        # self.subscription_dm = self.create_subscription(
        #     MarkerArray,
        #     'detected_markers',
        #     self.listener_callback,
        #     10)
        self.subscription_dst = self.create_subscription(
            ArucoDistance,
            'detected_marker_distance',
            self.distance_callback,
            10
        )
        # self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.twist = Twist()
        self.finish_move = False


        self.subscription_js = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        # self.subscription  # prevent unused variable warning

        self.get_joint = False
        self.marker = []

    def joint_states_callback(self, msg):
        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def distance_callback(self, msg):
        self.target_marker_id = 0                      # Change this to the desired marker ID
        ids, dists = msg.id, msg.distance
        for id, distance in zip(ids, dists):
            # print(f"marker.id:{marker.id} distance : {marker.pose.pose.distance}")
            if id == self.target_marker_id:
                self.get_logger().debug(f'Marker ID: {id}, Distance: {distance}')
                # print(f'z:[{marker.pose.pose.position.z}] x:[{marker.pose.pose.position.x}] ')
                if distance > 0.8:
                    self.publish_cmd_vel(0.10)
                elif distance > 0.7:
                    self.publish_cmd_vel(0.06)
                elif distance > 0.56 :
                    self.publish_cmd_vel(0.04)
                else:
                    self.publish_cmd_vel(0.0)
                    self.finish_move = True
                break
    def listener_callback(self, msg):
        self.target_marker_id = 0                      # Change this to the desired marker ID
        for marker in msg.markers:
            # print(f"marker.id:{marker.id} distance : {marker.pose.pose.distance}")
            if marker.id == self.target_marker_id:
                self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                print(f'z:[{marker.pose.pose.position.z}] x:[{marker.pose.pose.position.x}] ')
                if marker.pose.pose.position.z > 0.49:
                    self.publish_cmd_vel(0.10)
                elif marker.pose.pose.position.z > 0.47:
                    self.publish_cmd_vel(0.06)
                elif marker.pose.pose.position.z > 0.45 :
                    self.publish_cmd_vel(0.04)
                else:
                    self.publish_cmd_vel(0.0)
                    self.finish_move = True
                break
            
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        # self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)            

    def send_gripper_goal(self, position):  
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0

        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return
            
        self.gripper_action_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    #rclpy.spin(node)

    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()

    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500

    # point.positions = [0.0, -1.052, 1.106, 0.029]
    point.positions = [0.0, -1.1519, 0.0524, 2.0071]

    print("point",point.positions)
    trajectory_msg.points = [point]
    joint_pub.publish(trajectory_msg)

    print("init done")
    rclpy.spin_once(node)

    while(rclpy.ok()):
        key_value = getkey.getkey()
        if key_value == '1':          # cube home
            print(f"No. {key_value}")
            point.positions = [0.0, -1.1519, 0.0524, 2.0071]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)
            node.send_gripper_goal(0.015)

        elif key_value == '2':        # box home
            print(f"No. {key_value}")
            point.positions = [0.0, -1.052, 1.106, 0.029]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        # elif key_value == '3':        # move to cube
        #     print(f"No. {key_value}")
        #     while 1:
        #         rclpy.spin(node)
        #         if(node.finish_move == True):
        #             node.finish_move = False
        #             continue
        elif key_value == '3':        # move to cube
            print(f"No. {key_value} - Waiting for finish_move")
            count = 0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                print(f"Loop {count}, finish_move: {node.finish_move}")
                count += 1
                if node.finish_move:
                    print("Finish move detected, breaking loop")
                    node.finish_move = False
                    point.time_from_start.sec = 3
                    point.positions = [0.0, 1.0123, -0.8378, 1.2043]
                    print("Move to object:", point.positions)
                    node.send_gripper_goal(0.015)
                    joint_pub.publish(trajectory_msg)
                    time.sleep(2.5)

                    point.positions = [0.0, 0.6807, -0.8378, 1.2043]
                    node.send_gripper_goal(-0.005)
                    print("Grabbed the object")
                    time.sleep(1.5)
                    print("Lift object:", point.positions)
                    joint_pub.publish(trajectory_msg)
                    time.sleep(1.5)

                    point.positions = [1.0472, 0.6807, -0.8378, 1.2043]
                    print("Rotate to :", point.positions)
                    joint_pub.publish(trajectory_msg)
                    time.sleep(1.5)

                    point.positions = [1.0472, 1.0123, -0.8378, 1.2043]
                    print("Put object down :", point.positions)
                    joint_pub.publish(trajectory_msg)
                    time.sleep(2.5)
                    node.send_gripper_goal(0.015)
                    print("Let it go")
                    time.sleep(1.5)

                    point.positions = [1.0472, 0.6807, -0.8378, 1.2043]
                    print("Safe zone:", point.positions)
                    joint_pub.publish(trajectory_msg)
                    time.sleep(1.5)

                    point.positions = [0.0, -1.1519, 0.0524, 2.0071]
                    print("Initial position :", point.positions)
                    joint_pub.publish(trajectory_msg)
                    break


        elif key_value == '4':        # forward
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '5':        # backward
            print(f"No. {key_value}") 
            node.twist.linear.x =-0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '6':        # forward + left turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '7':        # forward + right turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z =-0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '8':        # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
            node.get_joint = False

        elif key_value == '9':        # distance and offset for marker
            print(f"No. {key_value}")
            rclpy.spin_once(node)
        elif key_value == '-':
            node.send_gripper_goal(-0.005)
            print('close')
        elif key_value == '=':
            node.send_gripper_goal(0.015)
            print('open')

        elif key_value == '0':
            print(f"No. {key_value}")
            point.time_from_start.sec = 3
            point.positions = [0.0, 1.0123, -0.8378, 1.2043]
            print("Move to object:", point.positions)
            node.send_gripper_goal(0.015)
            joint_pub.publish(trajectory_msg)
            time.sleep(2.5)

            point.positions = [0.0, 0.6807, -0.8378, 1.2043]
            node.send_gripper_goal(-0.005)
            print("Grabbed the object")
            time.sleep(1.5)
            print("Lift object:", point.positions)
            joint_pub.publish(trajectory_msg)
            time.sleep(1.5)

            point.positions = [1.0472, 0.6807, -0.8378, 1.2043]
            print("Rotate to :", point.positions)
            joint_pub.publish(trajectory_msg)
            time.sleep(1.5)

            point.positions = [1.0472, 1.0123, -0.8378, 1.2043]
            print("Put object down :", point.positions)
            joint_pub.publish(trajectory_msg)
            time.sleep(2.5)
            node.send_gripper_goal(0.015)
            print("Let it go")
            time.sleep(1.5)

            point.positions = [1.0472, 0.6807, -0.8378, 1.2043]
            print("Safe zone:", point.positions)
            joint_pub.publish(trajectory_msg)
            time.sleep(1.5)

            point.positions = [0.0, -1.1519, 0.0524, 2.0071]
            print("Initial position :", point.positions)
            joint_pub.publish(trajectory_msg)


        elif key_value == 'q':
            print(f"No. {key_value}")
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()