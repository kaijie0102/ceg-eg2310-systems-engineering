#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseArray
import math


class SavePoses(Node):

    def __init__(self):
        super().__init__('save_poses')

        self.pose_position_x = Pose()
        self.pose_position_y = Pose()
        self.pose_orientation_z = Pose()
        self.pose_orientation_w = Pose()

        self.poses_dict = {}
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/waypoints', self.sub_callback, 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/waypoints_array', 10)
        self.posearray = PoseArray()
        self.pose_no = 1
        self.write_to_file()

    def sub_callback(self, msg):
        self.posearray.header = msg.header
        self.posearray.poses.append(Pose(msg.pose.pose.position, msg.pose.pose.orientation))

        self.pose_position_x = msg.pose.pose.position.x
        self.pose_position_y = msg.pose.pose.position.y
        self.pose_orientation_z = msg.pose.pose.orientation.z
        self.pose_orientation_w = msg.pose.pose.orientation.w

        yawcalc1 = 2 * (self.pose_orientation_w * self.pose_orientation_z)
        yawcalc2 = 1 - (2 * (self.pose_orientation_z * self.pose_orientation_z))
        yaw = math.atan2(yawcalc1, yawcalc2)
        self.poses_dict["pose " + str(self.pose_no)] = [self.pose_position_x, self.pose_position_y, yaw]
        self.get_logger().info("Written pose " + str(self.pose_no))
        self.pose_no += 1

        self.pose_array_pub.publish(self.posearray)

    def write_to_file(self):
        loop = True
        while loop:
            user_input = input("End waypoint making? y \nRemove previous point? n\n").lower()
            if user_input == "y":
                self.posearray.poses = []
                self.pose_array_pub.publish(self.posearray)
                loop = False
            if user_input == 'n':
                if not self.posearray.poses:
                    self.get_logger().info('no points recorded yet')
                else:
                    self.get_logger().info('deleted previously marked point')
                    self.posearray.poses.pop()
                    self.pose_no -= 1
                    if self.pose_no < 1:
                        self.pose_no = 1
                    self.poses_dict["pose " + str(self.pose_no)].pop()
                self.pose_array_pub.publish(self.posearray)

        file_name = input("Name of file?")
        with open(file_name+'.txt', 'w') as file:
            for X in range(1, self.pose_no):
                file.write("    pose_"+ str(X) + ':\n' + '    - [ '+ str(self.poses_dict["pose " + str(X)][0]) + ', ' + str(self.poses_dict["pose " +str(X)][1]) + ',  0.000]\n' + '    - [ 0.000, 0.000, ' + str(self.poses_dict["pose " +str(X)][2]) + ']\n    - [0.0]\n\n')

        self.get_logger().info("Written all Poses to " + file_name + '.txt' " file")

def main(args=None):
    rclpy.init(args=args)
    save_poses = SavePoses()
    rclpy.spin(save_poses)

    save_poses.destroy_node()
    rclpy.shutdown()

main()