# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# adapted from https://github.com/Shashika007/teleop_twist_keyboard_ros2/blob/foxy/teleop_twist_keyboard_trio/teleop_keyboard.py

# file copied from r2waypointotate.py

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import math
import cmath
import numpy as np
import json
from geometry_msgs.msg import Pose
from scipy.spatial import distance
from httpServer import S
from sensor_msgs.msg import LaserScan
import time
import os

# constants
rotatechange = 0.3
speedchange = 0.15
PI = 3.141592653589793


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


# function to check if keyboard input is a number as
# isnumeric does not handle negative numbers
def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


# class for moving and rotating robot
class Waypoint(Node):
    def __init__(self):
        super().__init__('waypoint')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # create subscription to track occupancy
        self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # initial x and y values to offset
        self.x_initial = 0
        self.y_initial = 0
        self.first_val = True

        self.x_coord = 0
        self.y_coord = 0

        self.obstacle = False
        self.rotate_stuck = False
        # self.special_table = False
        self.docked = True
        # self.special_found = False
        self.og_left_laser = 2.1 # og - left < 400
        self.left_laser = 0
        # self.only_for_special = False

    def scan_callback(self, msg):
        # self.get_logger().info('in scan callback')
        # stop whenever the distance is below threshold
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(laser_range)
        # log the info
        # twist = Twist()

        self.left_laser = laser_range[90]

        # if (self.og_left_laser!=-1):
        #     self.get_logger().info('difference is: %f'%self.og_left_laser-laser_range[90])
        
        # if (self.only_for_special):
        #     self.get_logger().info("Difference...%f"%(self.og_left_laser - self.left_laser))
        #     if (s):
        #         self.get_logger().info("Special is found!")
        #         self.special_found=True

        # if the front of robot senses something in front, stop
        # self.get_logger().info('Dock: %d, Range at [0]: %f, [1]: %f, [359]: %f, [358]: %f' % (self.docked,laser_range[0],laser_range[1],laser_range[358],laser_range[359]))
        if (laser_range[0]<0.2 or laser_range[1]<0.2 or laser_range[359]<0.2 or laser_range[358]<0.2):
            if (not self.docked):
                # self.get_logger().info('STOP!!!!!!!!!! SELF>OBSTACLE TRUE!')
                self.obstacle=True
                self.stop()

        # push to handler for table 6
        # if (self.special_table):
        #     # first value 
        #     self.special_table=False
        #     self.only_for_special = True
        #     self.handle_special()

            

    def map2base_callback(self, msg):
        orientation_quat = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        self.x_coord = msg.position.x
        self.y_coord = msg.position.y

        # if (self.first_val == True):
        #     self.get_logger().info("INITIAL x is %f, y is %f" %(self.x_coord, self.y_coord))
        #     self.x_initial = self.x_coord
        #     self.y_initial = self.y_coord
        #     self.first_val = False

        # self.x_coord = self.x_coord - self.x_initial
        # self.y_coord = self.y_coord - self.y_initial

        # self.get_logger().info("x is %f, y is %f" %(self.x_coord, self.y_coord))

    def set_waypoint(self):

        # open json files containing waypoints and deserialise to dict
        wp_file = "waypoints.json"
        f = open(wp_file, 'r+')
        existing_waypoints = json.load(f)

        # print("existing waypoints", existing_waypoints)

        table_number = input("Enter table number: ")

        # check if key exist. Exist? Append:Create new entry
        if (table_number not in existing_waypoints):
            existing_waypoints[table_number] = [
                {"x": self.x_coord, "y": self.y_coord}]
        else:
            existing_waypoints[table_number].append(
                {"x": self.x_coord, "y": self.y_coord})

        # writing to json file
        f.seek(0)
        json.dump(existing_waypoints, f, indent=4)
        f.close()

    # check quadrant of new point relative to current point
    # north is positive x, west is positive y
    def checkQuadrant(self, curr, new):
        while True:
            if new[0] > curr[0] and new[1] < curr[1]:
                return 1
            elif new[0] > curr[0] and new[1] > curr[1]:
                return 2
            elif new[0] < curr[0] and new[1] > curr[1]:
                return 3
            elif new[0] < curr[0] and new[1] < curr[1]:
                return 4

    def move_to_point(self, curr, new):
        twist = Twist()
        current_yaw = math.degrees(self.yaw)
        norm = distance.euclidean(curr, new)
        self.get_logger().info("Norm is: %f" % (norm))
        target_angle = abs(np.arctan((new[1] - curr[1]) / (new[0] - curr[0])))
        # converting from radian to degree
        target_angle = target_angle * 180 / PI

         # calculating actual angle to turn, after taking into account the bot's bearings
         # make bot turn
        quadrant = self.checkQuadrant(curr, new)
        # self.get_logger().info("new point is in quadrant %d" % (quadrant))

        # switch case to check where new point is relative to old point
        if quadrant == 1:
            target_angle = 0-target_angle
        if quadrant == 2:
            target_angle = 0+target_angle
        if quadrant == 3:
            target_angle = 180-target_angle
        if quadrant == 4:
            target_angle = -180+target_angle

        # self.get_logger().info("Target Angle: %f" % (target_angle))
        final_angle = target_angle-current_yaw
        # self.get_logger().info("Pre Final Angle: %f" % (final_angle))
        if final_angle > 180 or final_angle < -180:  # to avoid turning the long
            final_angle = final_angle - 360

        self.rotatebot(final_angle)
        while (self.rotate_stuck==True):
            self.rotatebot(40)
            self.rotatebot(final_angle)
            self.rotate_stuck=False

        # twist.linear.x += speedchange
        # twist.angular.z = 0.0
        # self.publisher_.publish(twist)


        
        # walk to destination
        self.obstacle=False
        if norm>0.5:
            self.forward()
        else:
            self.forward_slow()
        prev_norm = 999
        recalib_counter=0 

        while (norm > 0.04 and (norm<=prev_norm+0.02)):
            # count+=1
            prev_norm = norm
            # self.get_logger().info("prev norm is: %f" % (prev_norm))
            norm = distance.euclidean(curr, new)
            # self.get_logger().info("norm is: %f" % (norm))
            rclpy.spin_once(self)
            curr = (self.x_coord, self.y_coord)
            # self.get_logger().info("curr x: %f curr y: %f" %
            #                         (curr[0], curr[1]))

            if self.obstacle:
                # count=0
                self.stop()
                self.get_logger().info("oKOOok i stop")
                # self.backward()
                break
            
            recalib_counter+=1   
            if recalib_counter>30 and norm > 0.5 and not self.obstacle:
                self.get_logger().info("recalib, continue walking to x:%f y:%f"%(new[0],new[1]))
                self.move_to_point(curr,new)
    
        # self.get_logger().info("norm-prev_norm: %d, self.obstacle %d"%(norm-prev_norm, self.obstacle))
        norm = distance.euclidean(curr, new)
        # if stop but norm too big, recallibrate
        if (norm > 0.08 and not self.obstacle):
            self.get_logger().info("norm:%f is too far!, continue walking to x:%f y:%f"%(norm,new[0],new[1]))
            curr = (self.x_coord, self.y_coord)
            self.move_to_point(curr,new)

        # elif(norm>0.5 and self.obstacle):
        #     self.move_to_point(curr,new)    

        current_yaw = math.degrees(self.yaw)
        self.get_logger().info("MOVEMENT ENDS!")
        # stop the bot after it has finished walking
        self.stop()
        # self.recallibrate = False

    def auto_navigate(self):
        post_file = "serverData.json"
        file_size = os.stat(post_file).st_size
        while True:

            self.get_logger().info("Please input table number into keypad: ")
            # getting table number by checking if there is a new entry in serverData.json
            while True: 
                rclpy.spin_once(self)
                self.obstacle=False
                current_size = os.stat(post_file).st_size
                if current_size>file_size:
                    file_size=current_size
                    self.docked=True    

                    # if there is new entry, break out of the while loop
                    break
            
            # open file and read the latest value
            f = open(post_file, 'r') 
            existing_data = json.load(f)
            target_table = existing_data["postD"][-1] 

            self.get_logger().info("Walking to table: %s"%target_table)

            self.backward()
            self.obstacle=False
            self.face_back()
            self.docked=False
            self.get_logger().info('Leaving dock...')

            # twist = Twist()
            # target_table = int(input("Enter an integer from 1-6: "))
            # while target_table < 1 or target_table > 6:
            #     target_table = int(input("Enter an integer from 1-6 only: "))
            
            # if target_table==6:
            #     self.special_table = True

            current_yaw = math.degrees(self.yaw)
            self.get_logger().info('Starting auto-navigation: X:%f|Y:%f|Facing:%f ' %
                                (self.x_coord, self.y_coord, current_yaw))

            # open json files containing waypoints and deserialise to dict
            wp_file = "waypoints.json"
            f = open(wp_file, 'r+')
            existing_waypoints = json.load(f)

            # printing all waypoints
            # for i in existing_waypoints.keys():
            #     self.get_logger().info(i)

            target_waypoints = existing_waypoints[str(target_table)]

            # target_waypoints is an array of dict
            count = 1
            for item in target_waypoints:
                # visiting every waypoint
                curr = (self.x_coord, self.y_coord)
                self.get_logger().info("curr x: %f curr y: %f" %
                                    (curr[0], curr[1]))
                # curr_x = self.x_coord
                # curr_y = self.y_coord
                new = [0, 0]
                for i in item.values():
                    # storing new x and y to travel to
                    # obtaining x first
                    if (count == 1):
                        count += 1
                        new[0] = i
                        self.get_logger().info("Obtaining new x: %f" % (new[0]))
                    else:
                        # after both x and y coordinate is obtained
                        count -= 1
                        new[1] = i
                        self.get_logger().info("Obtaining new y: %f. Facing: %f" % (new[1],current_yaw))
                        new = tuple(new)
                        self.move_to_point(curr,new)

            self.get_logger().info("Reached the point %f, %f"%(new[0],new[1]))
            time.sleep(5)

            # if special table 6 is called
            if target_table=="6":
                self.get_logger().info("Starting special")
                self.face_back()
                # self.special_table=True
                self.handle_special()
                # after reaching entrance of table 6
                rclpy.spin_once(self)

            # walking back to dispenser
            dock_waypoints = existing_waypoints["0"]
            dock_count = 1

            # # when microswitch detect can has been lifted, dock. Dock.
            self.get_logger().info("Proceeding to dock!")

            for item in dock_waypoints:
                # visiting every waypoint
                curr = (self.x_coord, self.y_coord)
                self.get_logger().info("curr x: %f curr y: %f" %
                                    (curr[0], curr[1]))
                # curr_x = self.x_coord
                # curr_y = self.y_coord
                new = [0, 0]
                for i in item.values():
                    # storing new x and y to travel to
                    # obtaining x first
                    if (dock_count == 1):
                        dock_count += 1
                        new[0] = i
                        # self.get_logger().info("Obtaining new x: %f" % (new[0]))
                    else:
                        # after both x and y coordinate is obtained
                        dock_count -= 1
                        new[1] = i
                        # self.get_logger().info("Obtaining new y: %f" % (new[1]))

                        new = tuple(new)
                        self.move_to_point(curr,new)

            self.get_logger().info('Face front and then Walking straight to dispenser...:%f ' % (current_yaw))
            self.face_front() 
            self.obstacle=False
            self.forward_slow()
            # for i in range(10):
            #     rclpy.spin_once(self)
            
        
    

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # self.get_logger().info('Current: %f' % )
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        # self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        self.get_logger().info('Rotating from: %f to: %f' % (math.degrees(current_yaw),math.degrees(cmath.phase(c_target_yaw))))
    
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * speedchange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        stuck_count = 0
        while (c_change_dir * c_dir_diff > 0):
            stuck_count+=1
            # if (stuck_count%10==0):
            #     self.get_logger().info("stuck count: %d" % stuck_count)
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

            # if stuck for too long, break
            if stuck_count>600:
                self.rotate_stuck=True
                self.get_logger().info("STUCK!")
                break
    
        self.rotate_stuck=False

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
    
    def handle_special(self):
        # detection of special table
        self.get_logger().info('Starting detection of special table')
        self.get_logger().info('DIFF: %f' %(self.og_left_laser - self.left_laser))
        self.forward_slow()
        while (self.og_left_laser - self.left_laser<0.4):
            self.get_logger().info('DIFF: %f' %(self.og_left_laser - self.left_laser))
            # self.get_logger().info()
            self.get_logger().info("Checking on left....%f"%self.left_laser)  
            rclpy.spin_once(self)
        self.get_logger().info('DIFF FINISH: %f' %(self.og_left_laser - self.left_laser))
        for i in range(10):
            pass

        self.get_logger().info('Located special table!')
        self.stop()
        self.rotatebot(90)
        self.forward_slow()
        # forward until it is stopped by laser callback
        while not self.obstacle:
            rclpy.spin_once(self)

        self.docked=True
        self.get_logger().info('Reached special table!')
        # go back to base by reverse waypoints
        self.backward()
        self.face_back()
        self.docked=False
        self.obstacle = False
        wp_file = "waypoints.json"
        f = open(wp_file, 'r+')
        existing_waypoints = json.load(f)
        target_waypoints = existing_waypoints["6r"] # table 6 return journey

        count = 1
        for item in target_waypoints:
            # visiting every waypoint
            curr = (self.x_coord, self.y_coord)
            self.get_logger().info("curr x: %f curr y: %f" %
                                (curr[0], curr[1]))
            # curr_x = self.x_coord
            # curr_y = self.y_coord
            new = [0, 0]
            for i in item.values():
                # storing new x and y to travel to
                # obtaining x first
                if (count == 1):
                    count += 1
                    new[0] = i
                    self.get_logger().info("Obtaining new x: %f" % (new[0]))
                else:
                    # after both x and y coordinate is obtained
                    count -= 1
                    new[1] = i
                    self.get_logger().info("Obtaining new y: %f." % (new[1]))
                    new = tuple(new)
                    self.move_to_point(curr,new)
        


    def face_front(self):
        current_yaw = math.degrees(self.yaw)
        while (current_yaw > 1 or current_yaw<-1):
            self.rotatebot(-current_yaw)
            current_yaw = math.degrees(self.yaw)

    def face_back(self):
        current_yaw = math.degrees(self.yaw)
        while ((current_yaw < 179 and current_yaw>0) or (current_yaw<0 and current_yaw>-179)):
            self.rotatebot(180-current_yaw)
            current_yaw = math.degrees(self.yaw)


# function to read keyboard input
    def readKey(self):
        twist = Twist()
        try:
            while True:
                # get keyboard input
                rclpy.spin_once(self)
                cmd_char = str(input("Keys w/x/a/d -/+int s/p/auto: "))

                # use our own function isnumber as isnumeric
                # does not handle negative numbers
                if isnumber(cmd_char):
                    # rotate by specified angle
                    self.rotatebot(int(cmd_char))
                else:
                    # print("callback is called: ")
                    # check which key was entered
                    if cmd_char == 's':
                        # stop moving
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    elif cmd_char == 'w':
                        # move forward
                        twist.linear.x += speedchange
                        twist.angular.z = 0.0
                    elif cmd_char == 'x':
                        # move backward
                        twist.linear.x -= speedchange
                        twist.angular.z = 0.0
                    elif cmd_char == 'a':
                        # turn counter-clockwise
                        twist.linear.x = 0.0
                        twist.angular.z += rotatechange
                    elif cmd_char == 'd':
                        # turn clockwise
                        twist.linear.x = 0.0
                        twist.angular.z -= rotatechange
                    elif cmd_char == 'p':
                        # set waypoint
                        self.set_waypoint()
                    elif cmd_char == 'auto':
                        self.auto_navigate()
                    elif cmd_char=='ff':
                        self.face_front()
                    elif cmd_char=='b':
                        self.backward()
                    elif cmd_char=='fs':
                        self.forward_stop()
                    elif cmd_char=='fb':
                        self.face_back()

                    # start the movement
                    self.publisher_.publish(twist)

        except Exception as e:
            print(e)

            # Ctrl-c detected
        finally:
            # stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def forward(self):
        twist = Twist()
        twist.linear.x += speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    
    def forward_stop(self):
        twist = Twist()
        twist.linear.x += speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stop()

    def backward(self):
        twist = Twist()
        twist.linear.x -= speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stop()
    def forward_slow(self):
        twist = Twist()
        twist.linear.x += 0.05
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    def stop(self):
        twist = Twist()
        twist.linear.x += 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()
    waypoint.readKey()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
