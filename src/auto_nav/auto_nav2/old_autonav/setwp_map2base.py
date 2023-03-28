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

# constants
rotatechange = 0.1
speedchange = 0.05
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

    return roll_x, pitch_y, yaw_z # in radians


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
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        # create subscription to track occupancy
        self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

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

    def map2base_callback(self, msg):
        orientation_quat =  msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
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
            existing_waypoints[table_number] = [{"x":self.x_coord, "y":self.y_coord}]
        else:
            existing_waypoints[table_number].append({"x":self.x_coord, "y":self.y_coord})

        # writing to json file 
        f.seek(0)
        json.dump(existing_waypoints, f, indent = 4)
        f.close()

    
    def auto_navigate(self):
        target_table = int(input("Enter an integer from 1-6: "))
        while target_table<1 or target_table>6:
            target_table = int(input("Enter an integer from 1-6 only: "))

        current_yaw = math.degrees(self.yaw)
        self.get_logger().info('Starting auto-navigation: X:%f|Y:%f|Facing:%f ' % (self.x_coord,self.y_coord,current_yaw))
        
        # open json files containing waypoints and deserialise to dict
        wp_file = "waypoints.json"
        f = open(wp_file, 'r+')
        existing_waypoints = json.load(f)
        self.get_logger().info("hi1")

        # printing all waypoints
        for i in existing_waypoints.keys():
            self.get_logger().info(i)

        target_waypoints = existing_waypoints[str(target_table)]
        self.get_logger().info("hi2")

        # target_waypoints is an array of dict
        count = 1
        for item in target_waypoints:
            # visiting every waypoint
            curr = (self.x_coord,self.y_coord)
            self.get_logger().info("curr x: %f curr y: %f"%(curr[0],curr[1]))
            # curr_x = self.x_coord
            # curr_y = self.y_coord
            new = [0,0]
            for i in item.values():
                # storing new x and y to travel to
                # obtaining x first
                if (count == 1):
                    count+=1
                    new[0] = i
                    self.get_logger().info("new x: %f"%(new[0]))
                else: 
                    # after both x and y coordinate is obtained
                    count-=1
                    new[1] = i
                    self.get_logger().info("new y: %f"%(new[1]))

                    new = tuple(new)


                    # calculating angle (in radian) to turn, if robot faced "north"
                    norm = distance.euclidean(curr, new)
                    self.get_logger().info("Norm is: %f"%(norm))
                    angle = np.arccos( (new[1] - curr[1]) / distance.euclidean(new, curr))
                    # converting from radian to degree 
                    angle = angle * 180 / PI

                    # calculating actual angle to turn, after taking into account the bot's bearings
                    # make bot turn anticlockwise
                    if (current_yaw) < 0 and (current_yaw) > -180:
                        total_angle = -(current_yaw) - angle
                        self.get_logger().info("Turning by: %f"%(total_angle))
                    
                    else: 
                        # make bot turn clockwise
                        total_angle = current_yaw + angle
                        self.get_logger().info("Turning by: %f"%(total_angle))


                
            # self.get_logger().info("way to target: %d"%(item))
        




    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
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
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

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
