import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from gpiozero import Servo

class LidarMotor(Node):

    def __init__(self):
        super().__init__('lidar_motor')

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        #self.laser_range = np.array([])


    def scan_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(laser_range)
        # log the info
        self.get_logger().info('Shortest distance at %i degrees' % lr2i)
#        self.get_logger().info('Shortest dist: %f' % laser_range[lr2i])
        self.handleData(laser_range[lr2i])

        

    def handleData(self,min):
        
        if min<1:
            print('less!')

def main(args=None):
    rclpy.init(args=args)


    lidar_motor = LidarMotor()
    servo = Servo(17)
    rclpy.spin(lidar_motor)

#    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
