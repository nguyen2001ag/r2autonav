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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8, Bool, String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import scipy.stats
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.1
speedchange = 0.1
occ_bins = [-1, 0, 70, 100]  # Setting threshold between occupied and unoccupied cell
stop_distance = 0.25
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# Debug flag to print output
debug = True



# Some variable to tune your bot
stopping_time_in_seconds = 600 #seconds
initial_direction = 0  # "Front", "Left", "Right", "Back"
direction_dict = {0: "Front" , 1: "Left", 2: "Right", 3: "Back"}
back_angles = range(150, 211)

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

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.target_subscription = self.create_subscription(
            Int8,
            'hot_target',
            self.target_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.target_subscription  # prevent unused variable warning

        self.nfc_subscription = self.create_subscription(
            Int8,
            'nfc',
            self.nfc_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.nfc_subscription  # prevent unused variable warning

        self.button_subscription = self.create_subscription(
            Int8,
            'button',
            self.button_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.button_subscription  # prevent unused variable warning

        self.actuation_publisher = self.create_publisher(Int8, "actuation", 10)

        self.state = "Standing still"
        # Task flag
        self.is_finished_shooting = False
        self.is_detect_target = False

        ############################################################
        ##                                                        ##
        ##              Define a Finite State Machine             ##
        ## 0 : Exploration to find NFC tag                        ## 
        ## 1 : Found NFC tag, waiting for ping pong ball          ##  
        ## 2 : Loaded with ping pong ball, finding hot target     ##
        ## 3 : Found hot targer, aiming and fire                  ##  
        ## 4 : Shot at hot target, map the rest of the map        ##
        ##                                                        ##  
        ############################################################
        self.fsm = 0

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        # self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return

        self.x = trans.transform.translation.x
        self.y = trans.transform.translation.y
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        self.roll, self.pitch, self.yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def target_callback(self, msg):
        if msg.data == 1 and (self.fsm == 2):
            self.fsm == 3
        elif msg.data == 2 and self.fsm == 3:
            self.fsm = 4

    def nfc_callback(self, msg):
        if msg.data == 1 and self.fsm == 0:
            self.fsm == 1

    def button_callback(self,msg):
        if msg.data == 1 and self.fsm == 1:
            self.fsm = 2

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
        twist.angular.z = c_change_dir * rotatechange
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
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
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


    def wall_following(self):
        self.get_logger().info('Wall following for exploration')
        left_front = self.laser_range[10:45]
        right_front = self.laser_range[315:351]
        self.front_d = np.nan_to_num(self.laser_range[0], copy=False, nan=5)   # Choosing nan = 5 because, maximum lidar range is 3.5
        self.leftfront_d = np.nan_to_num(left_front, copy=False, nan=5)
        self.rightfront_d = np.nan_to_num(right_front, copy=False, nan=5)
        self.leftfront_d = np.max(self.leftfront_d)
        self.rightfront_d = np.max(self.rightfront_d)

        self.get_logger().info('Front Distance: %f' % self.front_d)
        self.get_logger().info('Front Left Distance: %f' % self.leftfront_d)
        self.get_logger().info('Front Right Distance: %f' % self.rightfront_d)

        wall_threshold = 0.3
        # Set turning speeds (to the left) in rad/s

        # These values were determined by trial and error.
        self.rotate_fast = 0.75 # Use when turning away
        self.rotate_slow = 0.40 # Use when searching for wall
        # Set movement speed
        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print("Im here")

        if self.leftfront_d > wall_threshold and self.front_d > wall_threshold and self.rightfront_d > wall_threshold:
            self.state = "search for wall"
            msg.linear.x = speedchange
            msg.angular.z = -self.rotate_slow  # turn right to find wall

        elif self.leftfront_d > wall_threshold and self.front_d < wall_threshold and self.rightfront_d > wall_threshold:
            self.state = "turn left"
            msg.angular.z = self.rotate_fast

        elif (self.leftfront_d > wall_threshold and self.front_d > wall_threshold and self.rightfront_d < wall_threshold):
            if (self.rightfront_d < 0.25): # Avoid crashing the wall
                self.state = "turn left"
                msg.linear.x = speedchange
                msg.angular.z = self.rotate_fast
            else:
                # Go straight ahead
                self.state = "follow wall"
                msg.linear.x = speedchange

        elif self.leftfront_d < wall_threshold and self.front_d > wall_threshold and self.rightfront_d > wall_threshold:
            self.state = "search for wall"
            msg.linear.x = speedchange
            msg.angular.z = -self.rotate_slow  # turn right to find wall

        elif self.leftfront_d > wall_threshold and self.front_d < wall_threshold and self.rightfront_d < wall_threshold:
            self.state = "turn left"
            msg.angular.z = self.rotate_fast

        elif self.leftfront_d < wall_threshold and self.front_d < wall_threshold and self.rightfront_d > wall_threshold:
            self.state = "turn left"
            msg.angular.z = self.rotate_fast

        elif self.leftfront_d < wall_threshold and self.front_d < wall_threshold and self.rightfront_d < wall_threshold:
            self.state = "turn left"
            msg.angular.z = self.rotate_fast

        elif self.leftfront_d < wall_threshold and self.front_d > wall_threshold and self.rightfront_d < wall_threshold:
            self.state = "search for wall"
            msg.linear.x = speedchange
            msg.angular.z = -self.rotate_slow  # turn right to find wall

        else:
            pass

        if debug:
            self.get_logger().info("The turtlebot is: %s " % self.state)

        # Send velocity command to the robot
        self.publisher_.publish(msg)
        rclpy.spin_once(self)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def firstMove(self):
        self.get_logger().info('Moving backward to find wall')
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        lrback = (self.laser_range[back_angles] < float(0.40)).nonzero()
        self.publisher_.publish(twist)
        while len(lrback[0]) <= 0:
            time.sleep(1)
            twist.linear.x = -speedchange
            twist.angular.z = 0.0
            rclpy.spin_once(self)
            lrback = (self.laser_range[back_angles] < float(0.40)).nonzero()
            self.publisher_.publish(twist)
        self.stopbot()
        self.rotatebot(-90)  # Since it is at the back of the turtlebot, turn right to do right wall following
        self.stopbot()


    def mover(self):
        try:
            rclpy.spin_once(self)

            # ensure that we have a valid lidar data before we start wall follow logic
            while (self.laser_range.size == 0):
                self.get_logger().info("Getting values from callbacks")
                rclpy.spin_once(self)

            start_time = time.time()

            # initial move to find the appropriate wall to follow
            # self.firstMove()
            # start wall follow logic
            if self.fsm == 0:
                self.wall_following()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > stopping_time_in_seconds:
                        print(
                            "Specified time has passed. Automatically shutting down.")
                        break

                    if self.fsm == 0:
                        self.wall_following()
                    elif self.fsm == 1:        ## Wait for button
                        self.stopbot()
                    elif self.fsm == 2:
                        self.wall_following()
                    elif self.fsm == 3:
                        self.actuation_publisher.publish(1)
                    elif self.fsm == 4:
                        self.wall_following()
                    # Continue wall following until detect target

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
            # save map
            # cv2.imwrite('mazemapfinally.png', self.occdata)


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
