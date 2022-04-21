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
wall_following_dir = 1
wall_following_dir_dic = {1 : "Right", -1 : "Left"}
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
        ## 0 : Mapping the maze                                   ##
        #  1 : Exploration to find NFC tag                        ## 
        ## 2 : Found NFC tag, waiting for ping pong ball          ##  
        ## 3 : Loaded with ping pong ball, finding hot target     ##
        ## 4 : Found hot targer, aiming and fire                  ##  
        ## 5 : Shot at hot target, end program                    ##
        ##                                                        ##  
        ############################################################
        self.fsm = 3

        # These values were determined by trial and error.
        self.rotate_fast = 0.65 # Use when turning away
        self.rotate_slow = 0.40 # Use when searching for wall
        self.follow_wall_speed = 0.2

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
        if msg.data == 1 and (self.fsm == 3):
            self.fsm = 4
        elif msg.data == 2 and self.fsm == 4:
            self.fsm = 5

    def nfc_callback(self, msg):
        if msg.data == 1 and self.fsm == 1:
            self.fsm = 2

    def button_callback(self,msg):
        if msg.data == 1 and self.fsm == 2:
            self.fsm = 3

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


    def wall_following(self, direction):
        self.get_logger().info('Wall following for exploration')
        if direction == 1:
            other_front = self.laser_range[45]
            wall_front = self.laser_range[315]
        else:
            other_front = self.laser_range[315]
            wall_front = self.laser_range[45]
        # self.front_d_r = np.nan_to_num(self.laser_range[330:360], copy=False, nan=5)   # Choosing nan = 5 because, maximum lidar range is 3.5
        # self.front_d_l = np.nan_to_num(self.laser_range[0:30], copy=False, nan=5)   # Choosing nan = 5 because, maximum lidar range is 3.5
        self.otherfront_d = np.nan_to_num(other_front, copy=False, nan=5)
        self.wallfront_d = np.nan_to_num(wall_front, copy=False, nan=5)
        # self.front_d_r = np.min(self.front_d_r)
        # self.front_d_l = np.min(self.front_d_l)
        # self.front_d = min(self.front_d_l, self.front_d_r)

        # self.otherfront_d = np.min(self.otherfront_d)
        # self.wallfront_d = np.min(self.wallfront_d)
        self.front_d = np.nan_to_num(self.laser_range[0], copy=False, nan=5)

        self.get_logger().info('Front Distance: %f' % self.front_d)
        self.get_logger().info('Front Left Distance: %f' % self.otherfront_d)
        self.get_logger().info('Front Right Distance: %f' % self.wallfront_d)

        wall_threshold = 0.35
        # Set turning speeds (to the left) in rad/s


        if (self.otherfront_d > wall_threshold and self.front_d > wall_threshold and self.wallfront_d < wall_threshold): 
            if (self.wallfront_d < 0.22): #0.22: # Avoid crashing the wall, probably decrease this value
                self.state = "turn left"
                self.turn(direction)
            else: 
                # Go straight ahead
                self.state = "follow wall"
                self.follow_wall(direction)


        elif self.otherfront_d > wall_threshold and self.front_d < wall_threshold and self.wallfront_d > wall_threshold: ##left/right far from wall, front far from wall, right/ left near to wall
            self.state = "turn left"
            self.turn(direction)

        elif self.otherfront_d > wall_threshold and self.front_d > wall_threshold and self.wallfront_d > wall_threshold: #all sides far from wall
            self.state = "search for wall"
            self.search_for_wall(direction)

        elif self.otherfront_d < wall_threshold and self.front_d > wall_threshold and self.wallfront_d > wall_threshold: #other side far from wall
            self.state = "search for wall"
            self.search_for_wall(direction)

        elif self.otherfront_d > wall_threshold and self.front_d < wall_threshold and self.wallfront_d < wall_threshold: #front near to wall, either left or right near to wall => at corner
            self.state = "turn left"
            self.turn(direction, 0.9)

        elif self.otherfront_d < wall_threshold and self.front_d < wall_threshold and self.wallfront_d > wall_threshold: #also at corner
            self.state = "turn left"
            self.turn(direction, 0.9)

        elif self.otherfront_d < wall_threshold and self.front_d < wall_threshold and self.wallfront_d < wall_threshold: #all 3 sides near wall 
            self.state = "turn left"
            self.turn(direction)

        elif self.otherfront_d < wall_threshold and self.front_d > wall_threshold and self.wallfront_d < wall_threshold: #left and right near wall
            self.state = "search for wall"
            self.search_for_wall(direction)

        else:
            pass

        if debug:
            self.get_logger().info("The turtlebot is: %s " % self.state)

        rclpy.spin_once(self)


    def follow_wall(self, direction: int):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        msg.linear.x = self.follow_wall_speed
        self.publisher_.publish(msg)

    ## direction == 1 -> turn left (Right-wall following)
    ## direction == -1 -> turn right (Left-wall following)    
    def turn(self, direction : int, baseline = 0.4): 
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        msg.angular.z = direction * 0.75 #0.75
        self.publisher_.publish(msg)

    ## direction == 1 -> turn right (Right-wall following)
    ## direction == -1 -> turn left (Left-wall following) 
    def search_for_wall(self, direction: int, baseline = 0.75):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        # msg.linear.x = speedchange
        # msg.angular.z = direction * -self.rotate_slow  
        msg.linear.x = 0.15 #0.15
        msg.angular.z = direction * - 0.6 #0.6
        self.publisher_.publish(msg)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def firstMove(self, direction):
        # if initial_direction == 3:
        #     self.get_logger().info("Going back")
        # elif initial_direction == 2:
        #     self.get_logger().info("Going right")
        #     self.rotatebot(90)
        # elif initial_direction == 1:
        #     self.get_logger().info("Going right")
        #     self.rotatebot(-90)
        # elif initial_direction == 0:
        #     self.get_logger().info("Going Forward")
        #     self.rotatebot(180)

        self.get_logger().info('Moving backward to find wall')
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        # print("im not here")
        lrback = (self.laser_range[back_angles] < float(0.50)).nonzero()
        # print("im here")
        print(lrback)
        self.publisher_.publish(twist)
        while len(lrback[0]) <= 0:
            time.sleep(1)
            twist.linear.x = -speedchange
            twist.angular.z = 0.0
            rclpy.spin_once(self)
            lrback = (self.laser_range[back_angles] < float(0.50)).nonzero()
            self.publisher_.publish(twist)
        self.stopbot()

        ## direction == 1 -> turn right (Right-wall following)
        ## direction == -1 -> turn left (Left-wall following) 
        self.rotatebot(direction * -90)  # Since it is at the back of the turtlebot, turn right to do right wall following
        self.stopbot()


    def mover(self):
        try:
            rclpy.spin_once(self)

            shot = False

            # ensure that we have a valid lidar data before we start wall follow logic
            while (self.laser_range.size == 0):
                self.get_logger().info("Getting values from callbacks")
                rclpy.spin_once(self)

            start_time = time.time()

            # initial move to find the appropriate wall to follow
            # start wall follow logic
            # while self.fsm == 0:
            #     # self.firstMove(wall_following_dir)
            #     self.wall_following(wall_following_dir)

            while rclpy.ok():
                if self.laser_range.size != 0:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > stopping_time_in_seconds:
                        print(
                            "Specified time has passed. Automatically shutting down.")
                        break

                    if self.fsm == 0:
                        self.wall_following(wall_following_dir)
                        # if self.closure(self.occdata) or elapsed_time > 240:
                        if elapsed_time > 30:
                            self.fsm = 1
                            msg = Int8()
                            msg.data = 1
                            self.actuation_publisher.publish(msg)
                            self.get_logger().warn("Changing to STATE 1: Finding NFC")
                    elif self.fsm == 1:        ## Finding NFC
                        self.wall_following(wall_following_dir)
                        self.get_logger().warn("At STATE 1: Finding NFC") 
                    elif self.fsm == 2:        ## Wait for button
                        self.stopbot()
                        self.get_logger().warn("At STATE 2: waiting for balls") 
                        # time.sleep(10)
                        # self.fsm = 3
                    elif self.fsm == 3:        ## After receive button, proceed to find hot_target
                        self.wall_following(wall_following_dir)
                        self.get_logger().warn("At STATE 3: Finding hot target") 
                    elif self.fsm == 4:        ## Found hot_target, shooot
                        self.get_logger().warn("At STATE 4: Sending command to shoot at hot target") 
                        msg = Int8()
                        if not shot:
                            msg.data = 2
                            self.actuation_publisher.publish(msg)
                            shot = True
                    elif self.fsm == 5:        ## Finish
                        self.wall_following(wall_following_dir)
                        # self.stopbot()
                    # Continue wall following until detect target

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
            self.stopbot()

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
            # save map
            cv2.imwrite('map.png', self.occdata)

    def closure(self, occdata):
        # This function checks if mapdata contains a closed contour. The function
        # assumes that the raw map data from SLAM has been modified so that
        # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
        # values go from 1 to 101.

        # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
        # closed contours have larger areas than arc length, while open contours have larger
        # arc length than area. But in my experience, open contours can have areas larger than
        # the arc length, but closed contours tend to have areas much larger than the arc length
        # So, we will check for contour closure by checking if any of the contours
        # have areas that are more than 10 times larger than the arc length
        # This value may need to be adjusted with more testing.
        ALTHRESH = 10
        # We will slightly fill in the contours to make them easier to detect
        DILATE_PIXELS = 3
        mapdata = occdata
        # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
        # and other positive values up to 101 (occupied)
        # so we will apply a threshold of 2 to create a binary image with the
        # occupied pixels set to 255 and everything else is set to 0
        # we will use OpenCV's threshold function for this
        ret, img2 = cv2.threshold(mapdata, 2, 255, 0)
        # we will perform some erosion and dilation to fill out the contours a
        # little bit
        element = cv2.getStructuringElement(
            cv2.MORPH_CROSS, (DILATE_PIXELS, DILATE_PIXELS))
        # img3 = cv2.erode(img2,element)
        img4 = cv2.dilate(img2, element)
        # use OpenCV's findContours function to identify contours
        # OpenCV version 3 changed the number of return arguments, so we
        # need to check the version of OpenCV installed so we know which argument
        # to grab
        fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (major, minor, _) = cv2.__version__.split(".")
        if(major == '3'):
            contours = fc[1]
        else:
            contours = fc[0]
        # find number of contours returned
        lc = len(contours)
        # rospy.loginfo('# Contours: %s', str(lc))
        # create array to compute ratio of area to arc length
        cAL = np.zeros((lc, 2))
        for i in range(lc):
            cAL[i, 0] = cv2.contourArea(contours[i])
            cAL[i, 1] = cv2.arcLength(contours[i], True)

        # closed contours tend to have a much higher area to arc length ratio,
        # so if there are no contours with high ratios, we can safely say
        # there are no closed contours
        cALratio = cAL[:, 0]/cAL[:, 1]
        # rospy.loginfo('Closure: %s', str(cALratio))
        if np.any(cALratio > ALTHRESH):
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    try:
        auto_nav = AutoNav()
        auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        auto_nav.stopbot()
        auto_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
