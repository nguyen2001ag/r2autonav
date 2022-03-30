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

from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = "Min range is " + str(min_range)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)


class Scanner(Node):
    min_range = 0

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.minimal_publisher = MinimalPublisher()


    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(laser_range)

        # log the info
        #self.get_logger().info('Shortest distance at %i degrees' % lr2i)
        global min_range 
        min_range = str(laser_range[lr2i])
        print(min_range) 
        rclpy.spin(self.minimal_publisher)
        


def main(args=None):

    rclpy.init(args=args)

    scanner = Scanner()


    rclpy.spin(scanner)
    min_range = scanner.min_range
    scanner.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
