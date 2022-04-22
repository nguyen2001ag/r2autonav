import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import time
import busio
import board
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

# constants
rotatechange = 0.1
speedchange = 0.05

go_straight = 0.2

heat_threshold = 35.0
firing_threshold = 40.0


class Sensor(Node):
    def __init__(self):
        super().__init__('Sensor')
        self.nfc_pub = self.create_publisher(
            Int8, 'nfc', 10)

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #####################################################
        #                   PINOUT                          #
        # GPIO 6: DC Motor driver Enable                    #
        # GPIO 12 (PWM1) : DC Motor1                        #
        # GPIO 13 (PWM1) : DC Motor2                        #
        # GPIO 22, 23, 24, 27: Stepper Motor Control        #
        # GPIO 17: Button                                   #          
        # GPIO 2, 3: SDA, SCL (I2C) for IR camera           #
        # GPIO 10, 09, 11, 08: MOSI, MISO, SCK, SS for NFC  #
        # GPIO 25: RST for NFC RC522                        #                     
        #                                                   #
        #####################################################

        GPIO.setmode(GPIO.BCM)
        self.nfc_found = False

        self.reader = SimpleMFRC522()
        self.get_logger().info("Setup the NFC Reader")

    def timer_callback(self):    
        if self.nfc_found:
            self.get_logger().info("NFC Tag found, Changing to next state")
            self.nfc_found = True
            self.stopbot()
            self.fsm = 2
            msg = Int8()
            msg.data = 1
            self.nfc_pub.publish(msg)          # Publish 1, indicating receive NFC signal
        else:
            twist = Twist()
            twist.linear.x = go_straight
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1)
            self.readRFID()
            self.get_logger().info("Reading NFC tag")
        
    def readRFID(self):
        id, data = self.reader.read_no_block()
        if id:
            self.get_logger().info("Read NFC with ID : %s" % str(id))
            self.nfc_found = True
            # self.stopbot()
        # print("Reading NFC")

    
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
   



def main(args=None):
    rclpy.init(args=args)
    turtle = Sensor()
    try:
        # rclpy.init(args=args)
        # thermalcamera = Sensor()
        # thermalcamera.hasFoundTarget()
        # rclpy.spin(thermalcamera)
        rclpy.spin(turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        print("Reached exception")
        turtle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()      