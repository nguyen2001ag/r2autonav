import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import time
import busio
import board
import adafruit_amg88xx
from rclpy.qos import qos_profile_sensor_data
# from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO


class amg(Node):
    def __init__(self):
        # i2c = busio.I2C(board.SCL, board.SDA)
        # amg = adafruit_amg88xx.AMG88XX(i2c)
        super().__init__('thermalcamera')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

    def detect(self):
        while True:
            for row in self.amg.pixels:
                # Pad to 1 decimal place
                print(["{0:.1f}".format(temp) for temp in row])
                print("")
            print("\n")
            time.sleep(1)

def main(args = None):
    try:
        rclpy.init(args=args)
        thermalcamera = amg()
        thermalcamera.detect()
    except KeyboardInterrupt:
        thermalcamera.destroy_node()
        rclpy.shutdown()
    
    # thermalcamera.destroy_node()
    # rclpy.shutdown()

if __name__ == "main":
    main()
