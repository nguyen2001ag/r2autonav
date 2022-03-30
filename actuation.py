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

heat_threshold = 32.0
firing_threshold = 35.0

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


class Sensor(Node):
    def __init__(self):
        super().__init__('Sensor')
        self.nfc_pub = self.create_publisher(
            Int8, 'nfc', 10)
        
        self.hot_pub = self.create_publisher(
            Int8, 'hot_target', 10)

        self.button_pub = self.create_publisher(
            Int8, 'button', 10)
        

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        
        self.actuation_subscriber = self.create_subscription(Int8, 'actuation', self.actuation_callback, 10)
        self.actuation_subscriber

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #############################################
        #                   PINOUT                  #
        # GPIO 22: DC Motor driver Enable           #
        # GPIO 23 (PWM1) : DC Motor1                #
        # GPIO 24 (PWM1) : DC Motor2                #
        # Stepper Motor :                           #
        # GPIO 2: Button                            #          
        #                         
        #############################################
        self.dc_driver_en = 22
        self.motor1_pin = 23
        self.motor2_pin = 24
        self.stepper_pin = 26
        self.button_pin = 2

        self.reader = SimpleMFRC522()

        GPIO.setmode(GPIO.BCM)

        # DC motor setup
        GPIO.setup(self.dc_driver_en, GPIO.OUT)
        GPIO.setup(self.motor1_pin, GPIO.OUT)
        GPIO.setup(self.motor2_pin, GPIO.OUT)


        # Button setup
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.get_logger().info("Setup the DC")

    # targeting_status callback function to stop wallfollower logic when target is detected
    def timer_callback(self):
        if self.readRFID():
            self.nfc_pub.publish(1)          # Publish 1, indicating receive NFC signal
        if self.hasFoundTarget():
            self.hot_pub.publish(1)          # Publish 1, indicating have detected hot signal
        if self.readButton():
            self.button_pub.publish(1)       # Publish 1, indicating user has press the button
        

    def actuation_callback(self, msg):
        flag = msg.data
        if flag == 1:
            # centre the target
            self.makeCenterTarget()

            while not self.lock_target():
                self.move_to_target()
                self.makeCenterTarget()
            self.allahuAkbar()
        
        self.get_logger().warning("Finish locking target!!!!!")
        self.hot_pub.publish(2)              # Publish 2, indicating that has shot all the ball


    def readRFID(self):
        if self.reader.read():
            return True
        return False

    def readButton(self):
        if GPIO.input(self.button_pin):
            return True
        return False
    
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
    

    ## Only use in makeCenterTarget, so time.sleep() should not cause any delay
    def turn(self, direction = 1): # 1 for anti-clockwise (turn left), -1 one for clockwise(turn_right)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = direction * rotatechange
        self.publisher_.publish(msg)
        time.sleep(1)        


    def hasFoundTarget(self):
        target_found = False
        for row in amg.pixels:
            for temp in row:
                if temp > heat_threshold:
                    target_found = True
        return target_found
        
    
    ## Adjust the robot to the middle of the hot target    
    def makeCenterTarget(self):
        centered = False

        while not centered:
            screen = amg.pixels
            max_column = -1
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(screen[row])):
                    current_value = screen[row][column]
                    if current_value > max_value:
                        max_column = column
                        max_value = current_value

            if not centered:
                # centre max value between row 3 and 4
                # Needs calibration during real run
                if max_column < 3:
                    # spin it anti-clockwise
                    self.turn(1)
                elif max_column > 4:
                    # spin it clockwise
                    self.turn(-1)
                else:
                    centered = True
                self.stopbot()
            return True

    # Redo this function for more robust operation
    def move_to_target(self):
        # move to the object in increments
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stopbot()

    def lock_target(self):
        for i in (3,4):
            for j in (3,4):
                if amg.pixels[i][j] > firing_threshold:
                    return True

        
    def allahuAkbar(self): ## Sorry I was to depressed to come up with a name
        # Start the DC motor to shoot the ball
        pwm1 = GPIO.PWM(self.motor1_pin, 1000)
        pwm2 = GPIO.PWM(self.motor2_pin, 1000)
        pwm1.start(60)
        pwm2.start(60)

        # # Start the Stepper Motor 2 seconds later

        # # Wait for 2 seconds
        time.sleep(10)

        # Start the stepper to load the ball

        # # Set up the Stepper Pins
        # control_pins = [26, 19, 13, 6]
        # for pin in control_pins:
        #     GPIO.setup(pin, GPIO.OUT)
        #     GPIO.output(pin, 0)

        # halfstep_seq = [
        #     [1, 0, 0, 0],
        #     [1, 1, 0, 0],
        #     [0, 1, 0, 0],
        #     [0, 1, 1, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 1, 1],
        #     [0, 0, 0, 1],
        #     [1, 0, 0, 1]]

        # self.get_logger().info("Started the Stepper")
        # # Start Spinning the Stepper
        # for i in range(512):
        #     for halfstep in range(8):
        #         for pin in range(4):
        #             GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
        #         time.sleep(0.001)


        # # Stop the DC Motor
        GPIO.output(self.dc_driver_en, GPIO.LOW)
        pwm1.stop()
        pwm2.stop()
        self.get_logger().info("Stopped the DC Motor")

        # # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)
    thermalcamera = Sensor()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermalcamera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()