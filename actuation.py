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

heat_threshold = 35.0
firing_threshold = 36.7


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

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #####################################################
        #                   PINOUT                          #
        # GPIO 6: Button Enable                             #
        # GPIO 12 (PWM1) : DC Motor1                        #
        # GPIO 13 (PWM1) : DC Motor2                        #
        # GPIO 22, 23, 24, 27: Stepper Motor Control        #
        # GPIO 26: Button                                   #          
        # GPIO 2, 3: SDA, SCL (I2C) for IR camera           #
        # GPIO 10, 09, 11, 08: MOSI, MISO, SCK, SS for NFC  #
        # GPIO 25: RST for NFC RC522                        #                     
        #                                                   #
        #####################################################

        GPIO.setmode(GPIO.BCM)

        self.button_en = 6
        self.motor1_pin = 12
        self.motor2_pin = 13
        self.button_pin = 26

        # Set up the Stepper Pins
        self.stepper_pins = [22, 23, 24, 27]
        for pin in self.stepper_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        self.get_logger().info("Setup the Stepper Motor")

        self.reader = SimpleMFRC522()
        self.get_logger().info("Setup the NFC Reader")

        # DC motor setup
        GPIO.setup(self.button_en, GPIO.OUT)
        GPIO.output(self.button_en, True)
        GPIO.setup(self.motor1_pin, GPIO.OUT)
        GPIO.setup(self.motor2_pin, GPIO.OUT)
        self.pwm1 = GPIO.PWM(self.motor1_pin, 1000)
        self.pwm2 = GPIO.PWM(self.motor2_pin, 1000)
        self.get_logger().info("Setup the DC")

        self.target_found = False
        self.button_read = False
        self.nfc_found = False


        # Button setup
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.get_logger().info("Setup Button")

        # Set up Thermal Camera
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(self.i2c)
        self.get_logger().info("Setup AMG Thermal Camera")


        self.fsm = 3

        

    # targeting_status callback function to stop wallfollower logic when target is detected
    def timer_callback(self):
        if self.fsm == 1:
            self.get_logger().info("Finding NFC")
            if self.nfc_found:
                self.get_logger().info("NFC Tag found, Changing to next state")
                self.nfc_found = True
                self.fsm = 2
                msg = Int8()
                msg.data = 1
                self.nfc_pub.publish(msg)          # Publish 1, indicating receive NFC signal
            else:
                self.readRFID()
                self.get_logger().info("Reading NFC tag")

        elif self.fsm == 2:
            self.get_logger().info("Waiting for Button")    
            if self.button_read:
                self.get_logger().warn("Button read, Changing to next state")
                self.button_read = True
                msg = Int8()
                msg.data = 1
                self.fsm = 3
                self.button_pub.publish(msg)       # Publish 1, indicating user has press the button
            else:
                self.readButton()

        elif self.fsm == 3:
            self.get_logger().info("Finding Hot target")
            if self.target_found:
                self.get_logger().info("Hot targer found, Changing to next state")
                msg = Int8()
                msg.data = 1
                self.fsm = 4
                self.hot_pub.publish(msg)          # Publish 1, indicating have detected hot signal
            else:
                self.hasFoundTarget()
                self.get_logger().info("Finding hot target!!")
        
        

    def actuation_callback(self, msg):
        flag = msg.data
        if flag == 1:
            # centre the target
            # self.makeCenterTarget()

            # while not self.lock_target():
            #     self.move_to_target()
            #     self.makeCenterTarget()
            self.get_logger().warn("Changing to state 2: Finding NFC")
            self.fsm = 1
        elif flag == 2:
            self.get_logger().warn("Attempting to shoot at target")
            while not self.lock_target():
                self.makeCenterTarget()
                self.move_to_target()
            self.allahuAkbar()
            self.get_logger().warning("Finish shooting at target!!!!!")
            msg = Int8()
            msg.data = 2
            self.hot_pub.publish(msg)              # Publish 2, indicating that has shot all the ball


    def readRFID(self):
        id, data = self.reader.read_no_block()
        if id:
            self.get_logger().info("Read NFC with ID : %s" % str(id))
            self.nfc_found = True
        # print("Reading NFC")


    def readButton(self):
        # self.get_logger().info("Waiting for your command!!!!!")
        if GPIO.input(self.button_pin):
            self.get_logger().info("Button read, Received command to move")
            GPIO.output(self.button_en, False)
            self.button_read = True
    
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
        for row in self.amg.pixels:
            # print('[', end=" ")
            for temp in row:
                if temp > heat_threshold:
                    self.target_found = True
                # print("{0:.1f}".format(temp), end=" ")
            # print("]")
            # print("\n")

        
    
    ## Adjust the robot to the middle of the hot target    
    def makeCenterTarget(self):
        centered = False

        while not centered:
            screen = self.amg.pixels
            max_row = -1
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(screen[row])):
                    current_value = screen[row][column]
                    if current_value > max_value:
                        max_row = row
                        max_value = current_value

            if not centered:
                # centre max value between row 3 and 4
                # Needs calibration during real run
                if max_row < 3:
                    # spin it clockwise, turn right
                    self.get_logger().info("Turning right to adjust")
                    self.turn(-1)
                elif max_row > 4:
                    # spin it anti-clockwise, turn left
                    self.get_logger().info("Turning left to adjust")
                    self.turn(1)
                else:
                    centered = True
                self.stopbot()
            return True

    # Redo this function for more robust operation
    def move_to_target(self):
        # move to the object in increments
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stopbot()

    def lock_target(self):
        for i in (3,4):
            for j in (3,4):
                if self.amg.pixels[i][j] > firing_threshold:
                    return True

        
    def allahuAkbar(self): ## Sorry I was to depressed to come up with a name
        GPIO.output(self.button_en, True)
        self.pwm1.start(50)
        self.pwm2.start(50)
        self.get_logger().info("Started the Shooter")
        # # Wait for 2 seconds
        time.sleep(5)

        # Start the stepper to load the ball

        # careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
        step_sleep = 0.005
        direction = False
        motor_step_counter = 0
        step_sequence = [[1,1,0,0],
                        [0,1,1,0],
                        [0,0,1,1],
                        [1,0,0,1]]

        nSteps = 3072  # 5.625*(1/64) per step, 4096 steps is 360°
        self.get_logger().info("Started the Stepper")

        for i in range(nSteps):
            for pin in range(4):
                GPIO.output( self.stepper_pins[pin], step_sequence[motor_step_counter][pin] )
            if direction==True:
                motor_step_counter = (motor_step_counter - 1) % 4
            elif direction==False:
                motor_step_counter = (motor_step_counter + 1) % 4
            time.sleep( step_sleep )


        ## Stop Stepper Motor
        for i in range(4):
            GPIO.output(self.stepper_pins[i], GPIO.LOW)

        # # Stop the DC Motor
        GPIO.output(self.button_en, GPIO.LOW)
        self.pwm1.stop()
        self.pwm2.stop()
        self.get_logger().info("Stopped the DC Motor")

        # # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


    



def main(args=None):
    rclpy.init(args=args)
    thermalcamera = Sensor()
    try:
        # rclpy.init(args=args)
        # thermalcamera = Sensor()
        # thermalcamera.hasFoundTarget()
        rclpy.spin(thermalcamera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        print("Reached exception")
        thermalcamera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()