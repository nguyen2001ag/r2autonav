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
firing_threshold = 40.0


class Sensor(Node):
    def __init__(self):
        super().__init__('shooter')
        
        # self.actuation_subscriber = self.create_subscription(Int8, 'actuation', self.actuation_callback, 10)
        # self.actuation_subscriber

        
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
        self.dc_driver_en = 6
        self.motor1_pin = 12
        self.motor2_pin = 13
        self.button_pin = 17

        # Set up the Stepper Pins
        self.stepper_pins = [22, 23, 24, 27]
        for pin in self.stepper_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        self.get_logger().info("Setup the Stepper Motor")

        # DC motor setup
        GPIO.setup(self.dc_driver_en, GPIO.OUT)
        GPIO.output(self.dc_driver_en, False)
        GPIO.setup(self.motor1_pin, GPIO.OUT)
        GPIO.setup(self.motor2_pin, GPIO.OUT)
        self.pwm1 = GPIO.PWM(self.motor1_pin, 1000)
        self.pwm2 = GPIO.PWM(self.motor2_pin, 1000)
        self.get_logger().info("Setup the DC")
        
        
    
        
    def allahuAkbar(self): ## Sorry I was to depressed to come up with a name

        GPIO.output(self.dc_driver_en, True)
        self.pwm1.start(40)
        self.pwm2.start(40)
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
        GPIO.output(self.dc_driver_en, GPIO.LOW)
        self.pwm1.stop()
        self.pwm2.stop()
        self.get_logger().info("Stopped the DC Motor")

        # # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)
    shooter = Sensor()
    try:
        # rclpy.init(args=args)
        # thermalcamera = Sensor()
        # thermalcamera.hasFoundTarget()
        shooter.allahuAkbar()
        # rclpy.spin(thermalcamera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        print("Reached exception")
        shooter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()