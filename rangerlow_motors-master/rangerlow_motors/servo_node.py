#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
from .servo import ServoWrapper
from adafruit_pca9685 import PCA9685

from rangerros_interfaces.msg import Servo as ServoMsg

class ServoNode(Node):
    def __init__(self ) -> None:
        super().__init__('servo_node')

        # INIT SERVO DRIVER
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        
        self.front_left_servo = ServoWrapper(self.pca.channels[1], 'front_left_servo')
        self.rear_left_servo = ServoWrapper(self.pca.channels[2], 'rear_left_servo')

        self.front_right_servo = ServoWrapper(self.pca.channels[0], 'front_right_servo')
        self.rear_right_servo = ServoWrapper(self.pca.channels[3], 'rear_right_servo')

        # PUBLISHER

        self.front_left_servo_pub_ = self.create_publisher(ServoMsg, f'/rangerlow_servo/{self.front_left_servo.getName()}_info',10)
        self.rear_left_servo_pub_ = self.create_publisher(ServoMsg, f'/rangerlow_servo/{self.rear_left_servo.getName()}_info',10)

        self.front_right_servo_pub_ = self.create_publisher(ServoMsg, f'/rangerlow_servo/{self.front_right_servo.getName()}_info',10)
        self.rear_right_servo_pub_ = self.create_publisher(ServoMsg, f'/rangerlow_servo/{self.rear_right_servo.getName()}_info',10)

        # SUBSCRIBER

        self.front_left_servo_sub_ = self.create_subscription(ServoMsg, f'/rangerlow_servo/{self.front_left_servo.getName()}', self.front_left_servo_callback,10)
        self.rear_left_servo_sub_ = self.create_subscription(ServoMsg, f'/rangerlow_servo/{self.rear_left_servo.getName()}', self.rear_left_servo_callback,10) 

        self.front_right_servo_sub_ = self.create_subscription(ServoMsg, f'/rangerlow_servo/{self.front_right_servo.getName()}', self.front_right_servo_callback,10)
        self.rear_right_servo_sub_ = self.create_subscription(ServoMsg, f'/rangerlow_servo/{self.rear_right_servo.getName()}', self.rear_right_servo_callback,10) 

        # self init 
        self.front_left_servo_sub_
        self.rear_left_servo_sub_
        self.front_right_servo_sub_
        self.rear_right_servo_sub_

        # info timer 
        self.servo_info_timer = self.create_timer(0.05, self.servo_info_publisher)

    def front_left_servo_callback(self, msg: ServoMsg):
        self.front_left_servo.setAngle(msg.angle)
    
    def rear_left_servo_callback(self, msg: ServoMsg):
        self.rear_left_servo.setAngle(msg.angle)
    
    def front_right_servo_callback(self, msg: ServoMsg):
        self.front_right_servo.setAngle(msg.angle)
    
    def rear_right_servo_callback(self, msg: ServoMsg):
        self.rear_right_servo.setAngle(msg.angle)
     
    def servo_info_publisher(self):
        # msg info
        front_left_msg = ServoMsg()
        rear_left_msg = ServoMsg()

        front_right_msg = ServoMsg()
        rear_right_msg = ServoMsg()

        front_left_msg.servo_name = self.front_left_servo.getName()
        front_left_msg.angle = self.front_left_servo.getAngle()

        rear_left_msg.servo_name = self.rear_left_servo.getName()
        rear_left_msg.angle = self.rear_left_servo.getAngle()

        front_right_msg.servo_name = self.front_right_servo.getName()
        front_right_msg.angle = self.front_right_servo.getAngle()


        rear_right_msg.servo_name = self.rear_right_servo.getName()
        rear_right_msg.angle = self.rear_right_servo.getAngle()

        # msg publishing 
        
        self.front_left_servo_pub_.publish(front_left_msg)
        self.rear_left_servo_pub_.publish(rear_left_msg)

        self.front_right_servo_pub_.publish(front_right_msg)
        self.rear_right_servo_pub_.publish(rear_right_msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ServoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('NODE STOPPED')
    finally:
        node.destroy_node()
        rclpy.shutdown()


  