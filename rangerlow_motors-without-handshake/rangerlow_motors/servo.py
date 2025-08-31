import adafruit_motor.servo as s
import time


class ServoWrapper():

    def __init__(self, channel, name:str) -> None:
        self.channel = channel
        self.servo = s.Servo(channel)
        self.name: str = name
        self.servo_angle:float = 0.0

    def getName(self) -> str:
        return self.name
        

    def setAngle(self, angle) -> None:
        self.servo.angle = angle      
        self.channel.duty_cycle = 0  

        self.servo_angle = angle
        time.sleep(1)

    def getAngle(self) -> float:
        return self.servo_angle  
        