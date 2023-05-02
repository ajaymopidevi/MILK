import rospy
from state_machine.msg import MotorCommand

from topics import POLULU_TOPIC

class PoluluCommand:
    SERVO_JOINT = 'servo'
    MOTOR_JOINT = 'brushless_motor'

    def __init__(self):
        self.pub = rospy.Publisher(POLULU_TOPIC, MotorCommand, queue_size=10)
    
    def send_command(self, joint_name, pos, speed=0):
        motor = MotorCommand()
        motor.joint_name = joint_name # Whether servo or motor
        motor.position = pos 
        motor.speed = speed 
        motor.acceleration = 1.0
        self.pub.publish(motor)
    
    def send_servo_command(self, pos, speed=0):
        self.send_command(joint_name=self.SERVO_JOINT, pos=pos, speed=speed)
    
    def send_motor_command(self, pos, speed=0):
        self.send_command(joint_name=self.MOTOR_JOINT, pos=pos, speed=speed)
    

polulu = PoluluCommand()