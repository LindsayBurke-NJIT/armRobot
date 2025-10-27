#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

def stop():
    set_velocity.publish(0,0,0)
    
def drive_robot(velocity: int, angle: float, turn_rate: int, time: int):
    '''
    velocity: int in range -100 to 100 (in units of mm/s)
        - Negative values makes the motor rotate counter-clockwise
    angle: int in range 0 to 360 (in units of degrees)
        - 0 degrees means go right
        - 90 degrees means go forward
        - 180 degrees means go left
        - 270 degrees means go backwards
    turn_rate: int in range -2 to 2 (in units of 5 degrees/second)
        - How fast to yaw
        - Negative rotates clockwise)
    time: int (in units of seconds)
        - Time to drive for
    '''
    rospy.init_node('car_forward_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    startTime = time.time()

    while time.time()-startTime <= time:
        set_velocity.publish(velocity,angle,0)
        rospy.sleep(1)
        
    set_velocity.publish(0,0,0)
        
if __name__ == '__main__':
    drive_robot(60, 90, 0, 2)