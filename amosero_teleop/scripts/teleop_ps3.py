#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import *

max_speed=255;

def myround(x):
    return int(round(x) - .5) + (x > 0)

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    global left_motor_pub,right_motor_pub,max_speed
    twist = Twist()
    #twist.linear.x = 4*data.axes[1]
    #twist.angular.z = 4*data.axes[0]
    print "Achse_1: " + str(data.axes[1])
    print "Achse_2: " + str(data.axes[3])

    if(data.buttons[5]==1):
      if(max_speed<255):
        max_speed=max_speed+5
    if(data.buttons[4]==1):
      if(max_speed>5):
        max_speed=max_speed-5

    #left_motor_pub.publish(255)
    #right_motor_pub.publish(255)
    normalised_speed_left = abs(data.axes[1]) / 1.0
    normalised_speed_right = abs(data.axes[3]) / 1.0

    if(data.axes[4]==-1.0):
        if(data.axes[1]>0.1):
           total_speed_left = myround(normalised_speed_left*max_speed+ 256)
           print "total_speed_left" + str(total_speed_left)
           left_motor_pub.publish(total_speed_left)

        if(data.axes[1]<-0.1):
           total_speed_left = myround(normalised_speed_left*max_speed)
           print "total_speed_left" + str(total_speed_left)
           left_motor_pub.publish(total_speed_left)
    else:
        left_motor_pub.publish(0)

    
    if(data.axes[5]==-1):
        if(data.axes[3]>0.1):
           total_speed_right = myround(normalised_speed_right*max_speed+ 256)
           print "total_speed_right" + str(total_speed_right)
           right_motor_pub.publish(total_speed_right)

        if(data.axes[3]<-0.1):
           total_speed_right = myround(normalised_speed_right*max_speed)
           print "total_speed_right" + str(total_speed_right)
           right_motor_pub.publish(total_speed_right)
    else:
        right_motor_pub.publish(0)
    print "max_speed: "+str(max_speed)

    pub.publish(twist)

# Intializes everything
def start():
    global left_motor_pub,right_motor_pub
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    left_motor_pub = rospy.Publisher('/left_motor', std_msgs.msg.Int32, queue_size=10)
    right_motor_pub = rospy.Publisher('/right_motor', std_msgs.msg.Int32, queue_size=10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()