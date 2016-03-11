#!/usr/bin/env python

import roslib; roslib.load_manifest('lsm9ds0_imu_9dof')
import rospy

import serial
import string
import math

from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from math import atan;
from math import atan2;
from math import sqrt;
import tf
import os

rospy.init_node("node")
pub = rospy.Publisher('imu_data', Imu, queue_size=10)
north_pub = rospy.Publisher('north', std_msgs.msg.Float32, queue_size=10)

imuMsg = Imu()
imuMsg.orientation_covariance = [999999 , 0 , 0,
0, 9999999, 0,
0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,
0 , 99999, 0,
0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
0 , 0.2, 0,
0 , 0 , 0.2]


PI = 3.141592
grad2rad = PI/180.0

used_port='/dev/ttyACM0'

#find the last ttyACM0
for possible_port in os.listdir('/dev/'):
    if possible_port.startswith("ttyACM"):
        used_port='/dev/'+possible_port

port = rospy.get_param('device', used_port)

roll=0
pitch=0
yaw=0

while 1:
    try:
        ser = serial.Serial(port=port,baudrate=115200, timeout=1)
        line = ser.readline()
        print line
        words = string.split(line,",")    # Fields split
            #  G:, -1.75, -0.36, 1.07, A:, -0.13, 0.22, 0.99, M:, -0.78, 0.00, 0.67, Heading:, 179.89,Pitch Roll:, -6.07, 11.52
            #	0,	   1,	  2,    3,  4,     5,    6,    7,  8,     9,   10,   11,       12,     13,         14,    15,    16	
        if len(words) > 2:
            yaw = float(words[13])
            pitch = float(words[15]) * grad2rad
            roll = float(words[16])  * grad2rad
            

            mx = float(words[9]);
            my = float(words[10]);
            mz = float(words[11]);
          

            north_pub.publish((yaw+180) * grad2rad);

            imuMsg.linear_acceleration.x = float(words[1])
            imuMsg.linear_acceleration.y = float(words[2])
            imuMsg.linear_acceleration.z = float(words[3])
            
            imuMsg.angular_velocity.x = float(words[5])
            imuMsg.angular_velocity.y = float(words[6])
            imuMsg.angular_velocity.z = float(words[7])

            q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            imuMsg.orientation.x = q[0]
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]
            imuMsg.header.stamp= rospy.Time.now()
            imuMsg.header.frame_id = 'base_footprint'
            pub.publish(imuMsg)


            print "y: " + str(yaw) + " p: " + str(pitch) + " r: " +  str(roll)
            print "if" 
        else:
            raise Exception("Not the expected format! wtf?")   
        print line
    except KeyboardInterrupt:
        exit()
    except:
        #reset that sucker
        #raise
        ser.close()
        ser = serial.Serial()
        ser.port=port
        ser.baudrate=1200
        ser.open(); ser.close()
        print "resetting!"
        rospy.sleep(11)

ser.close
