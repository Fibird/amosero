#!/usr/bin/env python
#not used, created during testing 

import roslib; roslib.load_manifest('amosero_bringup')
import rospy
from math import atan;
from math import atan2;
from math import sqrt;
import tf

from sensor_msgs.msg import *

PI = 3.141592
grad2rad = PI/180.0

l_pitch =[];
l_roll =[];
l_yaw =[];
avg_pitch = 0.0
avg_roll = 0.0
avg_yaw = 0.0
test_yaw = 0;

def imuCb(msg):
    global l_pitch,l_roll,l_yaw,avg_pitch,avg_roll,avg_yaw,test_yaw
    yaw = 0.0
    new_yaw = 0.0
    print "x: "+str(msg.orientation.x)+" y: "+str(msg.orientation.y)+" z: "+str(msg.orientation.z);

    pitch = atan2(msg.orientation.x, sqrt(msg.orientation.y * msg.orientation.y) + (msg.orientation.z * msg.orientation.z));
    roll = atan2(msg.orientation.y,  sqrt(msg.orientation.x * msg.orientation.x) + (msg.orientation.z * msg.orientation.z));
    #pitch = pitch * 180.0 / PI;
    #roll = roll *180.0 / PI;
    
    if(msg.orientation.y > 0):
        #yaw = (90 - (atan(msg.orientation.x / msg.orientation.y) * (180 / PI))) * grad2rad
        yaw = ((atan(msg.orientation.x / msg.orientation.y) * (180 / PI))) * grad2rad
    elif(msg.orientation.y < 0):
        #yaw = (-(atan(msg.orientation.x / msg.orientation.y) * (180 / PI))) * grad2rad
        yaw = ((atan(msg.orientation.x / msg.orientation.y) * (180 / PI))) * grad2rad
    else:
        if(msg.orientation.y < 0):
            yaw = 180 * grad2rad
        else:
            yaw = 0 * grad2rad
    
    new_yaw =  (atan(msg.orientation.x / msg.orientation.y))

    #print "yaw: "+str(yaw)+" \t new_yaw: "+str(new_yaw)+" \t pitch: "+str(pitch) +" \t roll: "+str(roll)

    msg.angular_velocity.x = float(msg.angular_velocity.x);
    msg.angular_velocity.y = float(msg.angular_velocity.y);
    msg.angular_velocity.z = float(msg.angular_velocity.z);

    north_pub.publish(yaw);
    
    if(test_yaw < 1 and test_yaw > 0):
        msg.orientation.w = test_yaw
        test_yaw=test_yaw+0.01
    elif(test_yaw <= 0):
        msg.orientation.w = test_yaw
        test_yaw=test_yaw+0.01
    else:
        test_yaw=-1


    #q = tf.transformations.quaternion_from_euler(roll - avg_roll,pitch - avg_pitch,yaw - avg_yaw);
    q = tf.transformations.quaternion_from_euler(roll - avg_roll,pitch - avg_pitch,yaw);
    #q = tf.transformations.quaternion_from_euler(roll - avg_roll,pitch - avg_pitch,test_yaw);
    print  " \t roll: "+str(roll - avg_roll) + "\t pitch: "+str(pitch - avg_pitch)  + "\t yaw: "+str(yaw - avg_yaw)
    #q = tf.transformations.quaternion_from_rpy(roll - avg_roll,pitch - avg_pitch,new_yaw - avg_yaw);
    if(len(l_yaw) < 100):
        l_pitch.append(pitch);
        l_roll.append(roll)
        l_yaw.append(yaw)
        avg_pitch = reduce(lambda x, y: x + y, l_pitch) / float(len(l_pitch))
        avg_roll = reduce(lambda x, y: x + y, l_roll) / float(len(l_roll))
        avg_yaw = reduce(lambda x, y: x + y, l_yaw) / float(len(l_yaw))
    else:
        print "calibrated!"

    
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];
    




    msg.header.frame_id = 'base_footprint'


    msg.orientation_covariance = [999999,0,0,0,9999999,0,0,0,999999]
    msg.angular_velocity_covariance = [9999,0,0,0,99999,0,0,0,0.02]
    msg.linear_acceleration_covariance =  [0.2,0,0,0,0.2,0,0,0,0.2]
    imu_pub.publish(msg);

if __name__ == "__main__":
    rospy.init_node('imu_calc', anonymous=True) #make node 
    rospy.Subscriber('imu_data_raw',Imu,imuCb)
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    north_pub = rospy.Publisher('/north', std_msgs.msg.Float32, queue_size=10)
    print "spinning!"
    rospy.spin()