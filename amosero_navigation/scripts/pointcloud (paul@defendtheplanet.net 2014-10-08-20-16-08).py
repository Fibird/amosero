#!/usr/bin/env python
# 3D Cheese source

import rospy
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *



data_list = [];



#listener
def listen():
    global pub, data_list, rospy
    rospy.init_node('listen', anonymous=True)
    pub = rospy.Publisher('camera/depth/points_replay', PointCloud2, queue_size=10)  
    rospy.Subscriber("points_photo_trigger", std_msgs.msg.Int32, callback_photo)

def callback_photo(data) :
    global pub,data_list, sub      
    sub = rospy.Subscriber("camera/depth/points", PointCloud2, callback_listen)

def callback_listen(data) :
    global pub,data_list,sub    
    data_list.append(data)
    rospy.loginfo("photo taken "+ str(len(data_list)))

    sub.unregister()


if __name__ == '__main__':
    try:   	
        listen()
        while not rospy.is_shutdown():
            for data in data_list:
                pub.publish(data)    
                print "publishing data!"
            data_list = data_list[100:]
    except rospy.ROSInterruptException:
        pass