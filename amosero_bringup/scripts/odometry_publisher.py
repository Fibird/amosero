#!/usr/bin/env python
#not used, created during testing 

import numpy as np
import rospy
import roslib
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import PyKDL as kdl

roslib.load_manifest('amosero_bringup')
rospy.init_node('talker', anonymous=True)

class RosOdomPublisher:
    gps_ekf_odom_pub = rospy.Publisher('odom',Odometry, queue_size=10)
    tf_br = tf.TransformBroadcaster()

    publish_odom_tf = True

    x=0
    y=0
    z=0

    R=0.0
    P=0.0
    Y=0.0

    frame_id = '/odom'
    child_frame_id = '/base_link'

    def publish_odom(self):
      msg = Odometry()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = self.frame_id # i.e. '/odom'
      msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

      msg.pose.pose.position = Point(self.x, self.y, self.z)
      msg.pose.pose.orientation = Quaternion()

      pos = (msg.pose.pose.position.x,
             msg.pose.pose.position.y,
             msg.pose.pose.position.z)

      ori = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)

      # Publish odometry message
      self.gps_ekf_odom_pub.publish(msg)

      # Also publish tf if necessary
      if self.publish_odom_tf:
          self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

if __name__ == '__main__':
    try:
        pub = RosOdomPublisher()
        br = tf.TransformBroadcaster()

        
        
        r = rospy.Rate(1) # 10hz
        pub.x = 0.0
        pub.y = 0.0
        pub.z = 0.0
        while not rospy.is_shutdown():
            print "x: " + str(pub.x) +"y: " + str(pub.y) +"z: " + str(pub.z);
            if(pub.x < 5):
                pub.x = pub.x+0.03
            else:
                pub.x = 0
            pub.publish_odom();  
            br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "odom",
                     "world")
            br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "camera_link")
            br.sendTransform((pub.x, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")               
            r.sleep()            
    except rospy.ROSInterruptException: pass