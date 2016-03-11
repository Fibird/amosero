#!/usr/bin/env python
#not used, created during testing 

import roslib; roslib.load_manifest('amosero_bringup')
import rospy
#import tf.transformations
from geometry_msgs.msg import *
from math import cos
from math import sin
from nav_msgs.msg import *
import tf

cur_linear_x = 0.0
cur_linear_y = 0.0
cur_linear_z = 0.0
cur_th = 0.0
cur_angular_z = 0.0
last_time = 0
cur_time = 0

br = tf.TransformBroadcaster()

def callback(msg):
    global cur_linear_x, cur_linear_y, cur_th,cur_angular_z,last_time,cur_time,cur_linear_z,odom_pub
    global left_motor_pub,left_motor_speed_pub,right_motor_pub,right_motor_speed_pub
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    if(msg.linear.x>0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(10)
            left_motor_speed_pub.publish(100)
    elif(msg.linear.x<0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(-10)
            left_motor_speed_pub.publish(100)
    else:
        left_motor_pub.publish(0)
        left_motor_speed_pub.publish(0)

    if(msg.linear.x>0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(10)
            right_motor_speed_pub.publish(100)
    elif(msg.linear.x<0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(-10)
            right_motor_speed_pub.publish(100)
    else:
        right_motor_pub.publish(0)
        right_motor_speed_pub.publish(0)

    
    cur_time = rospy.Time.now()
    
    if(last_time == 0):
        last_time = rospy.Time.now();
    
    dt = cur_time.secs - last_time.secs;
    vx = msg.linear.x
    vy = 0
    vth = msg.angular.z

    # v = s / t -> t = s / v  -> s = v * t 

    delta_x = (vx * cos(vth) - vy * sin(vth)) * dt
    delta_y = (vx * sin(vth) + vy * cos(vth)) * dt
    delta_th = vth * dt

    cur_linear_x += delta_x;
    cur_linear_y += delta_y;
    cur_th += delta_th;

    rospy.loginfo("cur: [%f, %f, %f]"%(cur_linear_x, cur_linear_y, delta_th))

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, cur_th)
    odom_quat = Quaternion(*odom_quat)


    #odom_trans = geometry_msgs.msg.TransformStamped() 
    #odom_trans.header.stamp = cur_time;
    #odom_trans.header.frame_id = "odom";
    #odom_trans.child_frame_id = "base_link"
    #odom_trans.transform.translation.x = cur_linear_x;
    #odom_trans.transform.translation.y = cur_linear_y;
    #odom_trans.transform.translation.z = 0.0;
    #odom_trans.transform.rotation = odom_quat;
    
    #t = tf.Transformer(True, rospy.Duration(10.0))
    #t.setTransform(odom_trans)

    #br.sendTransform(odom_trans);

    #TransformStamped
    odom = Odometry()
    odom.header.stamp = cur_time
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = delta_x;
    odom.pose.pose.position.y = delta_y;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);
    last_time = cur_time



    #left_motor_pub.sendTransform((cur_linear_x, cur_linear_y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, cur_th),
    #                 rospy.Time.now(),
    #                 "base_link",
    #                 "odom")

    br.sendTransform((cur_linear_x, cur_linear_y, 0),
                      tf.transformations.quaternion_from_euler(cur_linear_x, cur_linear_y, cur_th),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands

    #v_l = ...
    #v_r = ...

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    #wheel_left.set_speed(v_l)
    #wheel_right.set_speed(v_r)

def listener():
    global odom_pub,left_motor_pub,left_motor_speed_pub,right_motor_pub,right_motor_speed_pub
    rospy.Subscriber("/cmd_vel", Twist, callback)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    left_motor_pub = rospy.Publisher('/left_motor', std_msgs.msg.Int32, queue_size=10)
    left_motor_speed_pub = rospy.Publisher('/left_motor_speed', std_msgs.msg.Int32, queue_size=10)
    right_motor_pub = rospy.Publisher('/right_motor', std_msgs.msg.Int32, queue_size=10)
    right_motor_speed_pub = rospy.Publisher('/right_motor_speed', std_msgs.msg.Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    global br;
    try:
        rospy.init_node('amosero1')    
        last_time = rospy.Time.now();
        cur_time = rospy.Time.now();
        br = tf.TransformBroadcaster()   
        r = rospy.Rate(10) # 10hz

        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "camera_link")
        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
        listener()                      
    except rospy.ROSInterruptException: pass