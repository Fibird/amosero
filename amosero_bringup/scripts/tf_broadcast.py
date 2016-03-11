#!/usr/bin/env python
#not used, created during testing 

import roslib; roslib.load_manifest('amosero_bringup')
import rospy
from math import cos
from math import sin
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import tf

PI = 3.141592
grad2rad = PI/180.0

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]

ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


br = tf.TransformBroadcaster()  

wheel_radius_cm = 3.5
wheel_diameter = 2 * 3.1416 * wheel_radius_cm;
wheel_max_rpm = 170
wheel_max_speed = 255

wheel_max_cm_meters_per_minute = wheel_diameter * wheel_max_rpm
wheel_max_meters_per_minute = wheel_max_cm_meters_per_minute / 100
wheel_max_meters_per_second = 0.6

def myround(x):
    return int(round(x) - .5) + (x > 0)


cur_linear_x = 0.0
cur_linear_y = 0.0
cur_linear_z = 0.0
cur_th = 0.0
cur_angular_z = 0.0
last_time = 0
cur_time = 0

ori_x = 0.0;
ori_y = 0.0;
ori_z = 0.0;
ori_w = 0.0;

def imuCb(msg):
    global ori_x,ori_y,ori_z,ori_w,ori_covariance
    ori_x = msg.orientation.x
    ori_y = msg.orientation.y
    ori_z = msg.orientation.z
    ori_w = msg.orientation.w
    ori_covariance = [999999,0,0,0,9999999,0,0,0,999999]

north = 0.0;

def northCb(msg):
    global north
    north=msg.data


def callback(msg):
    global cur_linear_x, cur_linear_y, cur_th,cur_angular_z,last_time,cur_time,cur_linear_z,odom_pub
    global left_motor_pub,left_motor_speed_pub,right_motor_pub,right_motor_speed_pub
    global ori_x,ori_y,ori_z,ori_w
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    calc_speed =  msg.linear.x
    if(msg.linear.x>wheel_max_meters_per_second):
        calc_speed = msg.linear.x
    normalised_speed = calc_speed / wheel_max_meters_per_second 
    total_speed = myround(normalised_speed*255)
    print "calc_speed: " + str(calc_speed) +" \t calc_speed: " + str(normalised_speed) + " \ttotal_speed: " + str(total_speed)
    #print "ori_x: " + str(ori_x) 
    rospy.logwarn("north: " + str(north));
    speed_left=0

    if(msg.linear.x>0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(255+abs(total_speed))
    elif(msg.linear.x<0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(abs(total_speed))
    elif(msg.linear.x==0):
        if(msg.angular.z > 0):
            left_motor_pub.publish(255)
        elif(msg.angular.z < 0):
            left_motor_pub.publish(511)
        else:
            left_motor_pub.publish(0)
            left_motor_speed_pub.publish(0)

    if(msg.linear.x>0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(255+abs(total_speed))
    elif(msg.linear.x<0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(abs(total_speed))
    elif(msg.linear.x==0):
        if(msg.angular.z < 0):
            right_motor_pub.publish(255)
        elif(msg.angular.z > 0):
            right_motor_pub.publish(511)
        else:
            right_motor_pub.publish(0)
            right_motor_pub.publish(0)

    
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


    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, cur_th)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, north)
    odom_quat = Quaternion(*odom_quat)


    odom_trans = geometry_msgs.msg.TransformStamped() 
    odom_trans.header.stamp = cur_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint"
    odom_trans.transform.translation.x = cur_linear_x;
    odom_trans.transform.translation.y = cur_linear_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    t = tf.Transformer(True, rospy.Duration(10.0))
    t.setTransform(odom_trans)

    #br.sendTransform(odom_trans);

    #TransformStamped
    odom = Odometry()
    odom.header.stamp = cur_time
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = delta_x;
    #odom.pose.pose.position.y = delta_y;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    if(ori_x != 0):
        #odom.pose.pose.orientation.x = ori_x 
        #odom.pose.pose.orientation.y = ori_y 
        #odom.pose.pose.orientation.z = ori_z
        #odom.pose.pose.orientation.x = ori_w
        #odom.pose.pose.orientation.covariance = ori_covariance;
        odom.pose.pose.orientation = odom_quat
    else:
        odom.pose.pose.orientation = odom_quat;
    
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = ori_w;
    odom.twist.twist.linear.y = ori_w;

    if(north!=0.0):
        odom.twist.twist.angular.z = north;
    else:
        odom.twist.twist.angular.z = vth;


    odom.pose.covariance = ODOM_POSE_COVARIANCE
    odom.twist.covariance = ODOM_TWIST_COVARIANCE

    odom_pub.publish(odom);
    last_time = cur_time



    #left_motor_pub.sendTransform((cur_linear_x, cur_linear_y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, cur_th),
    #                 rospy.Time.now(),
    #                 "base_link",
    #                 "odom")

    ###br.sendTransform((cur_linear_x, cur_linear_y, 0),
    ###                  tf.transformations.quaternion_from_euler(0, 0, cur_th),
    ###                 rospy.Time.now(),
    ###                 "base_link",
    ###                 "odom")
    #br.sendTransform((cur_linear_x, cur_linear_y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, cur_th),
    #                rospy.Time.now(),
    #                 "base_footprint",
    #                 "odom")
    #####br.sendTransform((cur_linear_x, cur_linear_y, 0),
    #####                 tf.transformations.quaternion_from_euler(0, 0, cur_th),
    #####                 rospy.Time.now(),
    #####                 "base_footprint",
    #####                 "map")
    #br.sendTransform((cur_linear_x, cur_linear_y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, cur_th),
    #                 rospy.Time.now(),
    #                 "base_link",
    #                 "map")

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
    rospy.Subscriber('imu_data',Imu,imuCb)
    rospy.Subscriber('north',std_msgs.msg.Float32,northCb)
    
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    left_motor_pub = rospy.Publisher('/left_motor', std_msgs.msg.Int32, queue_size=10)
    left_motor_speed_pub = rospy.Publisher('/left_motor_speed', std_msgs.msg.Int32, queue_size=10)
    right_motor_pub = rospy.Publisher('/right_motor', std_msgs.msg.Int32, queue_size=10)
    right_motor_speed_pub = rospy.Publisher('/right_motor_speed', std_msgs.msg.Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('amosero1')    
        last_time = rospy.Time.now();
        cur_time = rospy.Time.now(); 
        rospy.Rate(10) # 10hz

        #br.sendTransform((0, 0, 0),
        #             tf.transformations.quaternion_from_euler(0, 0, 0),
        #             rospy.Time.now(),
        #           "base_footprint",
        #             "camera_link")
        #br.sendTransform((0, 0, 0),
        #             tf.transformations.quaternion_from_euler(0, 0, 0),
        #             rospy.Time.now(),
        #             "base_footprint",
        #             "odom")
        listener()                      
    except rospy.ROSInterruptException: pass