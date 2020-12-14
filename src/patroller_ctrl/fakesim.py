#!/usr/bin/env python  

import roslib
roslib.load_manifest('patroller_ctrl')
import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

robot_0_init_pos = np.array([0,0])
robot_1_init_pos = np.array([0,0])
robot_0_init_ang = 0
robot_1_init_ang = 0

robot_0_pos = np.array([0,0])
robot_1_pos = np.array([0,0])
robot_0_ang = 0
robot_1_ang = 0
robot_0_vel = [0,0,0]
robot_1_vel = [0,0,0]

odom_pub_0 = None
odom_pub_1 = None
gt_pub_0 = None
gt_pub_1 = None

ls_pub_0 = None
ls_pub_1 = None

tf_br = None


def handle_cmd_vel(msg, robot):
    global robot_0_vel
    global robot_1_vel
    global robot_0_pos
    global robot_1_pos
    
    
    if (robot == 0):
        robot_0_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
#        print("0", robot_0_vel, robot_0_pos)
    else:
        robot_1_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
#        print("1", robot_1_vel, robot_1_pos)


def handle_cmd_vel_0(msg):
    handle_cmd_vel(msg, 0)

def handle_cmd_vel_1(msg):
    handle_cmd_vel(msg, 1)


def publish_positions():
    global tf_br
    global robot_0_init_pos
    global robot_1_init_pos
    global robot_0_pos
    global robot_1_pos
    global robot_0_ang
    global robot_1_ang
    global robot_0_init_ang
    global robot_1_init_ang
    global robot_0_vel
    global robot_1_vel

    global odom_pub_0
    global odom_pub_1
    global gt_pub_0
    global gt_pub_1
    

    
    
    r_0_odom = Odometry()
    r_0_odom.pose.pose.position.x = robot_0_pos[0]
    r_0_odom.pose.pose.position.y = robot_0_pos[1]
    r_0_odom.pose.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, robot_0_ang)
    r_0_odom.pose.pose.orientation = Quaternion(*q)
    r_0_odom.twist.twist.linear.x = robot_0_vel[0]
    r_0_odom.twist.twist.linear.y = robot_0_vel[1]
    r_0_odom.twist.twist.angular.z = robot_0_vel[2]
    r_0_odom.header.stamp = rospy.Time.now()
    r_0_odom.header.frame_id = "/robot_0/odom"

    r_1_odom = Odometry()
    r_1_odom.pose.pose.position.x = robot_1_pos[0]
    r_1_odom.pose.pose.position.y = robot_1_pos[1]
    r_1_odom.pose.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, robot_1_ang)
    r_1_odom.pose.pose.orientation = Quaternion(*q)
    r_1_odom.twist.twist.linear.x = robot_1_vel[0]
    r_1_odom.twist.twist.linear.y = robot_1_vel[1]
    r_1_odom.twist.twist.angular.z = robot_1_vel[2]
    r_1_odom.header.stamp = rospy.Time.now()
    r_1_odom.header.frame_id = "/robot_1/odom"
        
        
    odom_pub_0.publish(r_0_odom)
    odom_pub_1.publish(r_1_odom)

    r_0_odom = Odometry()
    r_0_odom.pose.pose.position.x = robot_0_pos[0] + robot_0_init_pos[0]
    r_0_odom.pose.pose.position.y = robot_0_pos[1] + robot_0_init_pos[1]
    r_0_odom.pose.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, robot_0_ang + robot_0_init_ang)
    r_0_odom.pose.pose.orientation = Quaternion(*q)
    r_0_odom.twist.twist.linear.x = robot_0_vel[0]
    r_0_odom.twist.twist.linear.y = robot_0_vel[1]
    r_0_odom.twist.twist.angular.z = robot_0_vel[2]
    r_0_odom.header.stamp = rospy.Time.now()
    r_0_odom.header.frame_id = "/robot_0/odom"

    r_1_odom = Odometry()
    r_1_odom.pose.pose.position.x = robot_1_pos[0] + robot_1_init_pos[0]
    r_1_odom.pose.pose.position.y = robot_1_pos[1] + robot_1_init_pos[1]
    r_1_odom.pose.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, robot_1_ang + robot_1_init_ang)
    r_1_odom.pose.pose.orientation = Quaternion(*q)
    r_1_odom.twist.twist.linear.x = robot_1_vel[0]
    r_1_odom.twist.twist.linear.y = robot_1_vel[1]
    r_1_odom.twist.twist.angular.z = robot_1_vel[2]
    r_1_odom.header.stamp = rospy.Time.now()
    r_1_odom.header.frame_id = "/robot_1/odom"
    
    
    gt_pub_0.publish(r_0_odom)
    gt_pub_1.publish(r_1_odom)

    tf_br.sendTransform((0, 0, .25),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/robot_0/base_laser_link",
                     "/robot_0/base_link")
    
    tf_br.sendTransform((0, 0, .25),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/robot_1/base_laser_link",
                     "/robot_1/base_link")
        
    tf_br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/robot_0/base_link",
                     "/robot_0/base_footprint")
    
    tf_br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/robot_1/base_link",
                     "/robot_1/base_footprint")
        
    
    tf_br.sendTransform((robot_0_pos[0], robot_0_pos[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, robot_0_ang),
                     rospy.Time.now(),
                     "/robot_0/base_footprint",
                     "/robot_0/odom")
    
    tf_br.sendTransform((robot_1_pos[0], robot_1_pos[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, robot_1_ang),
                     rospy.Time.now(),
                     "/robot_1/base_footprint",
                     "/robot_1/odom")


if __name__ == '__main__':
    rospy.init_node('fake_simulator')

# set initial poses    
    r_0_x = rospy.get_param("~robot_0_x")
    r_0_y = rospy.get_param("~robot_0_y")
    robot_0_init_ang = rospy.get_param("~robot_0_a")

    r_1_x = rospy.get_param("~robot_1_x")
    r_1_y = rospy.get_param("~robot_1_y")
    robot_1_init_ang = rospy.get_param("~robot_1_a")
    
    robot_0_init_pos = np.array([r_0_x, r_0_y])
    robot_1_init_pos = np.array([r_1_x, r_1_y])
    
# subscribe to cmd_vel
    rospy.Subscriber("/robot_0/cmd_vel", Twist, handle_cmd_vel_0)
    rospy.Subscriber("/robot_1/cmd_vel", Twist, handle_cmd_vel_1)
    
                     
# publish odom
    odom_pub_0 = rospy.Publisher("/robot_0/odom", Odometry)
    odom_pub_1 = rospy.Publisher("/robot_1/odom", Odometry)

# publish base_pose_ground_truth
    gt_pub_0 = rospy.Publisher("/robot_0/base_pose_ground_truth", Odometry)
    gt_pub_1 = rospy.Publisher("/robot_1/base_pose_ground_truth", Odometry)


# publish empty laser scans
    ls_pub_0 = rospy.Publisher("/robot_0/base_scan", LaserScan)
    ls_pub_1 = rospy.Publisher("/robot_1/base_scan", LaserScan)

# broadcast transforms

    tf_br = tf.TransformBroadcaster()
                     
    # every 10th of a second move each robot by their velocities and publish tf, odom, and gt messages
    
    r = rospy.Rate(10) # 10hz
    lastTime = rospy.get_time()
    while not rospy.is_shutdown():
        r.sleep()
        
        # add the rotation vel to the current rotation
        robot_0_ang += -robot_0_vel[2] * (lastTime - rospy.get_time())
        robot_1_ang += -robot_1_vel[2] * (lastTime - rospy.get_time())
        
        # build a rotation matrix
        
        r_mat_0 = np.array([[math.cos(robot_0_ang + robot_0_init_ang), - math.sin(robot_0_ang + robot_0_init_ang) ],[math.sin(robot_0_ang + robot_0_init_ang), math.cos(robot_0_ang + robot_0_init_ang)] ] )
        r_mat_1 = np.array([[math.cos(robot_1_ang + robot_1_init_ang), - math.sin(robot_1_ang + robot_1_init_ang) ],[math.sin(robot_1_ang + robot_1_init_ang), math.cos(robot_1_ang + robot_1_init_ang)] ] )
        
        vel_mat_0 = np.array([[robot_0_vel[0]], [robot_0_vel[1]] ])
        vel_mat_1 = np.array([[robot_1_vel[0]], [robot_1_vel[1]] ])
        
        # multiply the x and y vel by the rotation matrix
        # add the result to our position
#        print(r_mat_0 , vel_mat_0, (r_mat_0.dot(vel_mat_0 )), np.transpose((r_mat_0.dot(vel_mat_0)) * (lastTime - rospy.get_time()))[0], robot_0_pos)
        robot_0_pos = robot_0_pos + np.transpose((r_mat_0.dot(-vel_mat_0 )) * (lastTime - rospy.get_time()))[0]
        robot_1_pos = robot_1_pos + np.transpose((r_mat_1.dot(-vel_mat_1)) * (lastTime - rospy.get_time()))[0]
#        print(robot_0_pos, robot_1_pos)
        
        publish_positions()
        
        m = LaserScan()
        m.angle_min = -90
        m.angle_max = 90
        m.angle_increment = 3.14 / 100.0
        m.time_increment = 1/10.0
        m.range_min = 0
        m.range_max = 10
        m.ranges = [4 for i in range(100)]
        m.intensities = [104 for i in range(100)]
        m.header.frame_id = "/robot_0/base_laser_link"
        m.header.stamp = rospy.Time.now()
        
        ls_pub_0.publish(m)
        
        m = LaserScan()
        m.angle_min = -90
        m.angle_max = 90
        m.angle_increment = 3.14 / 100.0 
        m.time_increment = 1/10.0 / 100.0
        m.range_min = 0
        m.range_max = 10
        m.ranges = [4 for i in range(100)]
        m.intensities = [104 for i in range(100)]
        m.header.frame_id = "/robot_1/base_laser_link"
        m.header.stamp = rospy.Time.now()
        
        ls_pub_1.publish(m)
        
        lastTime = rospy.get_time()
        