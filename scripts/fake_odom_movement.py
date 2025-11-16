#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
import tf

def fake_odom_movement():
    rospy.init_node('fake_odom_movement')
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10)  # 10Hz
    
    t = 0
    while not rospy.is_shutdown():
        # Движение по кругу для демонстрации
        x = 2.0 * math.cos(t * 0.1)
        y = 2.0 * math.sin(t * 0.1)
        theta = t * 0.1
        
        # Публикуем одометрию
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        
        pub.publish(odom)
        
        # Публикуем TF odom->base_footprint
        tf_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )
        
        t += 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_odom_movement()
    except rospy.ROSInterruptException:
        pass