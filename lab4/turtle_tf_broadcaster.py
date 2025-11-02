#!/usr/bin/env python
import rospy
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TurtleTfBroadcaster:
    def __init__(self):
        rospy.init_node('turtle_tf_broadcaster')
        
        # Получаем имя черепахи из параметра
        self.turtle_name = rospy.get_param('~turtle_tf_name', 'turtle1')
        
        # Создаем TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Подписываемся на топик с позой черепахи
        pose_topic = rospy.get_param('~input_pose', f'/{self.turtle_name}/pose')
        rospy.Subscriber(pose_topic, Pose, self.pose_callback)
        
        rospy.loginfo(f"Turtle TF broadcaster started for {self.turtle_name}")
        
    def pose_callback(self, msg):
        # Создаем трансформацию
        transform = TransformStamped()
        
        # Заполняем заголовок
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = self.turtle_name
        
        # Позиция
        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0
        
        # Ориентация (кватернион из угла theta)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(transform)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster = TurtleTfBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass