#!/usr/bin/env python
import rospy
import tf
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TurtleCarrotBroadcaster:
    def __init__(self):
        rospy.init_node('turtle_carrot_broadcaster')
        
        # Параметры для вращения морковки
        self.carrot_radius = 2.0  # радиус вращения морковки вокруг черепахи
        self.carrot_frequency = 0.5  # частота вращения (Гц)
        self.start_time = rospy.Time.now().to_sec()
        
        # Создаем TF broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.carrot_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Подписываемся на топик с позой черепахи
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        rospy.loginfo("Turtle and Carrot TF broadcaster started!")
        
    def pose_callback(self, msg):
        current_time = rospy.Time.now()
        
        # 1. Публикуем трансформацию для черепахи (world -> turtle1)
        turtle_transform = TransformStamped()
        turtle_transform.header.stamp = current_time
        turtle_transform.header.frame_id = "world"
        turtle_transform.child_frame_id = "turtle1"
        
        # Позиция черепахи
        turtle_transform.transform.translation.x = msg.x
        turtle_transform.transform.translation.y = msg.y
        turtle_transform.transform.translation.z = 0.0
        
        # Ориентация черепахи
        quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        turtle_transform.transform.rotation.x = quaternion[0]
        turtle_transform.transform.rotation.y = quaternion[1]
        turtle_transform.transform.rotation.z = quaternion[2]
        turtle_transform.transform.rotation.w = quaternion[3]
        
        # 2. Публикуем трансформацию для морковки (turtle1 -> carrot)
        carrot_transform = TransformStamped()
        carrot_transform.header.stamp = current_time
        carrot_transform.header.frame_id = "turtle1"
        carrot_transform.child_frame_id = "carrot"
        
        # Вычисляем позицию морковки (вращение вокруг черепахи)
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        angle = 2 * math.pi * self.carrot_frequency * elapsed_time
        
        # Морковка вращается по кругу вокруг черепахи
        carrot_transform.transform.translation.x = self.carrot_radius * math.cos(angle)
        carrot_transform.transform.translation.y = self.carrot_radius * math.sin(angle)
        carrot_transform.transform.translation.z = 0.0
        
        # Морковка всегда "смотрит" наружу от круга
        carrot_orientation = angle + math.pi  # +π чтобы смотреть наружу
        carrot_quaternion = tf.transformations.quaternion_from_euler(0, 0, carrot_orientation)
        carrot_transform.transform.rotation.x = carrot_quaternion[0]
        carrot_transform.transform.rotation.y = carrot_quaternion[1]
        carrot_transform.transform.rotation.z = carrot_quaternion[2]
        carrot_transform.transform.rotation.w = carrot_quaternion[3]
        
        # Публикуем обе трансформации
        self.tf_broadcaster.sendTransform(turtle_transform)
        self.carrot_broadcaster.sendTransform(carrot_transform)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster = TurtleCarrotBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass