#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math

def fake_laser_publisher():
    rospy.init_node('fake_laser_publisher')
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    
    rate = rospy.Rate(5)  # 5Hz
    
    count = 0
    while not rospy.is_shutdown():
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser_link"
        
        scan.angle_min = -math.pi/2
        scan.angle_max = math.pi/2
        scan.angle_increment = math.pi/180
        scan.time_increment = 0.0
        scan.scan_time = 0.2
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Генерируем fake данные - имитируем комнату
        ranges = []
        for i in range(180):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Имитируем стены в форме квадрата
            if abs(angle) < math.pi/6:
                # Передняя стена
                distance = 2.0 + 0.1 * math.sin(count * 0.1)
            elif angle > math.pi/6:
                # Правая стена
                distance = 1.5 / math.cos(angle - math.pi/2)
            else:
                # Левая стена
                distance = 1.5 / math.cos(-angle - math.pi/2)
                
            # Ограничиваем максимальное расстояние
            distance = min(distance, 10.0)
            ranges.append(distance)
                
        scan.ranges = ranges
        pub.publish(scan)
        
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_laser_publisher()
    except rospy.ROSInterruptException:
        pass