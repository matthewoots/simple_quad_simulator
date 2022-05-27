#!/usr/bin/env python3

import rospy
import random
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point


pub0 = rospy.Publisher('/trajectory/points', JointTrajectory, queue_size=10)

def data(num):
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.joint_names.append("drone0")
    
    point = Point()
    point.z = 2.0
    jtp1 = JointTrajectoryPoint()
    jtp1.positions.append(point.x)
    jtp1.positions.append(point.y)
    jtp1.positions.append(point.z)
    jt.points.append(jtp1)

    for i in range(6):
        jtp = JointTrajectoryPoint()
        point = generate_random_point(point)
        jtp.positions.append(point.x)
        jtp.positions.append(point.y)
        jtp.positions.append(point.z)
        jt.points.append(jtp)

    return jt

def generate_random_point(prev):
    point = Point()
    point.x = prev.x + round(random.uniform(-2, 2), 2)
    point.y = prev.y + round(random.uniform(-2, 2), 2)
    point.z = 2.0 + round(random.uniform(-0.1, 0.1), 2)

    return point

def publisher():
    rospy.init_node('relocalization_publisher', anonymous=True)
    rate_hz = 0.5
    rate = rospy.Rate(rate_hz) # 10hz
    while not rospy.is_shutdown():
        
        pub0.publish(data(0))

        rate.sleep()


if __name__ == '__main__':
    publisher()