#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Point
from mavros_msgs.msg import PositionTarget


pub0 = rospy.Publisher('/drone0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

def data(num, point):
    pt = PositionTarget()
    # geometry_msgs/Point position
    # geometry_msgs/Vector3 velocity
    # geometry_msgs/Vector3 acceleration_or_force
    # float32 yaw

    pt.header.stamp = rospy.Time.now()
    
    # in NWU frame
    pt.position = point

    pt.velocity.x = 0
    pt.velocity.y = 0
    pt.velocity.z = 0

    pt.acceleration_or_force.x = 0
    pt.acceleration_or_force.y = 0
    pt.acceleration_or_force.z = 0

    pt.yaw = 0

    return pt

def generate_random_point():
    point = Point()
    point.x = round(random.uniform(-10, 10), 2)
    point.y = round(random.uniform(-10, 10), 2)
    point.z = round(random.uniform(1, 2), 2)

    return point

def publisher():
    rospy.init_node('relocalization_publisher', anonymous=True)
    rate_hz = 10
    rate = rospy.Rate(rate_hz) # 10hz
    point = Point()
    point = generate_random_point()
    epoch = rospy.Time.now()
    while not rospy.is_shutdown():
        if (rospy.Time.now() - epoch).to_sec() > 5.0:
            point = generate_random_point()
            epoch = rospy.Time.now()
        
        pub0.publish(data(0, point))

        rate.sleep()


if __name__ == '__main__':
    publisher()