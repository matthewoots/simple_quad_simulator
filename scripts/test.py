#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget


pub0 = rospy.Publisher('/drone0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

def data(num):
    pt = PositionTarget()
    # geometry_msgs/Point position
    # geometry_msgs/Vector3 velocity
    # geometry_msgs/Vector3 acceleration_or_force
    # float32 yaw

    pt.header.stamp = rospy.Time.now()
    
    # in NWU frame
    pt.position.x = -5
    pt.position.y = -5
    pt.position.z = 1

    pt.velocity.x = 0
    pt.velocity.y = 0
    pt.velocity.z = 0

    pt.acceleration_or_force.x = 0
    pt.acceleration_or_force.y = 0
    pt.acceleration_or_force.z = 0

    pt.yaw = 0

    return pt

    

def publisher():
    rospy.init_node('relocalization_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        pub0.publish(data(0))
        rate.sleep()


if __name__ == '__main__':
    publisher()