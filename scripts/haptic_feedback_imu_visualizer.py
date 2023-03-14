#!/usr/bin/python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node('map_frame')

    # Define the transformation from "map" to "world" frame
    trans = TransformStamped()
    trans.header.frame_id = "imu_frame"
    trans.child_frame_id = "map"
    trans.transform.translation.x = 0.0
    trans.transform.translation.y = 0.0
    trans.transform.translation.z = 0.0
    trans.transform.rotation.x = 0.0
    trans.transform.rotation.y = 0.0
    trans.transform.rotation.z = 0.0
    trans.transform.rotation.w = 1.0

    # Publish the transformation using a TF2 broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(trans)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass