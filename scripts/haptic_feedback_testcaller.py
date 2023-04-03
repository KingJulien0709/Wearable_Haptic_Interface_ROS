#!/usr/bin/python3
import json
import random
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

MSG_PER_SECOND = 50

DRV_MSG_LEN = 63  # space for the zero byte at the end for the client

def callback_imu_data(msg):
    rospy.logdebug("Received data: %s", msg)


def create_lra_data_msg(arr):
    data_dict = {}
    for i in range(5):
        data_dict.setdefault(str(i),arr[i])
    jsonstring = json.dumps(data_dict)
    msg_len = len(jsonstring)
    if msg_len < DRV_MSG_LEN:
        jsonstring += " " * (DRV_MSG_LEN - msg_len)
    return jsonstring


def haptic_feedback_tester():
    pub = rospy.Publisher('/haptic_feedback/lra_motor_array', String, queue_size=MSG_PER_SECOND)
    sub = rospy.Subscriber('/haptic_feedback/imu_data', Imu, callback_imu_data)
    rospy.init_node('haptic_feedback_tester', anonymous=True)
    rate = rospy.Rate(MSG_PER_SECOND)  # 20hz
    while not rospy.is_shutdown():
        try:
            msg = create_lra_data_msg([random.randint(0, 255) for _ in range(5)])
            rospy.logdebug("Sending data: %s", msg)
            pub.publish(msg)
            rate.sleep()
        except Exception as e:
            rospy.logerr(e)


if __name__ == '__main__':
    try:
        haptic_feedback_tester()
    except rospy.ROSInterruptException:
        pass
