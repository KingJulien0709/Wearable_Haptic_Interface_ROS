#!/usr/bin/python3
import os

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import json
import socket

ROS_MASTER_URI = "http://192.168.178.123:11311"
IP_ADDRESS = "192.168.178.31"
PORT = 5000

IMU_MSG_LEN = 128


def create_imu_msg_from_json(msg):
    try:
        string_msg = str(msg)
        data = json.loads(string_msg[string_msg.find("{"):string_msg.find("}") + 1])
        rospy.loginfo("Received data: %s", data)
        # quaternion = quaternion_from_euler(math.radians(data["r"]), math.radians(data["p"]), math.radians(data["h"]), 'sxyz')
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.linear_acceleration.x = data["x"]
        imu_msg.linear_acceleration.y = data["y"]
        imu_msg.linear_acceleration.z = data["z"]
        imu_msg.orientation.x = data["xq"]
        imu_msg.orientation.y = data["yq"]
        imu_msg.orientation.z = data["zq"]
        imu_msg.orientation.w = data["w"]
        imu_msg.header.stamp = rospy.Time.now()
        return imu_msg
    except Exception as e:
        rospy.logerr(e)
        return None


class HapticFeedbackNode:
    def __init__(self):
        self.socket_client = None
        self.active = True
        rospy.init_node('haptic_feedback', anonymous=True)
        self.pub = rospy.Publisher('/haptic_feedback/imu_data', Imu, queue_size=20)
        self.sub = rospy.Subscriber('/haptic_feedback/lra_motor_array', String, self.sub_callback_lra_data)

        self.socket_master = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_master.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_master.bind((IP_ADDRESS, PORT))
        self.socket_master.listen(1)
        rospy.loginfo("Waiting for connection")

        self.rate = rospy.Rate(50)  # 10hz

    def sub_callback_lra_data(self, msg):
        try:
            if self.socket_client is None:
                rospy.logerr("No socket client connected")
                return
            if msg is None:
                rospy.logerr("Could not create lra data message")
                return
            rospy.logdebug("Sending data: %s", msg)
            send_msg = str(msg).encode("utf-8")
            self.socket_client.send(send_msg)
        except socket.error as socket_e:
            rospy.logerr(socket_e)
            self.active = False  # if a socket error occurs, the client is disconnected to enable a reconnect
        except Exception as e:
            rospy.logerr(e)

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for connection from client")
            try:
                # FIXME: add timeout and error handling
                self.socket_client, address = self.socket_master.accept()
                self.socket_client.settimeout(5)
                rospy.loginfo("Connected to %s", address)
                self.active = True
                while self.active:
                    rospy.logdebug("Waiting for data")
                    result = self.socket_client.recv(IMU_MSG_LEN)
                    len_received = len(result)
                    if len_received < IMU_MSG_LEN:
                        result += self.socket_client.recv(IMU_MSG_LEN - len_received)
                    rospy.logdebug("Received data: %s", result)
                    imu_msg = create_imu_msg_from_json(result)
                    if imu_msg is None:
                        rospy.logerr("Could not create imu message")
                        continue
                    self.pub.publish(imu_msg)  # publish the imu message to the imu topic
                    self.rate.sleep()  # the sleep rate is set to 100Hz, which is the maximum rate of the imu sensor
                self.socket_client.close()  # will be executed if the while loop is exited due to an error
                self.socket_client = None  # will free up the socket for a new connection
            except Exception as e:
                print(e)
                if self.socket_client is not None:
                    self.socket_client.close()
                    self.socket_client = None
        self.socket_master.close()


if __name__ == '__main__':
    try:
        node = HapticFeedbackNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
