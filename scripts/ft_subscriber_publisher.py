#!/usr/bin/env python

"""
Node listening for WrenchStamped messages on a topic, and republishing the forces and torques over a socket.
Acts as a socket *server*, e.g. clients can connect to it and receive string-encoded FT info.
"""

import os
from urlparse import urlparse

import rospy
from geometry_msgs.msg import WrenchStamped
import socket


def _get_my_ip():
    ros_master_uri = urlparse(os.environ["ROS_MASTER_URI"])
    return ros_master_uri.hostname


class FTSubscriberPublisher(object):
    def __init__(self):
        rospy.init_node('ft_subscriber_publisher', log_level=rospy.INFO, anonymous=True)
        port = rospy.get_param("~ft_socket_port")
        topic = rospy.get_param("~ft_topic")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = _get_my_ip()
        self.socket.bind((host, port))
        rospy.loginfo("FTSubscriberPublisher: Listening for incoming connections on {}:{}".format(host, port))
        self.socket.settimeout(0.1)
        self.connection = None
        self.ft_sub = rospy.Subscriber(topic, WrenchStamped, self.on_ft_received)
        rospy.on_shutdown(self.close_socket)

    def close_socket(self):
        self.socket.close()

    def run(self):
        self.socket.listen(1)
        rate = rospy.Rate(125)  # 125 Hz --> publish force every 8 ms
        while not rospy.is_shutdown():
            if self.connection is None:
                # rospy.loginfo("Waiting for connection...")
                try:
                    self.connection, _ = self.socket.accept()
                    rospy.loginfo("FTSubscriberPublisher: Socket connected")
                except socket.timeout:
                    pass
            rate.sleep()

    def on_ft_received(self, ft_data):
        if self.connection is None:
            return
        forces = ft_data.wrench.force
        torques = ft_data.wrench.torque
        payload = "({},{},{},{},{},{})".format(forces.x, forces.y, forces.z, torques.x, torques.y, torques.z)
        rospy.logdebug("FTSubscriberPublisher: Sending payload {}".format(payload))
        try:
            self.connection.sendall(payload)
        except Exception:
            rospy.logerr("FTSubscriberPublisher: Could not send payload over socket. Maybe the remote end disconnected?")
            self.connection = None


if __name__ == "__main__":
    ft_sub_pub = FTSubscriberPublisher()
    ft_sub_pub.run()
