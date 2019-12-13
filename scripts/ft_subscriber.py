#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import socket 
from functools import partial


def on_ft_received(ft_data, connection):
    forces = ft_data.wrench.force
    torques = ft_data.wrench.torque
    payload = "({},{},{},{},{},{})".format(forces.x, forces.y, forces.z, torques.x, torques.y, torques.z)
    rospy.loginfo("FTStreamer: Sending payload {}".format(payload))
    try:
        connection.sendall(payload)
    except Exception:
        rospy.logerr("Could not send payload over socket")

def close_socket(sock):
    sock.close()

def main(): 
    rospy.init_node('ft_subscriber', log_level=rospy.INFO, anonymous=True)
    rospy.loginfo("Started FT subscriber")
    port = rospy.get_param("/ft_socket_port")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    s.bind(("192.168.178.21", port)) 
    s.listen(5)
    c, _ = s.accept() 
    rospy.Subscriber("ee_forces_torques", WrenchStamped, partial(on_ft_received, connection=c))
    rospy.on_shutdown(partial(close_socket, s))
    rospy.spin()

if __name__ == "__main__":
    main()